#include "LSGeo_renderer.h"

#include <tinyexr.h>
#include <gl_utility/sh_utils.h>

#include <gflags/gflags.h>
DEFINE_double(lsgeo_mesomap_size, 2048.0, "mesomap size");

void LSGeoRenderParams::init(GLProgram& prog)
{
    prog.createUniform("u_enable_mask", DataType::UINT);
    prog.createUniform("u_cull_occlusion", DataType::UINT);

    prog.createUniform("u_cull_offset", DataType::FLOAT);
    prog.createUniform("u_uv_view", DataType::UINT);
    
    prog.createUniform("u_mesomap_size", DataType::FLOAT);
    
    prog.createUniform("u_light_pos1", DataType::VECTOR3);
    prog.createUniform("u_light_pos2", DataType::VECTOR3);
    prog.createUniform("u_light_pos3", DataType::VECTOR3);
}

void LSGeoRenderParams::update(GLProgram& prog)
{
    prog.setUniformData("u_enable_mask", (uint)enable_mask);
    prog.setUniformData("u_cull_occlusion", (uint)enable_cull_occlusion);
    
    prog.setUniformData("u_cull_offset", cull_offset);
    prog.setUniformData("u_uv_view", (uint)uv_view);
    
    prog.setUniformData("u_mesomap_size", mesomap_size);
    
    prog.setUniformData("u_light_pos1", light_pos1);
    prog.setUniformData("u_light_pos2", light_pos2);
    prog.setUniformData("u_light_pos3", light_pos3);
}

#ifdef WITH_IMGUI
void LSGeoRenderParams::updateIMGUI()
{
    ImGui::Checkbox("mask", &enable_mask);
    ImGui::Checkbox("uv view", &uv_view);
    ImGui::Checkbox("cull occlusion", &enable_cull_occlusion);
    ImGui::SliderFloat("Transparency", &alpha, 0.0, 1.0);
    ImGui::SliderFloat("cullOffset", &cull_offset, -1.0, 0.0);
    ImGui::SliderFloat3("light pos1", &light_pos1[0], -100.0, 100.0);
    ImGui::SliderFloat3("light pos2", &light_pos2[0], -100.0, 100.0);
    ImGui::SliderFloat3("light pos3", &light_pos3[0], -100.0, 100.0);
    
    const char* listbox_items[] = {"color"};
    ImGui::ListBox("RenderTarget", &location, listbox_items, 1);
}
#endif

void LSGeoRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr model)
{
    programs_["main"] = GLProgram(shader_dir, "lightstage.vert", "ls_geo.frag", DrawMode::TRIANGLES);
    programs_["depth"] = GLProgram(shader_dir, "depthmap.vert", "depthmap.frag", DrawMode::TRIANGLES);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog_main = programs_["main"];
    auto& prog_depth = programs_["depth"];
    auto& prog_pl = programs_["plane"];
    
    param_.enable_mask = true;
    param_.mesomap_size = FLAGS_lsgeo_mesomap_size;
    
    param_.init(prog_main);
    prog_main.createTexture("u_sample_mask", data_dir + "deepls_mask.png");
    fb_ = Framebuffer::Create(1, 1, RT_NAMES::count); // will be resized based on frame size
    fb_depth_ = Framebuffer::Create(1, 1, 0);
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    
    Camera::initializeUniforms(prog_main, U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW | U_CAMERA_WORLD | U_CAMERA_POS);
    Camera::initializeUniforms(prog_depth, U_CAMERA_MVP);
    
    mesh_.init(prog_main, AT_POSITION | AT_NORMAL | AT_UV | AT_TAN);
    mesh_.init(prog_depth, AT_POSITION);
    
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog_main, AT_UV);
    
    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_->color(RT_NAMES::color), fb_->width(), fb_->height());
    
    prog_main.createTexture("u_sample_depth", fb_depth_->depth(), fb_depth_->width(), fb_depth_->height());    
    prog_main.createTexture("u_sample_disp", 0, 0, 0);
}

void LSGeoRenderer::render(const Camera& camera, const FaceData& fd)
{
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;

    if((param_.sub_samp*w != fb_->width()) || (param_.sub_samp*h != fb_->height()))
        fb_->Resize(param_.sub_samp*w, param_.sub_samp*h, RT_NAMES::count);
    if((param_.sub_samp*w != fb_depth_->width()) || (param_.sub_samp*h != fb_depth_->height()))
        fb_depth_->Resize(param_.sub_samp*w, param_.sub_samp*h, 0);

    // render parameters update
    auto& prog_main = programs_["main"];
    auto& prog_pl = programs_["plane"];
    auto& prog_depth = programs_["depth"];
    
    param_.update(prog_main);
    
    const auto& maps = fd.maps();
    assert(maps.size() != 0);
    prog_main.updateTexture("u_sample_disp", (GLuint)maps[0]);
    
    prog_pl.setUniformData("u_alpha", param_.alpha);

    // camera parameters update
    camera.updateUniforms(prog_main, fd.getRT(), U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW | U_CAMERA_WORLD | U_CAMERA_POS);
    camera.updateUniforms(prog_depth, fd.getRT(), U_CAMERA_MVP);
    
    // update mesh attributes
    Eigen::MatrixX3f tan, btan;
    calcTangent(tan, btan, fd.pts_, fd.uvs(), fd.tripts(), fd.triuv());

    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_tangent(tan, btan, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update_uv(fd.uvs(), fd.triuv());

    mesh_.update(prog_main, AT_POSITION | AT_NORMAL | AT_UV | AT_TAN);
    mesh_.update(prog_depth, AT_POSITION);
    
    // NOTE: need to make sure the viewport size matches the framebuffer size
    fb_depth_->Bind();
    glViewport(0, 0, fb_depth_->width(), fb_depth_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    prog_depth.draw();
    fb_depth_->Unbind();
    
    // draw mesh
    fb_->Bind();
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    prog_main.draw(wire_);

    fb_->Unbind();
    
    prog_pl.updateTexture("u_texture", fb_->color((uint)param_.location));
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
}

#ifdef FACE_TOOLKIT
void LSGeoRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    if(show_)
        render(result.cameras[cam_id], result.fd[frame_id]);
}
#endif

#ifdef WITH_IMGUI
void LSGeoRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        param_.updateIMGUI();
    }
}
#endif

RendererHandle LSGeoRenderer::Create(std::string name, bool show)
{
    auto renderer = new LSGeoRenderer(name, show);
    
    return RendererHandle(renderer);
}

