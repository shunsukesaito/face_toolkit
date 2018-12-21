#include "differedLS_renderer.h"

#include <tinyexr.h>
#include <gl_utility/sh_utils.h>

#include <gflags/gflags.h>
DEFINE_double(dls_mesomap_size, 2048.0, "mesomap size");

void DifferedLSRenderParams::init(GLProgram& prog)
{
    prog.createUniform("u_enable_mask", DataType::UINT);
    prog.createUniform("u_mesomap_size", DataType::FLOAT);
}

void DifferedLSRenderParams::update(GLProgram& prog)
{
    prog.setUniformData("u_enable_mask", (uint)enable_mask);
    prog.setUniformData("u_mesomap_size", mesomap_size);
}

#ifdef WITH_IMGUI
void DifferedLSRenderParams::updateIMGUI()
{
    ImGui::Checkbox("mask", &enable_mask);
    ImGui::SliderFloat("Transparency", &alpha, 0.0, 1.0);
    
    const char* listbox_items[] = {"all", "diff albedo", "spec albedo", "normal", "uv"};
    ImGui::ListBox("RenderTarget", &location, listbox_items, 4);
}
#endif

void DifferedLSRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr model)
{
    programs_["main"] = GLProgram(shader_dir, "differed_ls.vert", "differed_ls.frag", DrawMode::TRIANGLES);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog_main = programs_["main"];
    auto& prog_pl = programs_["plane"];
    
    param_.enable_mask = true;
    param_.mesomap_size = FLAGS_dls_mesomap_size;
    
    param_.init(prog_main);
    prog_main.createTexture("u_sample_mask", data_dir + "deepls_mask.png");
    fb_ = Framebuffer::Create(1, 1, RT_NAMES::count); // will be resized based on frame size
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    
    Camera::initializeUniforms(prog_main, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.init(prog_main, AT_POSITION | AT_NORMAL | AT_UV | AT_TAN);
    
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog_main, AT_UV);
    
    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_->color(RT_NAMES::diff_albedo), fb_->width(), fb_->height());
    
    prog_main.createTexture("u_sample_diff_albedo", 0, 0, 0);
    prog_main.createTexture("u_sample_spec_albedo", 0, 0, 0);
    prog_main.createTexture("u_sample_disp", 0, 0, 0);
}

void DifferedLSRenderer::render(const Camera& camera, const FaceData& fd)
{
    if(!show_)
        return;
    
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;

    if((param_.sub_samp*w != fb_->width()) || (param_.sub_samp*h != fb_->height()))
        fb_->Resize(param_.sub_samp*w, param_.sub_samp*h, RT_NAMES::count);

    // render parameters update
    auto& prog_main = programs_["main"];
    auto& prog_pl = programs_["plane"];
    
    param_.update(prog_main);
    
    const auto& maps = fd.maps();
    assert(maps.size() != 0);
    prog_main.updateTexture("u_sample_disp", (GLuint)maps[0]);
    prog_main.updateTexture("u_sample_diff_albedo", (GLuint)maps[1]);
    prog_main.updateTexture("u_sample_spec_albedo", (GLuint)maps[2]);
    
    prog_pl.setUniformData("u_alpha", param_.alpha);

    // camera parameters update
    camera.updateUniforms(prog_main, fd.RT(), U_CAMERA_MVP | U_CAMERA_MV);
    
    // update mesh attributes
    Eigen::MatrixX3f tan, btan;
    calcTangent(tan, btan, fd.pts_, fd.uvs(), fd.tripts(), fd.triuv());

    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_tangent(tan, btan, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update_uv(fd.uvs(), fd.triuv());

    mesh_.update(prog_main, AT_POSITION | AT_NORMAL | AT_UV | AT_TAN);
    
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
void DifferedLSRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    render(result.cameras[cam_id], result.fd[frame_id]);
}
#endif

#ifdef WITH_IMGUI
void DifferedLSRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        param_.updateIMGUI();
    }
}
#endif

RendererHandle DifferedLSRenderer::Create(std::string name, bool show)
{
    auto renderer = new DifferedLSRenderer(name, show);
    
    return RendererHandle(renderer);
}

