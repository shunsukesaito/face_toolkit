//
//  posmap_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "posmap_renderer.h"

#include <utility/exr_loader.h>

// constants
#include <gflags/gflags.h>
DEFINE_uint32(pm_size, 256, "size of geometry_image");
DEFINE_uint32(pm_tessin, 3, "number of tessellation in inner loop");
DEFINE_uint32(pm_tessout, 3, "number of tessellation in outer loop");
DEFINE_string(pmrec_file, "", "pos map file name under asset folder");

void PosMapRenderer::init(std::string data_dir,
                          std::string shader_dir,
                          FaceModelPtr model)
{
    programs_["main"] = GLProgram(shader_dir, "subdiv.vert", "subdiv.tc", "subdiv.te", "", "subdiv.frag", DrawMode::PATCHES);
    
    programs_["recon"] = GLProgram(shader_dir, "posmap_recon.vert", "posmap_recon.tc", "posmap_recon.te", "", "posmap_recon.frag", DrawMode::PATCHES);
    
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog = programs_["main"];
    auto& prog_rec = programs_["recon"];
    auto& prog_pl = programs_["plane"];
    
    fb_ = Framebuffer::Create(FLAGS_pm_size, FLAGS_pm_size, 2, GL_LINEAR); // will be resized based on frame size
    
    prog.createUniform("u_tessinner", DataType::FLOAT);
    prog.createUniform("u_tessouter", DataType::FLOAT);
    prog.createUniform("u_tessalpha", DataType::FLOAT);
    
    
    mesh_.init(prog, AT_POSITION | AT_NORMAL | AT_UV);
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog, AT_UV);
    
    prog_rec.createUniform("u_tessinner", DataType::FLOAT);
    prog_rec.createUniform("u_tessouter", DataType::FLOAT);
    prog_rec.createUniform("u_delta", DataType::FLOAT);
    
    mesh_.init(prog_rec, AT_UV);
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog_rec, AT_UV);
    Camera::initializeUniforms(prog_rec, U_CAMERA_MVP | U_CAMERA_MV);
    
    prog_rec.createTexture("u_sample_pos", fb_->color(0), fb_->width(), fb_->height());
    
    plane_.init(prog_pl,0.01);
    prog_pl.createTexture("u_texture", fb_->color(0), fb_->width(), fb_->height());
    
    tessInner_ = FLAGS_pm_tessin;
    tessOuter_ = FLAGS_pm_tessout;
}

void PosMapRenderer::render(const FaceData& fd, std::string out_name)
{
    // render parameters update
    auto& prog = programs_["main"];
    auto& prog_pl = programs_["plane"];

    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_tessalpha", tessAlpha_);

    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw(wire_);
    fb_->Unbind();
    
    std::vector<cv::Mat_<cv::Vec4f>> out;
    fb_->RetrieveFBO(FLAGS_pm_size, FLAGS_pm_size, out);
    
    GLFWwindow* window = glfwGetCurrentContext();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    
    cv::Mat_<cv::Vec3f> pos, dIx, dIy;
    cv::cvtColor(out[0], pos, cv::COLOR_BGRA2BGR);
    cv::Scharr(pos, dIx, CV_32F, 1, 0);
    cv::Scharr(pos, dIy, CV_32F, 0, 1);
    
    cv::Mat_<cv::Vec3f> nml(pos.size());
    cv::imwrite(out_name, pos);
}

void PosMapRenderer::render(const Camera& camera, const FaceData& fd)
{
    // render parameters update
    auto& prog = programs_["main"];
    auto& prog_rec = programs_["recon"];
    
    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_tessalpha", tessAlpha_);
    
    prog_rec.setUniformData("u_tessinner", (float)tessInner_);
    prog_rec.setUniformData("u_tessouter", (float)tessOuter_);
    prog_rec.setUniformData("u_delta", delta_);
    camera.updateUniforms(prog_rec, fd.RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw(wire_);
    fb_->Unbind();
    
    GLFWwindow* window = glfwGetCurrentContext();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog_rec.draw(wire_);
}


void PosMapRenderer::render(int w, int h, const FaceData& fd, std::vector<cv::Mat_<cv::Vec4f>>& out)
{
    if(w != fb_->width() || h != fb_->height())
        fb_->Resize(w, h, 2);
    
    auto& prog = programs_["main"];
    
    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_tessalpha", tessAlpha_);
    
    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw(wire_);
    
    fb_->Unbind();
    
    fb_->RetrieveFBO(w, h, out);
}

#ifdef FACE_TOOLKIT
void PosMapRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    if(show_)
        render(result.cameras[cam_id], result.fd[frame_id]);
}
#endif

#ifdef WITH_IMGUI
void PosMapRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        const char* listbox_items[] = { "positions", "normals"};
        ImGui::ListBox("RenderTarget", &location_, listbox_items, 2);
        ImGui::InputInt("TessInner", &tessInner_);
        ImGui::InputInt("TessOuter", &tessOuter_);
        ImGui::SliderFloat("TessAlpha", &tessAlpha_, 0.0, 1.0);
        ImGui::InputFloat("Delta", &delta_);
    }
}
#endif

RendererHandle PosMapRenderer::Create(std::string name, bool show)
{
    auto renderer = new PosMapRenderer(name, show);
    
    return RendererHandle(renderer);
}

void PosMapReconRenderer::init(std::string data_dir,
                               std::string shader_dir,
                               FaceModelPtr model)
{
    programs_["main"] = GLProgram(shader_dir,"posmap_recon.vert", "posmap_recon.tc", "posmap_recon.te", "", "posmap_recon.frag", DrawMode::PATCHES);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    fb_ = Framebuffer::Create(1, 1, 1); // will be resized based on frame size
    
    auto& prog = programs_["main"];
    auto& prog_pl = programs_["plane"];
    
    prog.createUniform("u_tessinner", DataType::FLOAT);
    prog.createUniform("u_tessouter", DataType::FLOAT);
    prog.createUniform("u_delta", DataType::FLOAT);
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);

    mesh_.init(prog, AT_UV);
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog, AT_UV);
    Camera::initializeUniforms(prog, U_CAMERA_MVP | U_CAMERA_MV);
    
    cv::Mat posmap;
    loadEXRToCV(FLAGS_pmrec_file, posmap);
    cv::cvtColor(posmap,posmap,CV_BGRA2RGB);
    prog.createTexture("u_sample_pos", posmap);
    
    tessInner_ = FLAGS_pm_tessin;
    tessOuter_ = FLAGS_pm_tessout;
    
    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_->color(0), fb_->width(), fb_->height());
}

void PosMapReconRenderer::render(const Camera& camera, const FaceData& fd)
{
    if((sub_samp_*camera.width_ != fb_->width()) || (sub_samp_*camera.height_ != fb_->height()))
        fb_->Resize(sub_samp_*camera.width_, sub_samp_*camera.height_, 1);
    
    // render parameters update
    auto& prog = programs_["main"];
    auto& prog_pl = programs_["plane"];
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_delta", delta_);
    
    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_->color(0));
    
    camera.updateUniforms(prog, fd.RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    int w, h;
    GLFWwindow* window = glfwGetCurrentContext();
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    
    fb_->Bind();
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    prog.draw(wire_);
    
    fb_->Unbind();
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
}


#ifdef FACE_TOOLKIT
void PosMapReconRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    if(show_)
        render(result.cameras[cam_id], result.fd[frame_id]);
}
#endif

#ifdef WITH_IMGUI
void PosMapReconRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::SliderFloat("Transparency", &alpha_, 0.0, 1.0);
        ImGui::InputInt("TessInner", &tessInner_);
        ImGui::InputInt("TessOuter", &tessOuter_);
        ImGui::InputFloat("Delta", &delta_);
    }
}
#endif

RendererHandle PosMapReconRenderer::Create(std::string name, bool show)
{
    auto renderer = new PosMapReconRenderer(name, show);
    
    return RendererHandle(renderer);
}

