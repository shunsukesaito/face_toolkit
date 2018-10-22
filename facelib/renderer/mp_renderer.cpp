//
//  mesh_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "mp_renderer.h"

void MPRenderer::init(std::string shader_dir,
                      const Eigen::MatrixX3i& tripts,
                      const Eigen::MatrixX3i& triuv,
                      const Eigen::MatrixX2f& uv,
                      const cv::Mat& img)
{
    programs_["mp_init"] = GLProgram(shader_dir, "mp_init.vert", "mp_init.frag", DrawMode::TRIANGLES);
    programs_["mp_main"] = GLProgram(shader_dir, "mp_main.vert", "mp_main.frag", DrawMode::TRIANGLES);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog_init = programs_["mp_init"];
    auto& prog_main = programs_["mp_main"];
    auto& prog_pl = programs_["plane"];
    
    fb_tex_ = Framebuffer::Create(4096, 4096, 1); // will be resized based on frame size
    fb_plane_ = Framebuffer::Create(1, 1, 1); // will be resized based on frame size
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    
    Camera::initializeUniforms(prog_init, U_CAMERA_MVP);
    Camera::initializeUniforms(prog_main, U_CAMERA_MVP | U_CAMERA_MV);

    mesh_.update_tri(tripts);
    mesh_.init(prog_init, AT_POSITION | AT_UV);
    mesh_.init(prog_main, AT_POSITION | AT_NORMAL | AT_UV);
    
    mesh_.update_uv(uv, triuv);
    mesh_.update(prog_init, AT_UV);
    mesh_.update(prog_main, AT_UV);
    
    prog_init.createTexture("u_image", img);
    prog_main.createTexture("u_texture", fb_tex_->color(0), fb_tex_->width(), fb_tex_->height());

    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_plane_->color(0), fb_plane_->width(), fb_plane_->height());
}

void MPRenderer::render(const Camera& camera,
                        const Eigen::MatrixX3i& tri,
                        const Eigen::VectorXf& pts,
                        const Eigen::MatrixX3f& nml,
                        const cv::Mat& img)
{
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;

    if((sub_samp_*w != fb_plane_->width()) || (sub_samp_*h != fb_plane_->height()))
        fb_plane_->Resize(sub_samp_*w, sub_samp_*h, 1);
    
    auto& prog_init = programs_["mp_init"];
    auto& prog_main = programs_["mp_main"];
    auto& prog_pl = programs_["plane"];
    
    camera.updateUniforms(prog_init, U_CAMERA_MVP);
    camera.updateUniforms(prog_main, U_CAMERA_MVP | U_CAMERA_MV);

    mesh_.update_position(pts,tri);
    mesh_.update_normal(nml,tri);
    mesh_.update(prog_init, AT_POSITION);
    mesh_.update(prog_main, AT_POSITION | AT_NORMAL);
    
    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_plane_->color(0));

    if (texture_update_){
        if(!img.empty())
            prog_init.updateTexture("u_image", img);
        fb_tex_->Bind();
        glViewport(0, 0, fb_tex_->width(), fb_tex_->height());
        clearBuffer(COLOR::COLOR_ALPHA);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glEnable(GL_CULL_FACE);
        prog_init.draw(wire_);
        fb_tex_->Unbind();
    }
    
    fb_plane_->Bind();
    glViewport(0, 0, fb_plane_->width(), fb_plane_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    prog_main.draw(wire_);
    fb_plane_->Unbind();
    
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
}

void MPRenderer::render(const Camera& camera,
                        const Eigen::Matrix4f& RT,
                        const Eigen::MatrixX3i& tri,
                        const Eigen::VectorXf& pts,
                        const Eigen::MatrixX3f& nml,
                        const cv::Mat& img)
{
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;

    if((sub_samp_*w != fb_plane_->width()) || (sub_samp_*h != fb_plane_->height()))
        fb_plane_->Resize(sub_samp_*w, sub_samp_*h, 1);

    auto& prog_init = programs_["mp_init"];
    auto& prog_main = programs_["mp_main"];
    auto& prog_pl = programs_["plane"];
    
    camera.updateUniforms(prog_init, RT, U_CAMERA_MVP);
    camera.updateUniforms(prog_main, RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update_position(pts,tri);
    mesh_.update_normal(nml,tri);
    mesh_.update(prog_init, AT_POSITION);
    mesh_.update(prog_main, AT_POSITION | AT_NORMAL);

    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_plane_->color(0));

    if (texture_update_){
        if(!img.empty())
            prog_init.updateTexture("u_image", img);
        fb_tex_->Bind();
        glViewport(0, 0, fb_tex_->width(), fb_tex_->height());
        clearBuffer(COLOR::COLOR_ALPHA);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glEnable(GL_CULL_FACE);
        prog_init.draw(wire_);
        fb_tex_->Unbind();
    }
    
    fb_plane_->Bind();
    glViewport(0, 0, fb_plane_->width(), fb_plane_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    prog_main.draw(wire_);
    fb_plane_->Unbind();
    
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
}

#ifdef FACE_TOOLKIT
void MPRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr fm)
{
    init(shader_dir,fm->tri_pts_,fm->tri_uv_,fm->uvs_);
}

void MPRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    if(show_)
        render(result.cameras[cam_id], result.fd[frame_id].getRT(), result.fd[frame_id].tripts(),
               result.fd[frame_id].pts_, result.fd[frame_id].nml_, result.cap_data[frame_id][cam_id].img_);
}
#endif

#ifdef WITH_IMGUI
void MPRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::Checkbox("tex update", &texture_update_);
        ImGui::SliderFloat("Transparency", &alpha_, 0.0, 1.0);
    }
}
#endif

RendererHandle MPRenderer::Create(std::string name, bool show)
{
    auto renderer = new MPRenderer(name, show);
    
    return RendererHandle(renderer);
}
