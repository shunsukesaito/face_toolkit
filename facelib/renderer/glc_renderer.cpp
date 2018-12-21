//
//  glc_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "glc_renderer.h"

void GLCRenderer::init(std::string shader_dir,
                       const Eigen::MatrixX3i &tri)
{
    programs_["mesh"] = GLProgram(shader_dir, "glc.vert", "glc.frag", DrawMode::TRIANGLES_IDX);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);

    auto &prog = programs_["mesh"];
    auto &prog_pl = programs_["plane"];

    fb_ = Framebuffer::Create(1, 1, 1); // will be resized based on frame size

    prog_pl.createUniform("u_alpha", DataType::FLOAT);

    GLCCamera::initializeUniforms(prog, U_GLCCAMERA_MV);

    mesh_.update_tri(tri);
    mesh_.init(prog, AT_POSITION | AT_NORMAL | AT_TRI);

    plane_.init(prog_pl, 0.5);
    prog_pl.createTexture("u_texture", fb_->color(0), fb_->width(), fb_->height());
}

void GLCRenderer::render(const GLCCamera &camera,
                         const Eigen::VectorXf &pts,
                         const Eigen::MatrixX3f &nml,
                         const Eigen::Matrix4f &RT)
{
    if(!show_)
        return;
    
    GLFWwindow *window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;

    if ((sub_samp_ * w != fb_->width()) || (sub_samp_ * h != fb_->height()))
        fb_->Resize(sub_samp_ * w, sub_samp_ * h, 1);

    auto &prog = programs_["mesh"];
    auto &prog_pl = programs_["plane"];

    camera.updateUniforms(prog, RT, U_GLCCAMERA_MV);

    mesh_.update_position(pts);
    mesh_.update_normal(nml);
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_TRI);

    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_->color(0));

    fb_->Bind();
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
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

void GLCRenderer::render(const std::vector<GLCCamera> &cameras,
                         const Eigen::VectorXf &pts,
                         const Eigen::MatrixX3f &nml,
                         const Eigen::Matrix4f &RT)
{
    if(!show_)
        return;
    
    if (cameras.size() == 0)
        return;

    GLFWwindow *window = glfwGetCurrentContext();
    int w = cameras[0].width_;
    int h = cameras[0].height_;

    if ((sub_samp_ * w != fb_->width()) || (sub_samp_ * h != fb_->height()))
        fb_->Resize(sub_samp_ * w, sub_samp_ * h, 1);

    auto &prog = programs_["mesh"];
    auto &prog_pl = programs_["plane"];

    mesh_.update_position(pts);
    mesh_.update_normal(nml);
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_TRI);
    
    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_->color(0));

    fb_->Bind();
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    for (int i = 0; i < cameras.size(); ++i)
    {
        cameras[i].updateUniforms(prog, RT, U_GLCCAMERA_MV);
        prog.draw(wire_);
    }
    fb_->Unbind();

    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
}

// #ifdef FACE_TOOLKIT
// void GLCRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr fm)
// {
//     init(shader_dir,fm->tri_pts_);
// }

// void GLCRenderer::render(const FaceResult& result, int cam_id, int frame_id)
// {
//     if(show_)
//         render(result.cameras[cam_id], result.fd[frame_id].getRT(), result.fd[frame_id].pts_, result.fd[frame_id].nml_);
// }
// #endif

#ifdef WITH_IMGUI
void GLCRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str()))
    {
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::SliderFloat("Transparency", &alpha_, 0.0, 1.0);
        //        ImGui::InputInt("TessInner", &tessInner_);
        //        ImGui::InputInt("TessOuter", &tessOuter_);
        //        ImGui::SliderFloat("TessAlpha", &tessAlpha_, 0.0, 1.0);
    }
}
#endif

RendererHandle GLCRenderer::Create(std::string name, bool show)
{
    auto renderer = new GLCRenderer(name, show);

    return RendererHandle(renderer);
}
