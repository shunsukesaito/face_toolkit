//
//  mesh_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "hairmesh_renderer.h"

void HairMeshRenderer::init(std::string shader_dir,
                            const Eigen::MatrixX3i& tri_t,
                            const Eigen::MatrixX3i& tri_h)
{
    programs_["mesh"] = GLProgram(shader_dir, "hairmesh.vert", "hairmesh.frag", DrawMode::TRIANGLES_IDX);
    programs_["depth"] = GLProgram(shader_dir, "depthmap.vert", "depthmap.frag", DrawMode::TRIANGLES_IDX);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog = programs_["mesh"];
    auto& prog_depth = programs_["depth"];
    auto& prog_pl = programs_["plane"];
    
    fb_ = Framebuffer::Create(1, 1, 2); // will be resized based on frame size
    fb_depth_ = Framebuffer::Create(1, 1, 0);
    
    Camera::initializeUniforms(prog, U_CAMERA_MVP | U_CAMERA_MV);
    Camera::initializeUniforms(prog_depth, U_CAMERA_MVP);

    mesh_hair_.update_tri(tri_h);
    mesh_hair_.init(prog, AT_POSITION | AT_NORMAL | AT_TRI);
    mesh_torso_.update_tri(tri_t);
    mesh_torso_.init(prog_depth, AT_POSITION | AT_TRI);
    
//    prog.createUniform("u_bias", DataType::FLOAT);
    prog.createTexture("u_sample_depth", fb_depth_->depth(), fb_depth_->width(), fb_depth_->height());
    
    plane_.init(prog_pl,0.5);
    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    prog_pl.createTexture("u_texture", fb_->color(0), fb_->width(), fb_->height());
}

void HairMeshRenderer::render(const Camera& camera,
                              const Eigen::VectorXf& pts_t,
                              const Eigen::VectorXf& pts_h,
                              const Eigen::MatrixX3f& nml)
{
    int w, h;
    GLFWwindow* window = glfwGetCurrentContext();
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    
    if((w != fb_->width()) || (h != fb_->height()))
        fb_->Resize(w, h, 2);
    if((w != fb_depth_->width()) || (h != fb_depth_->height()))
        fb_depth_->Resize(w, h, 0);
    
    auto& prog = programs_["mesh"];
    auto& prog_pl = programs_["plane"];
    auto& prog_depth = programs_["depth"];
    
    camera.updateUniforms(prog, U_CAMERA_MVP | U_CAMERA_MV);
    camera.updateUniforms(prog_depth, U_CAMERA_MVP);
    
    mesh_hair_.update_position(pts_h);
    mesh_hair_.update_normal(nml);
    mesh_hair_.update(prog, AT_POSITION | AT_NORMAL);
    
    mesh_torso_.update_position(pts_t);
    mesh_torso_.update(prog_depth, AT_POSITION);
    
//    prog.setUniformData("u_bias", bias_);
    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", 0);
    
    // NOTE: need to make sure the viewport size matches the framebuffer size
    fb_depth_->Bind();
    glViewport(0, 0, fb_depth_->width(), fb_depth_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    prog_depth.draw();
    fb_depth_->Unbind();
    
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

void HairMeshRenderer::render(const Camera& camera,
                              const Eigen::Matrix4f& RT,
                              const Eigen::VectorXf& pts_t,
                              const Eigen::VectorXf& pts_h,
                              const Eigen::MatrixX3f& nml)
{
    int w, h;
    GLFWwindow* window = glfwGetCurrentContext();
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    
    if((w != fb_->width()) || (h != fb_->height()))
        fb_->Resize(w, h, 2);
    if((w != fb_depth_->width()) || (h != fb_depth_->height()))
        fb_depth_->Resize(w, h, 0);
    
    auto& prog = programs_["mesh"];
    auto& prog_pl = programs_["plane"];
    auto& prog_depth = programs_["depth"];
    
    camera.updateUniforms(prog, RT, U_CAMERA_MVP | U_CAMERA_MV);
    camera.updateUniforms(prog_depth, RT, U_CAMERA_MVP);
    
    mesh_hair_.update_position(pts_h);
    mesh_hair_.update_normal(nml);
    mesh_hair_.update(prog, AT_POSITION | AT_NORMAL);
    
    mesh_torso_.update_position(pts_t);
    mesh_torso_.update(prog_depth, AT_POSITION);

//    prog.setUniformData("u_bias", bias_);
    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_->color(0));
    
    // NOTE: need to make sure the viewport size matches the framebuffer size
    fb_depth_->Bind();
    glViewport(0, 0, fb_depth_->width(), fb_depth_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    prog_depth.draw();
    fb_depth_->Unbind();

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

#ifdef WITH_IMGUI
void HairMeshRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::SliderFloat("Transparency", &alpha_, 0.0, 1.0);
//        ImGui::SliderFloat("Bias", &bias_, 0.0, 1.0);
    }
}
#endif

RendererHandle HairMeshRenderer::Create(std::string name, bool show)
{
    auto renderer = new HairMeshRenderer(name, show);
    
    return RendererHandle(renderer);
}
