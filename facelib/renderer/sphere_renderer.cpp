//
//  sphere_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "sphere_renderer.h"

void SphereRenderer::init(std::string data_dir, std::string shader_dir)
{
    programs_["mesh"] = GLProgram(shader_dir, "sphere.vert", "sphere.frag", DrawMode::TRIANGLES);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog = programs_["mesh"];
    auto& prog_pl = programs_["plane"];
    
    fb_ = Framebuffer::Create(1, 1, 1); // will be resized based on frame size
    
    prog.createUniform("u_uv_view", DataType::UINT);
    prog.createTexture("u_texture", data_dir + "render/uv.png");
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    
    Camera::initializeUniforms(prog, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.generateSphere(1.0,11,11,false);
    mesh_.init(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_->color(0), fb_->width(), fb_->height());
}

void SphereRenderer::render(const Camera& camera,
                            const Eigen::Matrix4f& RT)
{
    if(!show_)
        return;
    
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;

    if((sub_samp_*w != fb_->width()) || (sub_samp_*h != fb_->height()))
        fb_->Resize(sub_samp_*w, sub_samp_*h, 1);

    auto& prog = programs_["mesh"];
    auto& prog_pl = programs_["plane"];
        
    camera.updateUniforms(prog, RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    prog.setUniformData("u_uv_view", (uint)uvview_);

    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_->color(0));

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
void SphereRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr fm)
{
    init(data_dir, shader_dir);
}

void SphereRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    render(result.cameras[cam_id]);
}
#endif

#ifdef WITH_IMGUI
void SphereRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::Checkbox("uv view", &uvview_);
        ImGui::SliderFloat("Transparency", &alpha_, 0.0, 1.0);
    }
}
#endif

RendererHandle SphereRenderer::Create(std::string name, bool show)
{
    auto renderer = new SphereRenderer(name, show);
    
    return RendererHandle(renderer);
}
