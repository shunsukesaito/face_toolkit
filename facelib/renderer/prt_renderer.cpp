//
//  prt_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "prt_renderer.h"

void PRTRenderer::init(std::string shader_dir,
                       const Eigen::MatrixX3i& tri)
{
    programs_["mesh"] = GLProgram(shader_dir, "prt.vert", "prt.frag", DrawMode::TRIANGLES_IDX);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog = programs_["mesh"];
    auto& prog_pl = programs_["plane"];
    
    fb_ = Framebuffer::Create(1, 1, 1); // will be resized based on frame size

    prog.createUniform("u_analytic", DataType::UINT);
    prog.createUniform("u_SHCoeffs", DataType::VECTOR3);

    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    
    Camera::initializeUniforms(prog, U_CAMERA_MVP);
    
    mesh_.update_tri(tri);
    mesh_.init(prog, AT_POSITION | AT_NORMAL | AT_TRI);
    
    // create prt coeff attributes
    prog.createAttribute("v_prt0", DataType::VECTOR3, true);
    prog.createAttribute("v_prt1", DataType::VECTOR3, true);
    prog.createAttribute("v_prt2", DataType::VECTOR3, true);

    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_->color(0), fb_->width(), fb_->height());
}

void PRTRenderer::render(const Camera& camera,
                         const PRTData& data)
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
    
    // spherical harmonics update
    std::vector<glm::vec3> sh;
    for(int i = 0; i < 9; ++i)
    {
        sh.push_back(glm::vec3(data.SH()(0,i),data.SH()(1,i),data.SH()(2,i)));
    }
    
    prog.setUniformData("u_analytic", (unsigned)analytial_);
    prog.setUniformData("u_SHCoeffs", sh);
        
    camera.updateUniforms(prog, data.RT(), U_CAMERA_MVP);
    
    mesh_.update_position(data.pts());
    mesh_.update_normal(data.nml());
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_TRI);
    
    // update PRTCoeffs
    // NOTE: it's waste of computation if we don't update every frame (and always we don't)
    int nVert = data.pts().size()/3;
    if(glPrt_.size() != 3)
        glPrt_.resize(3);
    if(glPrt_[0].size() != nVert)
    {
        for(int i = 0; i < 3; ++i)
        {
            glPrt_[i].resize(nVert);
        }
    }
    const Eigen::MatrixXf& prt = data.prt_;
    for(int i = 0; i < nVert; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            glPrt_[j][i] = glm::vec3(prt(i,j*3+0),prt(i,j*3+1),prt(i,j*3+2));
        }
    }
    prog.setAttributeData("v_prt0", glPrt_[0]);
    prog.setAttributeData("v_prt1", glPrt_[1]);
    prog.setAttributeData("v_prt2", glPrt_[2]);

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

#ifdef WITH_IMGUI
void PRTRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::Checkbox("analytic", &analytial_);
        ImGui::SliderFloat("Transparency", &alpha_, 0.0, 1.0);
    }
}
#endif

RendererHandle PRTRenderer::Create(std::string name, bool show)
{
    auto renderer = new PRTRenderer(name, show);
    
    return RendererHandle(renderer);
}
