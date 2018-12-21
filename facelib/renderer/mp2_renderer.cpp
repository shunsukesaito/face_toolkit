//
//  mesh_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "mp2_renderer.h"

void MP2Renderer::init(std::string data_dir,
                      std::string shader_dir,
                      const Eigen::MatrixX3i& tri,
                      const cv::Mat& img)
{
    programs_["mp_main"] = GLProgram(shader_dir, "mp2.vert", "mp2.frag", DrawMode::TRIANGLES_IDX);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog_main = programs_["mp_main"];
    auto& prog_pl = programs_["plane"];
    
    fb_plane_ = Framebuffer::Create(1, 1, 1); // will be resized based on frame size
    
    mask_ = cv::imread(data_dir+"fw_mask.png",0);
    prog_main.createTexture("u_mask",mask_);
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    
    Camera::initializeUniforms(prog_main, U_CAMERA_MVP);

    mesh_.update_tri(tri);
    mesh_.init(prog_main, AT_POSITION | AT_NORMAL | AT_COLOR | AT_TRI);
    
    prog_main.createTexture("u_image", img);

    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_plane_->color(0), fb_plane_->width(), fb_plane_->height());
}

void MP2Renderer::render(const Camera& camera,
                         const Eigen::VectorXf& pts,
                         const std::vector<unsigned int>& tri,
                         const std::vector<glm::vec3>& bpts,
                         const cv::Mat& img)
{
    render(camera, Eigen::Matrix4f::Identity(), pts, tri, bpts, img);
}

void MP2Renderer::render(const Camera& camera,
                         const Eigen::Matrix4f& RT,
                         const Eigen::VectorXf& pts,
                         const std::vector<unsigned int>& tri,
                         const std::vector<glm::vec3>& bpts,
                         const cv::Mat& img)
{
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;
    
    if((sub_samp_*w != fb_plane_->width()) || (sub_samp_*h != fb_plane_->height()))
        fb_plane_->Resize(sub_samp_*w, sub_samp_*h, 1);
    
    auto& prog_main = programs_["mp_main"];
    auto& prog_pl = programs_["plane"];
    
    camera.updateUniforms(prog_main, RT, U_CAMERA_MVP );
    
    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_plane_->color(0));
    
    mesh_.update_position(pts);

    if (texture_update_){
        if(!img.empty())
            prog_main.updateTexture("u_image", img);
        
        pts_ = bpts;
        // use slot for normal
        Eigen::Matrix4f MVP = camera.getMVP(RT);
        Eigen::Vector4f p;
        mesh_.nml_.clear();
        for(int i = 0; i < pts.size()/3; ++i)
        {
            p << pts.b3(i), 1.0;
            p = MVP * p;
            mesh_.nml_.push_back(glm::vec3(p[0]/p[3],p[1]/p[3],p[2]/p[3]));
        }
        mesh_.nml_.insert(mesh_.nml_.end(), pts_.begin(), pts_.end());
        std::vector<glm::vec4> clr_add(pts_.size(),glm::vec4(0.0));
        mesh_.clr_.assign(mesh_.pts_.size(),glm::vec4(1.0));
        mesh_.clr_.insert(mesh_.clr_.end(), clr_add.begin(), clr_add.end());

        mesh_.update_tri(tri);
    }
    
    mesh_.pts_.insert(mesh_.pts_.end(), pts_.begin(), pts_.end());

    
    mesh_.update(prog_main, AT_POSITION | AT_NORMAL | AT_COLOR | AT_TRI);

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

void MP2Renderer::render(const Camera& camera,
                         const Eigen::Matrix4f& RT,
                         const Eigen::VectorXf& pts,
                         const cv::Mat& img)
{
    if(!show_)
        return;
    
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;
    
    if((sub_samp_*w != fb_plane_->width()) || (sub_samp_*h != fb_plane_->height()))
        fb_plane_->Resize(sub_samp_*w, sub_samp_*h, 1);
    
    auto& prog_main = programs_["mp_main"];
    auto& prog_pl = programs_["plane"];
    
    camera.updateUniforms(prog_main, RT, U_CAMERA_MVP );
    
    prog_pl.setUniformData("u_alpha", alpha_);
    prog_pl.updateTexture("u_texture", fb_plane_->color(0));
    
    mesh_.update_position(pts);
    
    if (texture_update_){
        if(!img.empty())
            prog_main.updateTexture("u_image", img);
        
        // use slot for normal
        Eigen::Matrix4f MVP = camera.getMVP(RT);
        Eigen::Vector4f p;
        mesh_.nml_.clear();
        for(int i = 0; i < pts.size()/3; ++i)
        {
            p << pts.b3(i), 1.0;
            p = MVP * p;
            mesh_.nml_.push_back(glm::vec3(p[0]/p[3],p[1]/p[3],p[2]/p[3]));
        }
        mesh_.nml_.insert(mesh_.nml_.end(), pts_.begin(), pts_.end());
        std::vector<glm::vec4> clr_add(pts_.size(),glm::vec4(0.0));
        mesh_.clr_.assign(mesh_.pts_.size(),glm::vec4(1.0));
        mesh_.clr_.insert(mesh_.clr_.end(), clr_add.begin(), clr_add.end());
        
        mesh_.update_tri(tri_);
    }
    
    mesh_.pts_.insert(mesh_.pts_.end(), pts_.begin(), pts_.end());
    
    
    mesh_.update(prog_main, AT_POSITION | AT_NORMAL | AT_COLOR | AT_TRI);
    
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
void MP2Renderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr fm)
{
    init(data_dir,shader_dir,fm->tri_pts_);
}

void MP2Renderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    render(result.cameras[cam_id], result.fd[frame_id].RT(),
           result.fd[frame_id].pts_, result.cap_data[frame_id][cam_id].img_);
}
#endif

#ifdef WITH_IMGUI
void MP2Renderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::Checkbox("tex update", &texture_update_);
        ImGui::SliderFloat("Transparency", &alpha_, 0.0, 1.0);
    }
}
#endif

RendererHandle MP2Renderer::Create(std::string name, bool show)
{
    auto renderer = new MP2Renderer(name, show);
    
    return RendererHandle(renderer);
}
