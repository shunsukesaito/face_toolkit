//
//  mesh_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "mesh_renderer.hpp"

void MeshRenderer::init(std::string data_dir,
                        const Eigen::MatrixX3i& tri)
{
    programs_["mesh"] = GLProgram(data_dir + "shaders/mesh.vert",
                                  data_dir + "shaders/mesh.frag",
                                  DrawMode::TRIANGLES_IDX);
    
    auto& prog = programs_["mesh"];
    
    Camera::initializeUniforms(prog, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update_tri(tri);
    mesh_.init(prog, AT_POSITION | AT_NORMAL | AT_TRI);
}

void MeshRenderer::render(const Camera& camera,
                          const Eigen::VectorXf& pts,
                          const Eigen::MatrixX3f& nml,
                          bool draw_sphere)
{
    auto& prog = programs_["mesh"];
    
    camera.updateUniforms(prog, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update_position(pts);
    mesh_.update_normal(nml);
    mesh_.update(prog, AT_POSITION | AT_NORMAL);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    prog.draw();
    
    if(draw_sphere){
        int w, h;
        GLFWwindow* window = glfwGetCurrentContext();
        glfwGetFramebufferSize(window, &w, &h);
        int sp_w = (int)(0.2*(float)std::min(w,h));
        glViewport(w-sp_w, 0, sp_w, sp_w);
        camera.updateUniforms4Sphere(prog, U_CAMERA_MVP | U_CAMERA_MV);
        ball_.update(prog, AT_POSITION | AT_NORMAL | AT_TRI);
        prog.draw();
        glViewport(0, 0, w, h);
    }
}

void MeshRenderer::render(const Camera& camera,
                          const Eigen::Matrix4f& RT,
                          const Eigen::VectorXf& pts,
                          const Eigen::MatrixX3f& nml,
                          bool draw_sphere)
{
    auto& prog = programs_["mesh"];
    
    camera.updateUniforms(prog, RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update_position(pts);
    mesh_.update_normal(nml);
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_TRI);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw();
    
    if(draw_sphere){
        int w, h;
        GLFWwindow* window = glfwGetCurrentContext();
        glfwGetFramebufferSize(window, &w, &h);
        int sp_w = (int)(0.2*(float)std::min(w,h));
        glViewport(w-sp_w, 0, sp_w, sp_w);
        camera.updateUniforms4Sphere(prog, U_CAMERA_MVP | U_CAMERA_MV);
        ball_.update(prog, AT_POSITION | AT_NORMAL | AT_TRI);
        prog.draw();
        glViewport(0, 0, w, h);
    }
}
