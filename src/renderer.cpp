//
//  renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "renderer.hpp"
#include "obj_loader.hpp"

void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

void Renderer::init(int w, int h, FaceModelPtr fm, std::string data_dir)
{
    data_dir_ = data_dir;
    face_model_ = fm;
    
    glfwSetErrorCallback(error_callback);
    
    if (!glfwInit())
        exit(EXIT_FAILURE);
    
#if __APPLE__
    glfwWindowHint(GLFW_SAMPLES, 8);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
    
    windows_.insert(std::make_pair(MAIN, Window(1, w, h, "Main Window")));
    glfwMakeContextCurrent(windows_[MAIN]);
    
    glfwSwapInterval(1);
    CHECK_GL_ERROR();
    
    // initialize GLEW
    glewExperimental=GL_TRUE;
    GLenum err = glewInit();
    CHECK_GL_ERROR();
    if (GLEW_OK != err)
    {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    else{
        fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    }
    if(!glewIsSupported("GL_ARB_vertex_buffer_object")){
        std::cout << "OMG! This machine does not support GLARBbuffer!!" << std::endl;
    }
    
    int major, minor, rev;
    major = glfwGetWindowAttrib(windows_[MAIN], GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(windows_[MAIN], GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(windows_[MAIN], GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
    
    glfwSetInputMode(windows_[MAIN],GLFW_CURSOR,GLFW_CURSOR_NORMAL);
    
}

void Renderer::draw(FaceResult& result)
{
    auto& camera = result.camera;
    auto& fParam = result.fParam;
    
    if(!initialized_){
        bg_renderer_.init(data_dir_, result.img);
        f2f_renderer_.init(data_dir_, *face_model_);
        mesh_renderer_.init(data_dir_, fParam.pts_, fParam.nml_, face_model_->tri_pts_);
        p3d_renderer_.init(data_dir_, getP3DFromP2PC(fParam.pts_, result.c_p2p));
        p2d_renderer_.init(data_dir_);
        
        initialized_ = true;
    }
    
    if(initialized_){
        bg_renderer_.render(result.img);

        if(show_mesh_)
            mesh_renderer_.render(camera, result.fParam.RT, result.fParam.pts_, result.fParam.nml_);
        if(show_f2f_)
            f2f_renderer_.render(camera, result.fParam);
        if(show_p3d_){
            p3d_renderer_.render(camera, result.fParam.RT, getP3DFromP2PC(result.fParam.pts_, result.c_p2p));
            p3d_renderer_.render(camera, result.fParam.RT, getP3DFromP2LC(result.fParam.pts_, result.c_p2l));
        }
        if(show_p2d_)
            p2d_renderer_.render(camera.width_, camera.height_, result.p2d);
    }
}

#ifdef WITH_IMGUI
void Renderer::updateIMGUI()
{
    ImGui::Checkbox("show mesh", &show_mesh_);
    ImGui::Checkbox("show f2f", &show_f2f_);
    ImGui::Checkbox("show p2d", &show_p2d_);
    ImGui::Checkbox("show p3d", &show_p3d_);
    f2f_renderer_.updateIMGUI();
}
#endif

