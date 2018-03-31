//
//  renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "renderer.h"

#include <utility/obj_loader.h>

void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

void Renderer::screenshot(cv::Mat& img)
{
    int width, height;
    glfwGetFramebufferSize(windows_[MAIN], &width, &height);
    
    unsigned char* img_ptr = new unsigned char[3 * width * height];
    glReadBuffer(GL_FRONT);
    glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, &img_ptr[0]);
    img = cv::Mat(height,width,CV_8UC3,img_ptr);
    cv::resize(img,img,cv::Size(windows_[MAIN].width_, windows_[MAIN].height_));
    
    cv::flip(img,img,0);
    
    delete [] img_ptr;
}

void Renderer::initGL(int w, int h)
{
    glfwSetErrorCallback(error_callback);
    
    if (!glfwInit())
        exit(EXIT_FAILURE);
    
#if __APPLE__
    glfwWindowHint(GLFW_SAMPLES, 8);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
    
    glfwWindowHint(GLFW_FOCUSED, true);
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

void Renderer::init(FaceModelPtr fm, std::string data_dir)
{
    data_dir_ = data_dir;
    face_model_ = fm;
    
    for(auto&& r : renderer_)
        r.second->init(data_dir_, face_model_);
}

void Renderer::addRenderer(std::string name, RendererHandle renderer)
{
    if(renderer_.find(name) != renderer_.end()){
        throw std::runtime_error("Attempted to create renderer with duplicate name " + name);
    }
    renderer_[name] = renderer;
}

void Renderer::draw(const FaceResult& result)
{
    for(auto&& r : renderer_)
        r.second->render(result);
}

#ifdef WITH_IMGUI
void Renderer::updateIMGUI()
{
    for(auto&& r : renderer_)
        r.second->updateIMGUI();
}
#endif

