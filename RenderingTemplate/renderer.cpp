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

void Renderer::init(int w, int h, std::string data_dir)
{
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
    
    //programs_["p2d"] = GLProgram(data_dir + "shaders/point2d.vert", data_dir + "shaders/point2d.frag", DrawMode::POINTS);
    //programs_["p3d"] = GLProgram(data_dir + "shaders/point3d.vert", data_dir + "shaders/point3d.frag", DrawMode::POINTS);
    
    clearBuffer(COLOR::COLOR_GREEN);
    CHECK_GL_ERROR();
    
    Eigen::Matrix4f RT = Camera::loadRTFromTxt(data_dir + "data/RT.txt");
    Eigen::Matrix4f K = Camera::loadKFromTxt(data_dir + "data/K.txt");
    camera_ = Camera(RT, K, w, h, 1, 1000);
    
    facemodel_.loadBinaryModel(data_dir + "data/PinModel.bin");
    fParam_.init(facemodel_);

    video_capture_.open(0);
    cv::Mat img;
    video_capture_ >> img;
    bg_renderer_.init(data_dir, img);
    //bg_renderer_.init(data_dir, data_dir + "data/cosimo.png");
    f2f_renderer_.init(data_dir, camera_, facemodel_);
    mesh_renderer_.init(data_dir, camera_, fParam_.pts_, fParam_.nml_, facemodel_.tri_pts_);
    
    center_ = getCenter(fParam_.pts_);
    
}

void Renderer::draw()
{
    char title[256];
    sprintf(title, "Main Window [fps: %.1f]", fps_.count());
    glfwSetWindowTitle(windows_[MAIN], title);

    cv::Mat img;
    video_capture_ >> img;
    bg_renderer_.render(img, true);
    mesh_renderer_.render(camera_, fParam_.pts_, fParam_.nml_);
    f2f_renderer_.render(camera_, fParam_);
    
}

void Renderer::update()
{
    fParam_.updateColor(facemodel_);
    fParam_.updateIdentity(facemodel_);
    fParam_.updateExpression(facemodel_);
}
