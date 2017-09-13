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
    
    programs_["main"] = GLProgram(data_dir + "shaders/mesh.vert", data_dir + "shaders/mesh.frag", DrawMode::TRIANGLES_IDX);
    programs_["bg"] = GLProgram(data_dir + "shaders/full_texture_bgr.vert", data_dir + "shaders/full_texture_bgr.frag", DrawMode::TRIANGLES);
    programs_["p2d"] = GLProgram(data_dir + "shaders/point2d.vert", data_dir + "shaders/point2d.frag", DrawMode::POINTS);
    programs_["p3d"] = GLProgram(data_dir + "shaders/point3d.vert", data_dir + "shaders/point3d.frag", DrawMode::POINTS);
    
    clearBuffer(COLOR::COLOR_GREEN);
    CHECK_GL_ERROR();

    glEnable(GL_DEPTH_TEST);
    CHECK_GL_ERROR();
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    CHECK_GL_ERROR();
    
    Eigen::Matrix4f RT = Camera::loadRTFromTxt(data_dir + "data/RT.txt");
    Eigen::Matrix4f K = Camera::loadKFromTxt(data_dir + "data/K.txt");
    camera_ = Camera(RT, K, w, h, 1, 1000);

    auto& prog = programs_["main"];
    auto& prog2d = programs_["p2d"];
    auto& prog3d = programs_["p3d"];
    
    camera_.intializeUniforms(prog, true, false);
    camera_.updateUniforms(prog, true, false);
    
    camera_.intializeUniforms(prog3d, false, false);
    camera_.updateUniforms(prog3d, false, false);

    facemodel_.loadBinaryModel(data_dir + "data/PinModel.bin");
    fParam_.init(facemodel_);
    mesh_.init_with_idx(prog, fParam_.pts_, fParam_.nml_, facemodel_.tri_pts_);
    
    f2f_renderer_.init(data_dir, camera_, facemodel_);
    
    glPlane bg;
    bg.init(programs_["bg"]);
    programs_["bg"].createTexture("u_texture", data_dir + "data/cosimo.png");
    
    center_ = getCenter(fParam_.pts_);
}

void Renderer::draw()
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);
    programs_["bg"].draw();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    //programs_["main"].draw();
    
    f2f_renderer_.render(camera_, fParam_);
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
}

void Renderer::update()
{
    auto& prog = programs_["main"];
    camera_.updateUniforms(prog, true, false);
    
    fParam_.updateColor(facemodel_);
    fParam_.updateIdentity(facemodel_);
    fParam_.updateExpression(facemodel_);
    mesh_.update_with_idx(prog, fParam_.pts_, fParam_.nml_);
}
