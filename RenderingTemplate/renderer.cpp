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
    //CHECK_GL_ERROR();
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
    
    programs_["main shader"] = GLProgram(data_dir + "shaders/mesh.vert", data_dir + "shaders/mesh.frag", DrawMode::TRIANGLES);
    clear(Renderer::COLOR_GREEN);
    CHECK_GL_ERROR();

    glEnable(GL_DEPTH_TEST);
    CHECK_GL_ERROR();
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    CHECK_GL_ERROR();
    
    Eigen::Matrix4f RT = Camera::loadRTFromTxt(data_dir + "data/RT.txt");
    Eigen::Matrix4f K = Camera::loadKFromTxt(data_dir + "data/K.txt");
    camera_ = Camera(RT, K, w, h, 1, 100);

    auto& prog = programs_["main shader"];
    
    camera_.intializeUniforms(prog, true, false);
    camera_.updateUniforms(prog, true, false);

    Eigen::VectorXf pts;
    Eigen::MatrixX3f nml;
    Eigen::MatrixX2f uvs;
    Eigen::MatrixX3i tri_pts;
    Eigen::MatrixX3i tri_uv;

    loadObjFile(data_dir + "data/neutral.obj", pts, nml, uvs, tri_pts, tri_uv);
    writeObj(data_dir + "data/debug.obj", pts, nml, uvs, tri_pts, tri_uv);
    mesh_.init(prog, pts, nml, tri_pts);
    
    center_ = getCenter(pts);
}

void Renderer::draw()
{
    glEnable(GL_DEPTH_TEST);
    CHECK_GL_ERROR();
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    CHECK_GL_ERROR();
    
    programs_["main shader"].draw();
}

void Renderer::update()
{
    auto& prog = programs_["main shader"];
    camera_.updateUniforms(prog, true, false);
}
