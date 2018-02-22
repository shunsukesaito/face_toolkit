//
//  renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <map>
#include <chrono>

#include <gl_utility/gl_core.h>
#include <gl_utility/framebuffer.h>
#include <gl_utility/gl_mesh.h>
#include <gl_utility/camera.h>
#include <gl_utility/gl_utils.h>

#include <renderer/base_renderer.h>

#include "face_module.h"

enum WINDOW { MAIN, WINDOW_COUNT};

struct Renderer;
typedef std::shared_ptr<Renderer> RendererPtr;

struct Window {
    bool valid = false;
    GLFWwindow* window_ = NULL;
    int width_;
    int height_;
    int supsample_scale_;
    
    int swidth() { return width_ * supsample_scale_; }
    int sheight() { return height_ * supsample_scale_; }
    
    Window(int supsample_scale = 0, int width = 0, int height = 0, const char* title = "")
    : width_(width)
    , height_(height)
    , supsample_scale_(supsample_scale)
    {
        int w = swidth();
        int h = sheight();
        
        window_ = glfwCreateWindow(w, h, title, NULL, NULL);
        if (!window_) {
            glfwTerminate();
            exit(EXIT_FAILURE);
        }
        
        glfwGetFramebufferSize(window_, &w, &h);
        glViewport(0, 0, w, h);
        valid = true;
    }
    ~Window() {
    }
    void resize(int supsample_scale, int width, int height) {
        supsample_scale_ = supsample_scale;
        width_ = width;
        height_ = height;
        
        int w = swidth();
        int h = sheight();
        
        glfwGetFramebufferSize(window_, &w, &h);
        glViewport(0, 0, w, h);
    }
    void launch(void (*func)(void)) {
        GLFWwindow* prev = glfwGetCurrentContext();
        if(prev != window_)
            glfwMakeContextCurrent(window_);
        func();
        if(prev != window_)
            glfwMakeContextCurrent(prev);
    }
    
    operator GLFWwindow*() const {return window_;}
};

struct Renderer {
    std::string data_dir_;
    FaceModelPtr face_model_;
    
    std::map<std::string, RendererHandle> renderer_;

    int frame_;
    std::chrono::time_point<std::chrono::system_clock> cur_time_;
    
    bool initialized_ = false;

    std::map<WINDOW, Window> windows_;
    Window & operator[](WINDOW idx) {return windows_[idx];}
    Window& get_window(WINDOW idx) { return windows_[idx]; }
    
    Renderer() {}
    Renderer(int w, int h, FaceModelPtr fm, std::string data_dir){ initGL(w,h); init(fm, data_dir); }
    ~Renderer() {}
    
    void initGL(int w, int h);
    void init(FaceModelPtr fm, std::string data_dir = "./");
    
    void addRenderer(std::string name, RendererHandle renderer);
    
    void flush() {
        glfwSwapBuffers(windows_[WINDOW::MAIN]);
    }
    void resize(int supsample_scale, int width, int height) {
        windows_[WINDOW::MAIN].resize(supsample_scale, width, height);
    }
    void draw(const FaceResult& result);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};
