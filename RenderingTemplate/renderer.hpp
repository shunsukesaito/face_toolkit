//
//  renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef renderer_hpp
#define renderer_hpp

#include <map>
#include <chrono>
#include <unordered_map>

#include "gl_core.hpp"
#include "framebuffer.hpp"
#include "gl_mesh.hpp"
#include "camera.hpp"
#include "face_model.hpp"
#include "mesh_renderer.hpp"
#include "f2f_renderer.hpp"
#include "bg_renderer.hpp"

#include "gl_utils.h"
#include "fps.hpp"

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
    std::unordered_map<std::string, GLProgram> programs_;
    Camera camera_;
    FPSCounter fps_;
    F2FRenderer f2f_renderer_;
    MeshRenderer mesh_renderer_;
    BGRenderer bg_renderer_;
    cv::VideoCapture video_capture_;
    // temp
    FaceModel facemodel_;
    FaceParams fParam_;

    Eigen::Vector3f center_;
    
    int frame_;
    std::chrono::time_point<std::chrono::system_clock> cur_time_;

    std::map<WINDOW, Window> windows_;
    Window & operator[](WINDOW idx) {return windows_[idx];}
    Window& get_window(WINDOW idx) { return windows_[idx]; }
    
//    void launch(WINDOW window, void (RendererImpl::*func)(void)) {
//        GLFWwindow* prev = glfwGetCurrentContext();
//        if(prev != windows_[window])
//            glfwMakeContextCurrent(windows_[window]);
//        (this->*func)();
//        if(prev != windows_[window])
//            glfwMakeContextCurrent(prev);
//    }
    
    Renderer() {}
    Renderer(int w, int h, std::string data_dir){ std::cout << w << " " << h << std::endl; init(w, h, data_dir); }
    ~Renderer() {}
    
    void init(int w, int h, std::string data_dir = "./");
    
    void flush() {
        glfwSwapBuffers(windows_[WINDOW::MAIN]);
    }
    void resize(int supsample_scale, int width, int height) {
        windows_[WINDOW::MAIN].resize(supsample_scale, width, height);
    }
    void draw();
    void update();
};


#endif /* renderer_hpp */
