//
//  base_renderer.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 12/4/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <utility/EigenHelper.h>
#include <gl_utility/camera.h>
#include <gl_utility/gl_core.h>
#include <gl_utility/gl_mesh.h>

#ifdef FACE_TOOLKIT
#include <shape_model/face_model.h>
#include <optimizer/face_result.h>
#endif

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

struct BaseRenderer;
typedef std::shared_ptr<BaseRenderer> RendererHandle;

enum WINDOW { MAIN, WINDOW_COUNT};

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
    inline void flush(){glfwSwapBuffers(window_);}
    
    operator GLFWwindow*() const {return window_;}
};

void initializeGL(int w, int h, std::map<WINDOW, Window>& windows);
void screenshot(cv::Mat& img, Window& window);

struct BaseRenderer
{
    std::unordered_map<std::string, GLProgram> programs_;
    std::string name_;
    bool show_ = false;
    bool wire_ = false;
    
    BaseRenderer() : name_(""), show_(false), wire_(false){}
    BaseRenderer(std::string name, bool show, bool wire = false) : name_(name), show_(show), wire_(wire){}
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, std::string shader_dir, FaceModelPtr fm){ throw std::runtime_error( "Error: Base class (Renderer) is called..."); }
    virtual void render(const FaceResult& result){ throw std::runtime_error( "Error: Base class (Renderer) is called..."); }
#endif
    
#ifdef WITH_IMGUI
    virtual inline void updateIMGUI(){ throw std::runtime_error( "Error: Base class (Renderer) is called..."); }
#endif
};
