//
//  GUI.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef GUI_hpp
#define GUI_hpp

#include <stdio.h>
#include <thread>

#include "capture_module.hpp"
#include "face_module.hpp"
#include "renderer.hpp"
#include "fps.hpp"

#ifdef WITH_IMGUI
#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"
#endif

class GUI
{
protected:
    GUI() {};
    static GUI *instance;
    
    enum MouseButton{Left,Right,Middle};
    enum MouseMode{Rotation, Translation, Zoom, None};
    
public:
    static void setInstance(GUI *gui);
    static GUI* getInstance(){ return instance;}
    
    static void resize_callback(GLFWwindow *window, int w, int h);
    static void keyboard_callback(GLFWwindow *window, int key, int s, int a, int m);
    static void charMods_callback(GLFWwindow *window, unsigned int, int);
    static void mouseMotion_callback(GLFWwindow *window, double x, double y);
    static void mousePressed_callback(GLFWwindow *window, int button, int s, int m);
    static void mouseScroll_callback(GLFWwindow *window, double x, double y);
    static void drop_callback(GLFWwindow *window,int count,const char **filenames);

    virtual void resize(int w, int h);
    virtual void keyboard(int key, int s, int a, int m);
    virtual void charMods(unsigned int c, int m);
    virtual void mouseMotion(double x, double y);
    virtual void mousePressed(int button, int s, int m);
    virtual void mouseDown(MouseButton mb, int m);
    virtual void mouseUp(MouseButton mb, int m);
    virtual void mouseScroll(double x, double y);

    void init(int w, int h);
    void loop();
    
private:
    FPSCounter fps_;
    
    Renderer renderer_;
    
    FaceModelPtr face_model_;
    P2DFitParamsPtr p2d_param_;
    F2FParamsPtr f2f_param_;
    
    std::mutex result_mutex_;
    FaceResult result_;
    
#ifdef WITH_IMGUI
    bool show_control_panel_ = true;
#endif
    
    int width_, height_;
    MouseMode mouse_mode = MouseMode::None;
    bool mouse_down = false;
    Eigen::Matrix4f curRT;
    Eigen::Vector3f up, right;
    Eigen::Vector3f lookat;
    double current_mouse_x, current_mouse_y;
    double down_mouse_x, down_mouse_y;
};

#endif /* GUI_hpp */
