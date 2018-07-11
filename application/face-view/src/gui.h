//
//  GUI.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#pragma once

#include <stdio.h>
#include <thread>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <Eigen/Dense>

#include <utility/fps.h>

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

    void initGL();
    void init(int w, int h);
    void loop();
    
private:
    FPSCounter fps_;
    
    bool pause_ = false;
    
#ifdef WITH_IMGUI
    bool show_control_panel_ = true;
#endif
    
    int width_, height_;
    MouseMode mouse_mode = MouseMode::None;
    bool mouse_down = false;
    Eigen::Matrix4f curRT, oriRT;
    Eigen::Vector3f up, right;
    Eigen::Vector3f lookat;
    double current_mouse_x, current_mouse_y;
    double down_mouse_x, down_mouse_y;
};
