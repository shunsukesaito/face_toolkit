//
//  GUI.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gui.hpp"

#include "trackball.hpp"

GUI* GUI::instance = new GUI();

void GUI::setInstance(GUI *gui)
{
    instance = gui;
}

void GUI::resize_callback(GLFWwindow *window, int w, int h)
{
#ifdef WITH_NANOGUI
    instance->screen_->resizeCallbackEvent(w, h);
#endif
    instance->resize(w, h);
}

void GUI::keyboard_callback(GLFWwindow *window, int key, int s, int a, int m)
{
#ifdef WITH_NANOGUI
    if(instance->screen_->keyCallbackEvent(key,s,a,m) == false)
        instance->keyboard(key, s, a, m);
#else
    instance->keyboard(key, s, a, m);
#endif
}

void GUI::charMods_callback(GLFWwindow *window, unsigned int c, int m)
{
#ifdef WITH_NANOGUI
    if(instance->screen_->charCallbackEvent(c) == false)
        instance->charMods(c, m);
#else
    instance->charMods(c, m);
#endif
}

void GUI::mouseMotion_callback(GLFWwindow *window, double x, double y)
{
#ifdef WITH_NANOGUI
    if(instance->screen_->cursorPosCallbackEvent(x,y) == false)
        instance->mouseMotion(x, y);
#else
    instance->mouseMotion(x, y);
#endif
}

void GUI::mousePressed_callback(GLFWwindow *window, int button, int s, int m)
{
#ifdef WITH_NANOGUI
    if(instance->screen_->mouseButtonCallbackEvent(button,s,m) == false)
        instance->mousePressed(button, s, m);
#else
    instance->mousePressed(button, s, m);
#endif
}

void GUI::mouseScroll_callback(GLFWwindow *window, double x, double y)
{
#ifdef WITH_NANOGUI
    if(instance->screen_->scrollCallbackEvent(x,y) == false)
        instance->mouseScroll(x, y);
#else
    instance->mouseScroll(x, y);
#endif
}

void GUI::drop_callback(GLFWwindow *window,int count,const char **filenames)
{
#ifdef WITH_NANOGUI
    instance->screen_->dropCallbackEvent(count, filenames);
#endif
}

void GUI::resize(int w, int h)
{
    width_ = w;
    height_ = h;

    renderer_.resize(1, w, h);
    
    screen_->performLayout();
}

void GUI::keyboard(int key, int s, int a, int m)
{
    Eigen::Matrix4f& RT = renderer_.camera_.extrinsic_;
    
    if(key == GLFW_KEY_F && a == GLFW_PRESS){
        rotCenter = renderer_.center_;
        RT.block<3,3>(0,0) = Eigen::Quaternion<float>(0, 1, 0, 0).toRotationMatrix();
        RT.block<3,1>(0,3) = rotCenter + Eigen::Vector3f(0,0,10);
    }
  
}

void GUI::charMods(unsigned int c, int m)
{
    
}

void GUI::mouseMotion(double x, double y)
{
    float tr_scale = 0.01;
    double rot_scale = 1.0;
    current_mouse_x = x;
    current_mouse_y = y;
    
    Eigen::Matrix4f& RT = renderer_.camera_.extrinsic_;
    
    switch (mouse_mode)
    {
        case MouseMode::Rotation:
        {
            Eigen::Quaternion<float> q;
            trackball(width_, height_, rot_scale, down_mouse_x, down_mouse_y, current_mouse_x, current_mouse_y, q);
            
            Eigen::Matrix3f R = q.toRotationMatrix();
            RT.block<3,3>(0,0) = R*curRT.block<3,3>(0,0);
            RT.block<3,1>(0,3) = R*(curRT.block<3,1>(0,3)-rotCenter)+rotCenter;
            break;
        }
        case MouseMode::Translation:
        {
            double dx = down_mouse_x - current_mouse_x;
            double dy = down_mouse_y - current_mouse_y;
            RT.block<3,1>(0,3) = tr_scale*((float)dx * right + (float)dy * up) + curRT.block<3,1>(0,3);
            break;
        }
        case MouseMode::Zoom:
        {
            break;
        }
        default:
            break;
    }
}

void GUI::mousePressed(int button, int s, int m)
{
    MouseButton mb;
    
    if (button == GLFW_MOUSE_BUTTON_1)
        mb = MouseButton::Left;
    else if (button == GLFW_MOUSE_BUTTON_2)
        mb = MouseButton::Right;
    else
        mb = MouseButton::Middle;
    
    if (s == GLFW_PRESS)
    {
        mouseDown(mb,m);
    } else
    {
        mouseUp(mb,m);
    }
}
    
void GUI::mouseDown(MouseButton mb, int m)
{
    down_mouse_x = current_mouse_x;
    down_mouse_y = current_mouse_y;
    curRT = renderer_.camera_.extrinsic_;
    up = curRT.block<3,3>(0,0).inverse()*Eigen::Vector3f(0,1,0);
    right = (curRT.block<3,1>(0,3)-rotCenter).cross(up).normalized();
    
    switch (mb)
    {
        case MouseButton::Left:
            mouse_mode = MouseMode::Rotation;
            break;
        case MouseButton::Right:
            mouse_mode = MouseMode::Translation;
            break;
        case MouseButton::Middle:
            mouse_mode = MouseMode::Zoom;
            break;
        default:
            mouse_mode = MouseMode::None;
            break;
    }
}

void GUI::mouseUp(MouseButton mb, int m)
{
    Eigen::Matrix4f& RT = renderer_.camera_.extrinsic_;
    
    if(mb == MouseButton::Right)
        rotCenter += RT.block<3,1>(0,3) - curRT.block<3,1>(0,3);
    mouse_mode = MouseMode::None;
}

void GUI::mouseScroll(double x, double y)
{
    Eigen::Matrix4f& RT = renderer_.camera_.extrinsic_;

    float scale = 0.01;
    Eigen::Vector3f t = RT.block<3,1>(0,3) - rotCenter;
    
    t = (1.0 + scale * y)*t;
    
    RT.block<3,1>(0,3) = t + rotCenter;
}

void GUI::init(int w, int h)
{
    width_ = w;
    height_ = h;
    
    renderer_.init(w, h, "./");
    
    rotCenter = renderer_.center_;
    
    std::cout << rotCenter << std::endl;
    
    GLFWwindow* window = renderer_.windows_[MAIN];

#ifdef WITH_NANOGUI
    screen_ = new nanogui::Screen();
    screen_->initialize(window, false);
    ngui_ = new nanogui::FormHelper(screen_);
    
    ngui_->setFixedSize(Eigen::Vector2i(100,20));
    ngui_->addWindow(Eigen::Vector2i(10,10),"Control Panel");
    
    ngui_->addGroup("Camera");
    ngui_->addVariable("tx",renderer_.camera_.extrinsic_(0,3))->setSpinnable(true);
    ngui_->addVariable("ty", renderer_.camera_.extrinsic_(1,3))->setSpinnable(true);
    ngui_->addVariable("tz", renderer_.camera_.extrinsic_(2,3))->setSpinnable(true);
    
    ngui_->addVariable("zN", renderer_.camera_.zNear_);
    ngui_->addVariable("zF", renderer_.camera_.zFar_);
    
    screen_->setVisible(true);
    screen_->performLayout();
#endif
    
    glfwSetKeyCallback(window, keyboard_callback);
    glfwSetCursorPosCallback(window,mouseMotion_callback);
    glfwSetWindowSizeCallback(window,resize_callback);
    glfwSetMouseButtonCallback(window, mousePressed_callback);
    glfwSetScrollCallback(window,mouseScroll_callback);
    glfwSetCharModsCallback(window,charMods_callback);
    glfwSetDropCallback(window,drop_callback);
}

void GUI::update()
{
    renderer_.clear(Renderer::COLOR_GREY);
    renderer_.update();
    renderer_.draw();
    
#ifdef WITH_NANOGUI
    ngui_->refresh();
    screen_->drawContents();
    screen_->drawWidgets();
#endif
    
    renderer_.flush();
}

void GUI::loop()
{
    //GLsync tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    
    while(!glfwWindowShouldClose(renderer_.windows_[MAIN]))
    {
        update();
        
        //glDeleteSync(tsync);
        //tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

        glfwPollEvents();
    }
    
    glfwTerminate();
}
