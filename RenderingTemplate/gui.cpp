//
//  GUI.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gui.hpp"

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
    
}

void GUI::keyboard(int key, int s, int a, int m)
{
    
}

void GUI::charMods(unsigned int c, int m)
{
    
}

void GUI::mouseMotion(double x, double y)
{
    
}

void GUI::mousePressed(int button, int s, int m)
{
    
}

void GUI::mouseScroll(double x, double y)
{
    
}

void GUI::init(int w, int h)
{
    renderer_.init(w, h);
    
    GLFWwindow* window = renderer_.windows_[MAIN];

#ifdef WITH_NANOGUI
    screen_ = new nanogui::Screen();
    screen_->initialize(window, false);
    ngui_ = new nanogui::FormHelper(screen_);
    
    ngui_->setFixedSize(Eigen::Vector2i(60,20));
    ngui_->addWindow(Eigen::Vector2i(10,10),"Main Window");
    
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
    renderer_.draw();
    
#ifdef WITH_NANOGUI
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
