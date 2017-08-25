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

#include "renderer.hpp"

#define WITH_NANOGUI

#ifdef WITH_NANOGUI
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
namespace nanogui { class FormHelper; class Screen; }
#endif

class GUI
{
protected:
    GUI() {};
    static GUI *instance;
    
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
    virtual void mouseScroll(double x, double y);

    void init(int w, int h);
    void loop();
    void update();
    
private:
    Renderer renderer_;

#ifdef WITH_NANOGUI
    nanogui::FormHelper* ngui_ = nullptr;
    nanogui::Screen* screen_ = nullptr;
#endif
};

#endif /* GUI_hpp */
