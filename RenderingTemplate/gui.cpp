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
    instance->resize(w, h);
}

void GUI::keyboard_callback(GLFWwindow *window, int key, int s, int a, int m)
{
#if defined(WITH_IMGUI)
    if(!ImGui::IsAnyItemActive())
        instance->keyboard(key, s, a, m);
#else
    instance->keyboard(key, s, a, m);
#endif
}

void GUI::charMods_callback(GLFWwindow *window, unsigned int c, int m)
{
#if defined(WITH_IMGUI)
    if(!ImGui::IsAnyItemActive())
        instance->charMods(c, m);
#else
    instance->charMods(c, m);
#endif
}

void GUI::mouseMotion_callback(GLFWwindow *window, double x, double y)
{
#if defined(WITH_IMGUI)
    if(!ImGui::IsAnyItemActive())
        instance->mouseMotion(x, y);
#else
    instance->mouseMotion(x, y);
#endif
}

void GUI::mousePressed_callback(GLFWwindow *window, int button, int s, int m)
{
#if defined(WITH_IMGUI)
    instance->mousePressed(button, s, m);
#else
    instance->mousePressed(button, s, m);
#endif
}

void GUI::mouseScroll_callback(GLFWwindow *window, double x, double y)
{
#if defined(WITH_IMGUI)
    if(!ImGui::IsAnyItemActive())
        instance->mouseScroll(x, y);
#else
    instance->mouseScroll(x, y);
#endif
}

void GUI::drop_callback(GLFWwindow *window,int count,const char **filenames)
{
}

void GUI::resize(int w, int h)
{
    width_ = w;
    height_ = h;

    renderer_.resize(1, w, h);
}

void GUI::keyboard(int key, int s, int a, int m)
{
    Eigen::Matrix4f& RT = renderer_.camera_.extrinsic_;
    
    if(key == GLFW_KEY_F && a == GLFW_PRESS){
        rotCenter = renderer_.center_;
        RT.block<3,3>(0,0) = Eigen::Quaternion<float>(0, 1, 0, 0).toRotationMatrix();
        RT.block<3,1>(0,3) = rotCenter + Eigen::Vector3f(0,0,50);
    }
    if(key == GLFW_KEY_1 && a == GLFW_PRESS){
        F2FRenderer f2frender;
        f2frender.init("./", renderer_.camera_, renderer_.facemodel_);
        std::vector<cv::Mat_<cv::Vec4f>> out;
        f2frender.render(width_/4, height_/4, renderer_.camera_, renderer_.fParam_, out);
        
        for(int i = 0; i < out.size(); ++i)
        {
            cv::imwrite(std::to_string(i) + ".png", 255.0*out[i]);
        }
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
            Eigen::Matrix3f Rpre = curRT.block<3,3>(0,0);
            RT.block<3,3>(0,0) = curRT.block<3,3>(0,0)*R;
            RT.block<3,1>(0,3) = RT.block<3,3>(0,0)*Rpre.transpose()*(curRT.block<3,1>(0,3)-rotCenter)+rotCenter;
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
    
    GLFWwindow* window = renderer_.windows_[MAIN];

#ifdef WITH_IMGUI
    ImGui_ImplGlfwGL3_Init(window, true);
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
    clearBuffer(COLOR::COLOR_GREY);
    int w, h;
    glfwGetFramebufferSize(renderer_.get_window(MAIN), &w, &h);
    glViewport(0, 0, w, h);
    
    renderer_.update();
    renderer_.draw();
    
#ifdef WITH_IMGUI
    ImGui_ImplGlfwGL3_NewFrame();
    ImGui::Begin("Control Panel", &show_control_panel_);
    
    FaceParams& fParam = renderer_.fParam_;
    
    if (ImGui::CollapsingHeader("Face Parameters")){
        if (ImGui::TreeNode("ID")){
            for(int i = 0; i < fParam.idCoeff.size(); ++i)
                ImGui::SliderFloat(("id" + std::to_string(i)).c_str(), &fParam.idCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("EX")){
            for(int i = 0; i < fParam.exCoeff.size(); ++i)
                ImGui::SliderFloat(("ex" + std::to_string(i)).c_str(), &fParam.exCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("CL")){
            for(int i = 0; i < fParam.alCoeff.size(); ++i)
                ImGui::SliderFloat(("al" + std::to_string(i)).c_str(), &fParam.alCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
        }
        ImGui::InputFloat3("Tr", &fParam.RT(0,3));
        if (ImGui::TreeNode("SH")){
            for(int i = 0; i < fParam.SH.cols(); ++i)
                ImGui::InputFloat3(("sh" + std::to_string(i)).c_str(), &fParam.SH.col(i)[0]);
            ImGui::TreePop();
        }
    }
    if (ImGui::CollapsingHeader("Camera Parameters")){
        Camera& cam = renderer_.camera_;
        ImGui::InputFloat("zNear", &cam.zNear_);
        ImGui::InputFloat("zFar", &cam.zFar_);
        ImGui::InputFloat("fx", &cam.intrinsic_(0,0));
        ImGui::InputFloat("fy", &cam.intrinsic_(1,1));
        ImGui::InputFloat("px", &cam.intrinsic_(0,2));
        ImGui::InputFloat("py", &cam.intrinsic_(1,2));
        ImGui::InputInt("width", &cam.width_);
        ImGui::InputInt("height", &cam.height_);
    }
    if (ImGui::CollapsingHeader("Rendering Parameters")){
        F2FRenderParams& param = renderer_.f2f_renderer_.param_;
        ImGui::Checkbox("mask", &param.enable_mask);
        ImGui::Checkbox("seg", &param.enable_seg);
        ImGui::Checkbox("texture", &param.enable_tex);
        const char* listbox_items[] = { "positions", "normals", "albedo", "texCoords", "diffuse", "shading", "vBarycentric", "vIndices"};
        ImGui::ListBox("RenderTarget", &param.location, listbox_items, 8);
    }
    ImGui::End();
    ImGui::Render();
    
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
    
#ifdef WITH_IMGUI
    ImGui_ImplGlfwGL3_Shutdown();
#endif
    
    glfwTerminate();
}
