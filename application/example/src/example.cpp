//
//  GUI.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gui.h"

#include <memory>

#include <renderer/base_renderer.h>
#include <renderer/bg_renderer.h>
#include <renderer/mesh_renderer.h>

#include <utility/str_utils.h>
#include <utility/obj_loader.h>
#include <utility/trackball.h>
#include <utility/openmesh_utilities.h>

#include <shape_model/mesh_data.h>

#ifdef WITH_IMGUI
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>
#endif

// constants
#include <gflags/gflags.h>
DEFINE_string(data_dir, "../assets/", "data directory");

DEFINE_bool(no_imgui, false, "disable IMGUI");
DEFINE_string(obj_path, "../assets/fw_topo.obj", "obj path");

DEFINE_string(camera_file, "", "camera file name");
DEFINE_uint32(camera_fov, 60, "default camera fov");
DEFINE_bool(weak_persp, false, "use weak perspective model");

DEFINE_uint32(cam_w, 0, "camera width");
DEFINE_uint32(cam_h, 0, "camera height");

struct Session{
    std::map<WINDOW, Window> windows_;

    MeshRenderer renderer_;
    Camera camera_;

    MeshData data_;
};

GUI* GUI::instance = new GUI();

// Leak session, because otherwise the application
// crashes in the CaptureModule destructor.
// The problem might be windows-specific. Needs some investigation.
// XXX: remove when the problem is fixed
Session &session = *(new Session());

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

    session.windows_[WINDOW::MAIN].resize(1, w, h);
}

void GUI::keyboard(int key, int s, int a, int m)
{
    Camera& cam = session.camera_;
    Eigen::Matrix4f& RT = cam.extrinsic_;
    
    if(key == GLFW_KEY_F1 && a == GLFW_PRESS){
        session.data_.saveObj("cur_mesh.obj");
    }
    if(key == GLFW_KEY_F2 && a == GLFW_PRESS){
        cv::Mat img;
        screenshot(img, session.windows_[WINDOW::MAIN]);
        cv::imwrite("screenshot.png", img);
    }
    if(key == GLFW_KEY_R && a == GLFW_PRESS){
        session.camera_.extrinsic_ = oriRT;
    }
    if(key == GLFW_KEY_F && a == GLFW_PRESS){
        lookat = Eigen::ApplyTransform(session.data_.RT, getCenter(session.data_.pts_));
        RT.block<3,3>(0,0) = Eigen::Quaternion<float>(0, 1, 0, 0).toRotationMatrix();
        RT.block<3,1>(0,3) = lookat + Eigen::Vector3f(0,0,2.);
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
    
    Camera& cam = session.camera_;
    
    switch (mouse_mode)
    {
        case MouseMode::Rotation:
        {
            Eigen::Matrix4f RT = curRT;
            
            Eigen::Quaternion<float> q;
            trackball(width_, height_, rot_scale, down_mouse_x, down_mouse_y, current_mouse_x, current_mouse_y, q);
            
            Eigen::Matrix3f R = q.toRotationMatrix();
            Eigen::Matrix3f Rpre = curRT.block<3,3>(0,0);
            RT.block<3,3>(0,0) = curRT.block<3,3>(0,0)*R;
            RT.block<3,1>(0,3) = RT.block<3,3>(0,0)*Rpre.transpose()*(curRT.block<3,1>(0,3)-lookat)+lookat;
            
            cam.extrinsic_ = RT.inverse();
            break;
        }
        case MouseMode::Translation:
        {
            Eigen::Matrix4f RT = curRT;
            
            double dx = down_mouse_x - current_mouse_x;
            double dy = down_mouse_y - current_mouse_y;
            RT.block<3,1>(0,3) = tr_scale*((float)dx * right + (float)dy * up) + curRT.block<3,1>(0,3);
            
            cam.extrinsic_ = RT.inverse();
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
    curRT = session.camera_.extrinsic_.inverse();
    up = curRT.block<3,3>(0,0).inverse()*Eigen::Vector3f(0,1,0);
    right = (curRT.block<3,1>(0,3)-lookat).cross(up).normalized();
    
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
    Camera& cam = session.camera_;
    Eigen::Matrix4f RT = cam.extrinsic_.inverse();
    
    if(mb == MouseButton::Right)
        lookat += RT.block<3,1>(0,3) - curRT.block<3,1>(0,3);
    mouse_mode = MouseMode::None;
}

void GUI::mouseScroll(double x, double y)
{
    Camera& cam = session.camera_;
    Eigen::Matrix4f RT = cam.extrinsic_.inverse();

    float scale = 0.01;
    Eigen::Vector3f t = RT.block<3,1>(0,3) - lookat;
    
    t = (1.0 + scale * y)*t;
    
    RT.block<3,1>(0,3) = t + lookat;
    
    cam.extrinsic_ = RT.inverse();
}

void GUI::init(int w, int h)
{
    width_ = w;
    height_ = h;
    
    std::string data_dir = FLAGS_data_dir;
    
    // initialize GL context
    initializeGL(w, h, session.windows_);

    // load camera
    if(FLAGS_camera_file.empty())
        session.camera_ = Camera::craeteFromFOV(w, h, FLAGS_camera_fov);
    else
        session.camera_ = Camera::parseCameraParams(data_dir + FLAGS_camera_file);
    session.camera_.weakPersp_ = FLAGS_weak_persp;
    std::cout << "Camera Info:" << std::endl;
    std::cout << "Intrinsic:" << std::endl;
    std::cout << session.camera_.intrinsic_ << std::endl;
    std::cout << "Extrinsic:" << std::endl;
    std::cout << session.camera_.extrinsic_ << std::endl;

    // load obj file
    session.data_.loadObj(FLAGS_obj_path);
    
    // normalize obj size
    Eigen::Vector3f vmin, vmax;
    computeAABB(session.data_.pts_, vmin, vmax);
    Eigen::Map<Eigen::Matrix3Xf> pts(session.data_.pts_.data(),3,session.data_.pts_.size()/3);
    pts = pts.colwise()-0.5*(vmin+vmax);
    session.data_.pts_ /= (vmax[1]-vmin[1]);
    
    // initialize renderer
    session.renderer_.init(data_dir + "shaders", session.data_.tri_pts_);
    
    GLFWwindow* window = session.windows_[MAIN];

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

void GUI::loop()
{
    GLsync tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glfwHideWindow(session.windows_[MAIN]);
    lookat = Eigen::ApplyTransform(session.data_.RT,getCenter(session.data_.pts_));
    session.camera_.extrinsic_.block<3,3>(0,0) = Eigen::Quaternion<float>(0, 1, 0, 0).toRotationMatrix();
    session.camera_.extrinsic_.block<3,1>(0,3) = lookat + Eigen::Vector3f(0,0,2.0);
    {
        Eigen::Matrix4f curRT = session.camera_.extrinsic_.inverse();
        Eigen::Matrix4f RT = curRT;
        Eigen::Matrix3f R = Eigen::eulerAngleToMat(0,0.0,0);
        Eigen::Matrix3f Rpre = curRT.block<3,3>(0,0);
        RT.block<3,3>(0,0) = curRT.block<3,3>(0,0)*R;
        RT.block<3,1>(0,3) = RT.block<3,3>(0,0)*Rpre.transpose()*(curRT.block<3,1>(0,3)-lookat)+lookat;
        session.camera_.extrinsic_ = RT.inverse();
    }

    while(!glfwWindowShouldClose(session.windows_[MAIN]))
    {        
        char title[256];
        sprintf(title, "Main Window [fps: %.1f]", fps_.count());
        glfwSetWindowTitle(session.windows_[MAIN], title);

        clearBuffer(COLOR::COLOR_GREY);
        int w, h;
        glfwGetFramebufferSize(session.windows_[MAIN], &w, &h);
        glViewport(0, 0, w, h);
        
        session.renderer_.render(session.camera_, session.data_.pts_, session.data_.nml_);
        
        // retrieve framebuffer object
        cv::Mat_<cv::Vec4f> render_output;
        session.renderer_.fb_->RetrieveFBO(render_output, 0);
        cv::imshow("output", render_output);
        cv::waitKey(1);
        
#ifdef WITH_IMGUI
        if(!FLAGS_no_imgui){
            ImGui_ImplGlfwGL3_NewFrame();
            ImGui::Begin("Control Panel", &show_control_panel_);
            session.renderer_.updateIMGUI();
            session.camera_.updateIMGUI();
            session.data_.updateIMGUI();
            
            ImGui::End();
            ImGui::Render();
        }
#endif        
        session.windows_[MAIN].flush();
        
        glDeleteSync(tsync);
        tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

        glfwPollEvents();
    }
    
#ifdef WITH_IMGUI
    ImGui_ImplGlfwGL3_Shutdown();
#endif
    
    glfwTerminate();
}

