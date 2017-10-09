//
//  GUI.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gui.hpp"

#include "obj_loader.hpp"
#include "trackball.hpp"

struct Session{
    ModuleHandle capture_module_;
    ModuleHandle face_module_;
    
    CapQueueHandle capture_queue_;
    FaceQueueHandle result_queue_;
    
    CmdQueueHandle capture_control_queue_;
    CmdQueueHandle face_control_queue_;
    
    std::thread capture_thread, face_thread;
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

    renderer_.resize(1, w, h);
}

void GUI::keyboard(int key, int s, int a, int m)
{
    Camera& cam = result_.camera;
    Eigen::Matrix4f& RT = cam.extrinsic_;
    
    if(key == GLFW_KEY_F1 && a == GLFW_PRESS){
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_.fParam.saveObj("cur_mesh.obj", *face_model_);
    }
    if(key == GLFW_KEY_F && a == GLFW_PRESS){
        std::lock_guard<std::mutex> lock(result_mutex_);
        lookat = getCenter(result_.fParam.pts_);
        RT.block<3,3>(0,0) = Eigen::Quaternion<float>(0, 1, 0, 0).toRotationMatrix();
        RT.block<3,1>(0,3) = lookat + Eigen::Vector3f(0,0,50);
    }
    if(key == GLFW_KEY_I && a == GLFW_PRESS){
        result_.fParam.init(*face_model_);
    }
    if(key == GLFW_KEY_SPACE && a == GLFW_PRESS){
        //renderer_.face_module_.enable_f2f_ = !renderer_.face_module_.enable_f2f_;
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
    
    Camera& cam = result_.camera;
    
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
    curRT = result_.camera.extrinsic_.inverse();
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
    Camera& cam = result_.camera;
    Eigen::Matrix4f RT = cam.extrinsic_.inverse();
    
    if(mb == MouseButton::Right)
        lookat += RT.block<3,1>(0,3) - curRT.block<3,1>(0,3);
    mouse_mode = MouseMode::None;
}

void GUI::mouseScroll(double x, double y)
{
    Camera& cam = result_.camera;
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
    
    std::string data_dir = "./";
    
    session.capture_queue_ = CapQueueHandle(new SPSCQueue<CaptureResult>(4));
    session.result_queue_ = FaceQueueHandle(new SPSCQueue<FaceResult>(4));
    session.capture_control_queue_ = CmdQueueHandle(new SPSCQueue<std::string>(10));
    session.face_control_queue_ = CmdQueueHandle(new SPSCQueue<std::string>(10));

    face_model_ = FaceModel::LoadModel(data_dir + "data/BVModel.bin");
    renderer_.init(w, h, face_model_, data_dir);
    
    p2d_param_ = P2DFitParamsPtr(new P2DFitParams());
    f2f_param_ = F2FParamsPtr(new F2FParams());
    
    session.capture_module_ = CaptureModule::Create("capture", data_dir, session.capture_queue_, session.capture_control_queue_);
    session.face_module_ = FaceModule::Create("face", data_dir, face_model_, p2d_param_, f2f_param_,
                                              session.capture_queue_, session.result_queue_, session.face_control_queue_);
    
    //lookat = getCenter(renderer_.face_module_.fParam_.pts_);
    
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

void GUI::loop()
{
    //GLsync tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    
    session.capture_thread = std::thread([&](){ session.capture_module_->Process(); });
    session.face_thread = std::thread([&](){ session.face_module_->Process(); });
    
    // this makes sure result has some value
    while(!session.result_queue_->front())
        ;
    result_ = *session.result_queue_->front();
    session.result_queue_->pop();
    lookat = getCenter(result_.fParam.pts_);

    while(!glfwWindowShouldClose(renderer_.windows_[MAIN]))
    {
        if(session.result_queue_->front()){
            std::lock_guard<std::mutex> lock(result_mutex_);
            auto& result = *session.result_queue_->front();
            
            result_.img = result.img;
            if(result.processed_){
                result_.camera = result.camera;
                result_.fParam = result.fParam;
                result_.c_p2l = result.c_p2l;
                result_.c_p2p = result.c_p2p;
                result_.p2d = result.p2d;
            }
            
            session.result_queue_->pop();
            result_.fParam.updateAll(*face_model_);
        }
        
        char title[256];
        sprintf(title, "Main Window [fps: %.1f]", fps_.count());
        glfwSetWindowTitle(renderer_.get_window(MAIN), title);

        clearBuffer(COLOR::COLOR_GREY);
        int w, h;
        glfwGetFramebufferSize(renderer_.get_window(MAIN), &w, &h);
        glViewport(0, 0, w, h);
        
        renderer_.draw(result_);
        
#ifdef WITH_IMGUI
        ImGui_ImplGlfwGL3_NewFrame();
        ImGui::Begin("Control Panel", &show_control_panel_);
        
        renderer_.updateIMGUI();
        result_.camera.updateIMGUI();
        result_.fParam.updateIMGUI();
        p2d_param_->updateIMGUI();
        f2f_param_->updateIMGUI();
        
        ImGui::End();
        ImGui::Render();
#endif
        
        renderer_.flush();
        
        //glDeleteSync(tsync);
        //tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

        glfwPollEvents();
    }
    
#ifdef WITH_IMGUI
    ImGui_ImplGlfwGL3_Shutdown();
#endif
    
    glfwTerminate();
}
