//
//  GUI.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gui.h"

#include <memory>
#include <map>

#include <module/capture_module.h>
#include <module/preprocess_module.h>
#include <module/face_module.h>
#include <module/face_renderer.h>

#include <renderer/base_renderer.h>
#include <renderer/bg_renderer.h>
#include <renderer/mesh_renderer.h>
#include <renderer/mp_renderer.h>
#include <renderer/mp2_renderer.h>

#include <optimizer/p2d_optimizer.h>

#include <utility/str_utils.h>
#include <utility/obj_loader.h>
#include <utility/exr_loader.h>
#include <utility/pts_loader.h>
#include <utility/trackball.h>
#include <utility/delaunator.h>

#ifdef WITH_IMGUI
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>
#endif

// constants
#include <gflags/gflags.h>
DEFINE_string(data_dir, "../assets/", "data directory");

DEFINE_bool(no_imgui, false, "disable IMGUI");
DEFINE_string(loader, "", "Loader (image file, video, integer(live stream), or empty");

DEFINE_double(loader_scale, 1.0, "image loader scale");

DEFINE_uint32(cam_w, 512, "camera width");
DEFINE_uint32(cam_h, 512, "camera height");

struct Session{
    ModuleHandle capture_module_;
    ModuleHandle preprocess_module_;
    ModuleHandle face_module_;

    CapQueueHandle capture_queue_;
    CapQueueHandle preprocess_queue_;
    FaceQueueHandle result_queue_;
    
    CmdQueueHandle capture_control_queue_;
    CmdQueueHandle preprocess_control_queue_;
    CmdQueueHandle face_control_queue_;
    
    std::thread capture_thread, preprocess_thread, face_thread;
    
    FaceRenderer renderer_;
    MP2Renderer mp2_renderer_;
    std::map<WINDOW, Window> windows_;
    
    FaceModelPtr face_model_;
//    P2DFitParamsPtr p2d_param_;
//    F2FParamsPtr f2f_param_;
//    PProParamsPtr pp_param_;
    
    std::mutex result_mutex_;
    FaceResult result_;
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
    Camera& cam = session.result_.cameras[cam_id_];
    Eigen::Matrix4f& RT = cam.extrinsic_;
    
    if(key == GLFW_KEY_F1 && a == GLFW_PRESS){
        std::lock_guard<std::mutex> lock(session.result_mutex_);
        session.result_.fd[0].saveObj("cur_mesh.obj");
    }
    if(key == GLFW_KEY_F2 && a == GLFW_PRESS){
        cv::Mat img;
        screenshot(img, session.windows_[WINDOW::MAIN]);
        cv::imwrite("screenshot.png", img);
    }
    if(key == GLFW_KEY_F && a == GLFW_PRESS){
        std::lock_guard<std::mutex> lock(session.result_mutex_);
        lookat = Eigen::ApplyTransform(session.result_.fd[frame_id_].RT, getCenter(session.result_.fd[frame_id_].pts_));
        RT.block<3,3>(0,0) = Eigen::Quaternion<float>(0, 1, 0, 0).toRotationMatrix();
        RT.block<3,1>(0,3) = lookat + Eigen::Vector3f(0,0,50);
    }
    if(key == GLFW_KEY_R && a == GLFW_PRESS){
        std::lock_guard<std::mutex> lock(session.result_mutex_);
        session.result_.cameras[cam_id_].extrinsic_ = oriRT;
    }
    if(key == GLFW_KEY_I && a == GLFW_PRESS){
        session.result_.fd[frame_id_].init();
    }
    if(key == GLFW_KEY_SPACE && a == GLFW_PRESS){
        if(!pause_)
            session.capture_control_queue_->push("pause");
        else
            session.capture_control_queue_->push("");
        pause_ = !pause_;
    }
    if(key == GLFW_KEY_P && a == GLFW_PRESS){
        session.capture_control_queue_->push("pause");
        session.capture_control_queue_->push("");
        session.capture_control_queue_->push("pause");
    }
//    if(key == GLFW_KEY_P && a == GLFW_PRESS){
//        auto ren = std::static_pointer_cast<MPRenderer>(session.renderer_.renderer_["MP"]);
//        cv::Mat_<cv::Vec4f> tmp;
//        ren->fb_tex_->RetrieveFBO(tmp, 0);
//        cv::imwrite("tex.png",255.0*tmp);
//    }
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
    
    Camera& cam = session.result_.cameras[frame_id_];
    
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
    curRT = session.result_.cameras[frame_id_].extrinsic_.inverse();
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
    Camera& cam = session.result_.cameras[frame_id_];
    Eigen::Matrix4f RT = cam.extrinsic_.inverse();
    
    if(mb == MouseButton::Right)
        lookat += RT.block<3,1>(0,3) - curRT.block<3,1>(0,3);
    mouse_mode = MouseMode::None;
}

void GUI::mouseScroll(double x, double y)
{
    Camera& cam = session.result_.cameras[frame_id_];
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
    
    initializeGL(w, h, session.windows_);
    
    session.capture_queue_ = CapQueueHandle(new SPSCQueue<CaptureResult>(4));
    session.preprocess_queue_ = CapQueueHandle(new SPSCQueue<CaptureResult>(4));
    session.result_queue_ = FaceQueueHandle(new SPSCQueue<FaceResult>(4));
    session.capture_control_queue_ = CmdQueueHandle(new SPSCQueue<std::string>(10));
    session.preprocess_control_queue_ = CmdQueueHandle(new SPSCQueue<std::string>(10));
    session.face_control_queue_ = CmdQueueHandle(new SPSCQueue<std::string>(10));

    session.face_model_ = BiLinearFaceModel::LoadModel(data_dir + "FWModel_BLv2.bin", "fw");

    session.renderer_.addRenderer("BG", BGRenderer::Create("BG Rendering", true));
    session.renderer_.addRenderer("Geo", MeshRenderer::Create("Geo Rendering", true));
    session.renderer_.addRenderer("MP", MPRenderer::Create("MP Rendering", true));
    
    session.renderer_.init(session.face_model_, data_dir);
    session.mp2_renderer_.init(data_dir,data_dir+"shaders",session.face_model_->tri_pts_);
    
    auto frame_loader = EmptyLoader::Create();
    
    if( FLAGS_loader.find("jpg") != std::string::npos ||
       FLAGS_loader.find("png") != std::string::npos ||
       FLAGS_loader.find("bmp") != std::string::npos){
        frame_loader = SingleImageLoader::Create(FLAGS_loader, FLAGS_loader_scale);
    }
    
    int cam_w = FLAGS_cam_w != 0 ? FLAGS_cam_w : w;
    int cam_h = FLAGS_cam_h != 0 ? FLAGS_cam_h : h;
    session.capture_module_ = CaptureModule::Create("capture", data_dir, cam_w, cam_h, frame_loader,
                                                    session.capture_queue_, session.capture_control_queue_);

    auto face_detector = std::shared_ptr<Face2DDetector>(new Face2DDetector(data_dir));

    session.preprocess_module_ = PreprocessModule::Create("face", face_detector, session.capture_queue_,
                                                          session.preprocess_queue_, session.preprocess_control_queue_);
    
    std::map<std::string, OptimizerHandle> optimizers;
    optimizers["p2d"] = P2DOptimizer::Create("Landmark Fitting");
    session.face_module_ = FaceOptModule::Create("face", data_dir, session.face_model_, optimizers,
                                                 session.preprocess_queue_, session.result_queue_,
                                                 session.face_control_queue_);
    
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

void extractMask(FaceRenderer& renderer, FaceResult& result, int grid_size)
{
    auto ren = std::static_pointer_cast<MPRenderer>(renderer.renderer_["MP"]);
    
    ren->render(result, 0, 0);
    
    cv::Mat_<cv::Vec4f> mat;
    ren->fb_plane_->RetrieveFBO(mat, 0);
    
    cv::Mat_<uchar> mask(mat.size(),0);
    for(int i = 0; i < mat.rows; ++i)
    {
        for(int j = 0; j < mat.cols; ++j)
        {
            if(mat(i,j)[3] == 0.0)
                mask(i,j) = 255;
        }
    }
    
    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = 10;
    cv::Mat element = cv::getStructuringElement( erosion_type,
                                                 cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                 cv::Point( erosion_size, erosion_size ) );
    cv::erode(mask, mask, element);
    
    auto pts = result.fd[0].pts_;
    auto nml = result.fd[0].nml_;
    auto uvs = result.fd[0].uvs();
    std::vector<bool> flags(pts.size()/3,true);
    auto tripts = result.fd[0].tripts();
    auto triuvs = result.fd[0].triuv();
    const cv::Mat_<uchar>& uv_mask = ren->mask_;
    for(int i = 0; i < tripts.rows(); ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            int ux = (int)((float)uv_mask.cols*uvs(triuvs(i,j),0));
            int uy = (int)((float)uv_mask.rows*uvs(triuvs(i,j),1));
            if(uv_mask(uy,ux) == 0)
                flags[tripts(i,j)] = false;
        }
    }
    Eigen::Vector4f p, n;
    Eigen::Matrix4f RT = result.cameras[0].extrinsic_ * result.fd[0].RT;
    Eigen::Matrix4f K = result.cameras[0].intrinsic_;
    std::vector<unsigned int> vidx;
    std::vector<double> pnts;
    for(int i = 0; i < pts.size()/3; ++i)
    {
        if(!flags[i])
            continue;
        n << nml.row(i).transpose(), 0.0;
        p << pts.b3(i), 1.0;
        
        n = RT * n;
        p = RT * p;
        if(n.dot(p) >= 0.0)
            continue;
        p = K * p;
        double px = p[0]/p[2];
        double py = p[1]/p[2];
        pnts.push_back(px);
        pnts.push_back(py);
        vidx.push_back(i);
    }
    
    int step_x = mask.cols / grid_size;
    int step_y = mask.rows / grid_size;
    int cnt = pts.size()/3;
    std::vector<glm::vec3> pos;
    double d_ave = 0.99;
    for(int i = 0; i < mask.cols; i += step_x)
    {
        for(int j = 0; j < mask.rows; j += step_y)
        {
            if(j < mask.rows/2 && (j % 6 != 0 || i % 6 != 0))
                continue;
            if(mask(j,i) == 255){
                pnts.push_back((double)i);
                pnts.push_back((double)j);
                vidx.push_back(cnt++);
                pos.push_back(glm::vec3((double)i/(double)(mask.cols-1),(double)j/(double)(mask.rows-1),d_ave));
                
                if(i+step_x >= mask.cols && j+step_y >= mask.rows){
                    pnts.push_back((double)(mask.cols-1));
                    pnts.push_back((double)(mask.rows-1));
                    vidx.push_back(cnt++);
                    pos.push_back(glm::vec3(1.0,1.0,d_ave));

                }
                else if(i+step_x >= mask.cols){
                    pnts.push_back((double)(mask.cols-1));
                    pnts.push_back((double)j);
                    vidx.push_back(cnt++);
                    pos.push_back(glm::vec3(1.0,(double)j/(double)(mask.rows-1),d_ave));
                }
                else if(j+step_y >= mask.rows){
                    pnts.push_back((double)i);
                    pnts.push_back((double)(mask.rows-1));
                    vidx.push_back(cnt++);
                    pos.push_back(glm::vec3((double)i/(double)(mask.cols-1),1.0,d_ave));
                }
            }
        }
    }
    
    delaunator::Delaunator d(pnts);
    std::vector<unsigned int> tri_list;
    for(size_t i = 0; i < d.triangles.size(); i+=3) {
        int t1 = d.triangles[i];
        int t2 = d.triangles[i+1];
        int t3 = d.triangles[i+2];
        tri_list.push_back(vidx[d.triangles[i]]);
        tri_list.push_back(vidx[d.triangles[i+1]]);
        tri_list.push_back(vidx[d.triangles[i+2]]);
        cv::line(mask,cv::Point(pnts[t1*2+0],pnts[t1*2+1]),cv::Point(pnts[t2*2+0],pnts[t2*2+1]),cv::Scalar(122));
        cv::line(mask,cv::Point(pnts[t1*2+0],pnts[t1*2+1]),cv::Point(pnts[t3*2+0],pnts[t3*2+1]),cv::Scalar(122));
        cv::line(mask,cv::Point(pnts[t3*2+0],pnts[t3*2+1]),cv::Point(pnts[t2*2+0],pnts[t2*2+1]),cv::Scalar(122));
    }
    
    session.mp2_renderer_.render(result.cameras[0], result.fd[0].getRT(),
                                 result.fd[0].pts_, tri_list, pos, result.cap_data[0][0].img_);

    cv::imshow("mask",mask);
}

void GUI::loop()
{
    GLsync tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

    session.capture_thread = std::thread([&](){ session.capture_module_->Process(); });
    session.preprocess_thread = std::thread([&](){ session.preprocess_module_->Process(); });
    session.face_thread = std::thread([&](){ session.face_module_->Process(); });
    
    // this makes sure result has some value
    while(!session.result_queue_->front())
        ;
    // update lookat
    session.result_ = *session.result_queue_->front();
    
    lookat = Eigen::ApplyTransform(session.result_.fd[frame_id_].RT,getCenter(session.result_.fd[frame_id_].pts_));
    while(!glfwWindowShouldClose(session.windows_[MAIN]))
    {
        if(session.result_queue_->front()){
            std::lock_guard<std::mutex> lock(session.result_mutex_);
            auto& result = *session.result_queue_->front();
            
            if(result.processed_){
                session.result_.processed_ = true;
                session.result_ = result;
                oriRT = session.result_.cameras[cam_id_].extrinsic_;
            }
            else{
                session.result_.cap_data = result.cap_data;
                session.result_.processed_ = false;
            }
            session.result_queue_->pop();
        
            session.result_.fd[frame_id_].updateAll();
        }
        char title[256];
        sprintf(title, "Main Window [fps: %.1f]", fps_.count());
        glfwSetWindowTitle(session.windows_[MAIN], title);

        clearBuffer(COLOR::COLOR_GREY);
        int w, h;
        glfwGetFramebufferSize(session.windows_[MAIN], &w, &h);
        glViewport(0, 0, w, h);
        
        extractMask(session.renderer_, session.result_, 40);
        session.renderer_.draw(session.result_,cam_id_,frame_id_);
        
#ifdef WITH_IMGUI
        if(!FLAGS_no_imgui){
            ImGui_ImplGlfwGL3_NewFrame();
            ImGui::Begin("Control Panel", &show_control_panel_);
            session.preprocess_module_->updateIMGUI();
            session.face_module_->updateIMGUI();
            session.renderer_.updateIMGUI();
            session.mp2_renderer_.updateIMGUI();
            session.result_.cameras[cam_id_].updateIMGUI();
            session.result_.fd[frame_id_].updateIMGUI();
            
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

