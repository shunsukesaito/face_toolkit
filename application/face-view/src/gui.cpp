//
//  GUI.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gui.h"

#include <memory>

#include <module/capture_module.h>
#include <module/preprocess_module.h>
#include <module/face_module.h>
#include <module/face_renderer.h>

#include <renderer/base_renderer.h>
#include <renderer/bg_renderer.h>
#include <renderer/mesh_renderer.h>
#include <renderer/IBL_renderer.h>
#include <renderer/LS_renderer.h>
#include <renderer/LSGeo_renderer.h>
#include <renderer/DeepLS_renderer.h>
#include <renderer/p3d_renderer.h>
#include <renderer/p2d_renderer.h>
#include <renderer/posmap_renderer.h>
#include <f2f/f2f_renderer.h>

#include <utility/str_utils.h>
#include <utility/obj_loader.h>
#include <utility/exr_loader.h>
#include <utility/pts_loader.h>
#include <utility/trackball.h>

#ifdef WITH_IMGUI
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>
#endif

// constants
#include <gflags/gflags.h>
DEFINE_string(data_dir, "../assets/", "data directory");

// for batch face fitting
DEFINE_bool(dump_pca, false, "dumping out pca albedo texture map");
// for batch rendering
DEFINE_bool(center_cam, false, "rendering with face center");

DEFINE_bool(fd_record, false, "dumping out frames for facedata");
DEFINE_bool(no_imgui, false, "disable IMGUI");
DEFINE_string(mode, "opt", "view mode (opt/preview)");
DEFINE_string(facemodel, "pin", "FaceModel to use");
DEFINE_string(renderer, "geo", "Renderer to use");
DEFINE_string(fd_path, "", "FaceData path");
DEFINE_string(loader, "", "Loader (image file, video, integer(live stream), or empty");
DEFINE_int32(fd_begin_id, -1, "FaceData start frame id");
DEFINE_int32(fd_end_id, -1, "FaceData end frame id");

DEFINE_string(fd_ip, "127.0.0.1", "IP for face parameters");
DEFINE_int32(fd_port, 19820, "port number for face parameters");
DEFINE_int32(fd_dof_id, 40, "FaceData TCP dof of ID");
DEFINE_int32(fd_dof_ex, 51, "FaceData TCP dof of EX");
DEFINE_int32(fd_dof_al, 40, "FaceData TCP dof of AL");
DEFINE_int32(fd_dof_rf, 9, "FaceData TCP dof of ROT");
DEFINE_int32(fd_dof_tf, 3, "FaceData TCP dof of TR");
DEFINE_int32(fd_dof_sh, 27, "FaceData TCP dof of SH");

DEFINE_double(loader_scale, 1.0, "image loader scale");

DEFINE_uint32(cam_w, 0, "camera width");
DEFINE_uint32(cam_h, 0, "camera height");

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
    std::map<WINDOW, Window> windows_;
    
    FaceModelPtr face_model_;
    P2DFitParamsPtr p2d_param_;
    F2FParamsPtr f2f_param_;
    PProParamsPtr pp_param_;
    
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

    if( FLAGS_facemodel.find("pinpca") != std::string::npos )
        session.face_model_ = LinearFaceModel::LoadModel(data_dir + "PinModelPCA.bin", "pin_pca");
    else if( FLAGS_facemodel.find("pinfacs") != std::string::npos )
        session.face_model_ = LinearFaceModel::LoadModel(data_dir + "PinModelFACS.bin", "pin_facs");
    else if( FLAGS_facemodel.find("bv") != std::string::npos )
        session.face_model_ = LinearFaceModel::LoadModel(data_dir + "BVModel.bin", "bv");
    else if(FLAGS_facemodel.find("deepls") != std::string::npos ){
        int start = FLAGS_facemodel.find("deepls") + 6;
        int len = FLAGS_facemodel.size()-6;
        std::string fm_name =  FLAGS_facemodel.substr(start,len);
        session.face_model_ = LinearFaceModel::LoadLSData(data_dir + "LS/" + fm_name + "/" , true);
    }
    else if(FLAGS_facemodel.find("ls") != std::string::npos ){
        int start = FLAGS_facemodel.find("ls") + 2;
        int len = FLAGS_facemodel.size()-2;
        std::string fm_name = FLAGS_facemodel.substr(start, len);
        session.face_model_ = LinearFaceModel::LoadLSData(data_dir + "LS/" + fm_name + "/" );
    }
    else if(FLAGS_facemodel.find("fwbl") != std::string::npos)
        session.face_model_ = BiLinearFaceModel::LoadModel(data_dir + "FWModel_BL.bin", "fw");
    else
        throw std::runtime_error("Unrecongnized face model... "+FLAGS_facemodel);

    auto renderers = vector2map(split_str(FLAGS_renderer, "/"));
    
    if( renderers.find("bg") != renderers.end() )
        session.renderer_.addRenderer("BG", BGRenderer::Create("BG Rendering", true));
    if( renderers.find("geo") != renderers.end() )
        session.renderer_.addRenderer("Geo", MeshRenderer::Create("Geo Rendering", true));
    if( renderers.find("ibl") != renderers.end() )
        session.renderer_.addRenderer("IBL", IBLRenderer::Create("IBL Rendering", true));
    if( renderers.find("deepls") != renderers.end() )
        session.renderer_.addRenderer("DeepLS", DeepLSRenderer::Create("DeepLS Rendering", true));
    if( renderers.find("ls") != renderers.end() )
        session.renderer_.addRenderer("LS", LSRenderer::Create("LS Rendering", true));
    if( renderers.find("lsgeo") != renderers.end() )
        session.renderer_.addRenderer("LSGeo", LSGeoRenderer::Create("LSGeo Rendering", true));
    if( renderers.find("pmrec") != renderers.end() )
        session.renderer_.addRenderer("PMRec", PosMapReconRenderer::Create("PosMapRecon Rendering", true));
    if( renderers.find("pm") != renderers.end() )
        session.renderer_.addRenderer("PM", PosMapRenderer::Create("PosMap Rendering", true));
    if( renderers.find("f2f") != renderers.end() )
        session.renderer_.addRenderer("F2F", F2FRenderer::Create("F2F Rendering", true));
    if( renderers.find("p3d") != renderers.end() )
        session.renderer_.addRenderer("P3D", P3DRenderer::Create("P3D Rendering", true));
    if( renderers.find("p2d") != renderers.end() )
        session.renderer_.addRenderer("P2D", P2DRenderer::Create("P2D Rendering", true));
    
    if( session.renderer_.size() == 0)
        throw std::runtime_error("No renderer is set... " +FLAGS_renderer);
    
    session.renderer_.init(session.face_model_, data_dir);
    
    session.p2d_param_ = P2DFitParamsPtr(new P2DFitParams());
    session.f2f_param_ = F2FParamsPtr(new F2FParams());
    session.pp_param_ = PProParamsPtr(new PProParams());
    
    auto frame_loader = EmptyLoader::Create();
    
    if( FLAGS_loader.find("%") != std::string::npos &&
       (FLAGS_loader.find("jpg") != std::string::npos ||
        FLAGS_loader.find("png") != std::string::npos ||
        FLAGS_loader.find("bmp") != std::string::npos)){
        std::string root_dir = FLAGS_loader.substr(0,FLAGS_loader.find_last_of("/"));
        std::string img_fmt = FLAGS_loader.substr(FLAGS_loader.find_last_of("/")+1,FLAGS_loader.size()-FLAGS_loader.find_last_of("/")-1);
        frame_loader = ImageSequenceLoader::Create(root_dir, img_fmt, 0, 5000);
    }
    else if( FLAGS_loader.find("jpg") != std::string::npos ||
       FLAGS_loader.find("png") != std::string::npos ||
       FLAGS_loader.find("bmp") != std::string::npos){
        frame_loader = SingleImageLoader::Create(FLAGS_loader, FLAGS_loader_scale);
    }
    else if( FLAGS_loader.find("avi") != std::string::npos ||
            FLAGS_loader.find("mp4") != std::string::npos ||
            FLAGS_loader.find("mov") != std::string::npos){
        frame_loader = VideoLoader::Create(FLAGS_loader, FLAGS_loader_scale);
    }
    else if( FLAGS_loader.find("txt") != std::string::npos){
        std::string root_dir = FLAGS_loader.substr(0,FLAGS_loader.find_last_of("/"));
        frame_loader = ImageSequenceLoader::Create(root_dir, FLAGS_loader, FLAGS_loader_scale);
    }
    else if( !FLAGS_loader.empty() ){
        int vid;
        std::istringstream( FLAGS_loader ) >> vid;
        frame_loader = VideoLoader::Create(vid, FLAGS_loader_scale);
    }
    
    int cam_w = FLAGS_cam_w != 0 ? FLAGS_cam_w : w;
    int cam_h = FLAGS_cam_h != 0 ? FLAGS_cam_h : h;
    session.capture_module_ = CaptureModule::Create("capture", data_dir, cam_w, cam_h, frame_loader,
                                                    session.capture_queue_, session.capture_control_queue_);
    if(FLAGS_mode == "preview")
        session.face_module_ = FacePreviewModule::Create("face", data_dir, session.face_model_, session.capture_queue_,
                                                         session.result_queue_, session.face_control_queue_,
                                                         FLAGS_fd_path, FLAGS_fd_begin_id, FLAGS_fd_end_id);
    else if(FLAGS_mode == "brender"){
        std::string root_dir = FLAGS_loader.substr(0,FLAGS_loader.find_last_of("/"));
        session.face_module_ = FacePreviewModule::Create("face", data_dir, session.face_model_, session.capture_queue_,
                                                         session.result_queue_, session.face_control_queue_,
                                                         root_dir, FLAGS_loader );
    }
    else if(FLAGS_mode == "preview_vgpt"){
        std::vector<std::pair<std::string, int>> dof;
        dof.push_back(std::pair<std::string,int>("id",FLAGS_fd_dof_id));
        dof.push_back(std::pair<std::string,int>("ex",FLAGS_fd_dof_ex));
        dof.push_back(std::pair<std::string,int>("al",FLAGS_fd_dof_al));
        dof.push_back(std::pair<std::string,int>("rf",FLAGS_fd_dof_rf));
        dof.push_back(std::pair<std::string,int>("tf",FLAGS_fd_dof_tf));
        dof.push_back(std::pair<std::string,int>("sh",FLAGS_fd_dof_sh));
        session.face_module_ = FacePreviewModule::Create("face", data_dir, session.face_model_, session.capture_queue_,
                                                         session.result_queue_, session.face_control_queue_, dof,
                                                         FLAGS_fd_path, FLAGS_fd_begin_id, FLAGS_fd_end_id);
    }
    else if(FLAGS_mode == "tcp"){
        std::vector<std::pair<std::string, int>> dof;
        dof.push_back(std::pair<std::string,int>("id",FLAGS_fd_dof_id));
        dof.push_back(std::pair<std::string,int>("ex",FLAGS_fd_dof_ex));
        dof.push_back(std::pair<std::string,int>("al",FLAGS_fd_dof_al));
        dof.push_back(std::pair<std::string,int>("rf",FLAGS_fd_dof_rf));
        dof.push_back(std::pair<std::string,int>("tf",FLAGS_fd_dof_tf));
        dof.push_back(std::pair<std::string,int>("sh",FLAGS_fd_dof_sh));
        session.face_module_ = FacePreviewModule::Create("face", data_dir, session.face_model_, session.capture_queue_,
                                                         session.result_queue_, session.face_control_queue_,
                                                         FLAGS_fd_ip, FLAGS_fd_port,dof,256,false);
    }
    else if(FLAGS_mode == "opt" || FLAGS_mode == "bopt"){
        auto face_detector = std::shared_ptr<Face2DDetector>(new Face2DDetector(data_dir));

        session.preprocess_module_ = PreprocessModule::Create("face", session.pp_param_, face_detector, session.capture_queue_,
                                                              session.preprocess_queue_, session.preprocess_control_queue_);

        session.face_module_ = FaceOptModule::Create("face", data_dir, session.face_model_, session.p2d_param_, session.f2f_param_,
                                                     session.preprocess_queue_, session.result_queue_, session.face_control_queue_);
    }
    else
        throw std::runtime_error("Unrecognized mode... "+FLAGS_mode);
    
    
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

// for batch face fitting
void save_result(FaceResult& result)
{
    std::cout << result.name << std::endl;
    std::string filename = result.name;
    result.fd[0].saveObj(filename.substr(0,filename.size()-4) + ".obj");
    cv::imwrite(filename.substr(0,filename.size()-4) + "_seg.png", result.cap_data[0][0].seg_);
    write_pts(filename.substr(0,filename.size()-4) + ".pts", result.cap_data[0][0].q2V_);
    auto r = session.renderer_.renderer_["F2F"];
    auto f2f_r = std::static_pointer_cast<F2FRenderer>(r);
    std::vector<cv::Mat_<cv::Vec4f>> outs;
    f2f_r->param_.tex_mode = 1;
    f2f_r->param_.enable_seg = 1;
    f2f_r->param_.enable_tex = 1;
    f2f_r->param_.enable_mask = 1;
    f2f_r->param_.enable_inv_diffuse = 1;
    f2f_r->param_.enable_cull = 1;
    f2f_r->param_.cull_offset = -0.25;
    f2f_r->updateSegment(result.cap_data[0][0].seg_);
    f2f_r->programs_["f2f"].updateTexture("u_sample_texture", result.cap_data[0][0].img_);
    f2f_r->render(1024,1024,result.cameras[0], result.fd[0], outs);
    cv::Mat_<cv::Vec4f> tmp;
    cv::cvtColor(outs[2],tmp,CV_RGBA2BGRA);
    for(int h = 0; h < tmp.rows; ++h)
    {
        for(int w = 0; w < tmp.cols; ++w)
        {
            if(tmp(h,w)[3] == 0)
                tmp(h, w) = cv::Vec4f(0,1,0,1.0);
        }
    }
    cv::Mat out;
    cv::cvtColor(tmp, out, CV_BGRA2BGR);
    out = 255.0 * out;
    out.convertTo(out, CV_8UC3);
    cv::imwrite(filename.substr(0,filename.size()-4) + "_tex.png", out);
    if (FLAGS_dump_pca){
        cv::cvtColor(outs[4],tmp,CV_RGBA2BGRA);
        for(int h = 0; h < tmp.rows; ++h)
        {
            for(int w = 0; w < tmp.cols; ++w)
            {
                if(tmp(h,w)[3] == 0)
                    tmp(h, w) = cv::Vec4f(0,1,0,1.0);
            }
        }
        cv::cvtColor(tmp, out, CV_BGRA2BGR);
        out = 255.0 * out;
        out.convertTo(out, CV_8UC3);
        cv::imwrite(filename.substr(0,filename.size()-4) + "_tex_inv.png", out);
        f2f_r->param_.tex_mode = 1;
        f2f_r->param_.enable_mask = 0;
        f2f_r->param_.enable_seg = 0;
        f2f_r->param_.enable_tex = 0;
        f2f_r->param_.enable_inv_diffuse = 0;
        f2f_r->param_.enable_cull = 0;
        f2f_r->render(1024,1024,result.cameras[0],result.fd[0],outs);
        // mixChannels: split [rgba] image to [bgr] and [a]
        cv::Mat bgr( 1024, 1024, CV_32FC3 );
        cv::Mat alpha( 1024, 1024, CV_32FC1 );
        cv::Mat_<uchar> mask(1024,1024,uchar(255));
        cv::Mat comp[] = { bgr, alpha };
        // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
        // rgba[2] -> bgr[0], rgba[3] -> alpha[0]
        int from_to[] = { 0,2, 1,1, 2,0, 3,3 };
        mixChannels( &outs[2], 1, comp, 2, from_to, 4 );
        mask.setTo(0,alpha != 0.0f);
        bgr = 255.0*bgr;
        bgr.convertTo(bgr, CV_8UC3);
        cv::inpaint(bgr, mask, bgr, 3.0, cv::INPAINT_TELEA);
        cv::imwrite(filename.substr(0,filename.size()-4) + "_tex_pca.png", bgr);
    }
    
    f2f_r->param_.tex_mode = 0;
    f2f_r->param_.enable_mask = 1;
    f2f_r->param_.enable_seg = 1;
    f2f_r->param_.enable_tex = 0;
    f2f_r->param_.cull_offset = 0.0;
    
    result.saveToTXT(filename.substr(0,filename.size()-4) + "_params.txt");
}

// for batch rendering
void save_render(FaceResult& result)
{
    std::cout << "processing... " << result.name << std::endl;
    std::string filename = result.name;
    session.face_model_->maps_.resize(3);
    cv::Mat_<cv::Vec4f> disp;
    loadEXRToCV(filename.substr(0,filename.find_last_of("/")) + "/disp.exr", disp);
    session.face_model_->maps_[0] = GLTexture::CreateTexture(disp);
    cv::Mat_<cv::Vec3b> diff = cv::imread(filename.substr(0,filename.find_last_of("/")) + "/diff.png");
    cv::flip(diff,diff,0);
    session.face_model_->maps_[1] = GLTexture::CreateTexture(diff);
    cv::Mat_<cv::Vec3b> spec = cv::imread(filename.substr(0,filename.find_last_of("/")) + "/spec.png");
    cv::flip(spec,spec,0);
    session.face_model_->maps_[2] = GLTexture::CreateTexture(spec);
    
    if (FLAGS_center_cam){
        result.cameras[0] = Camera::craeteFromFOV(1024, 1024, 40);
        Eigen::Matrix4f RT;
        RT << 1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 40.0,
        0, 0, 0, 1;
        result.cameras[0].extrinsic_ = RT;
        result.fd[0].RT = Eigen::Matrix4f::Identity();
        result.fd[0].updateAll();
    }
    
    auto r1 = session.renderer_.renderer_["DeepLS"];
    auto dls_r = std::static_pointer_cast<DeepLSRenderer>(r1);
    auto r2 = session.renderer_.renderer_["LSGeo"];
    auto lsg_r = std::static_pointer_cast<LSGeoRenderer>(r2);
    
    std::vector<cv::Mat_<cv::Vec4f>> outs;
    cv::Mat out;
    dls_r->render(result);
    dls_r->fb_->RetrieveFBO(outs);
    cv::cvtColor(outs[3],outs[3],CV_RGBA2BGRA);
    cv::cvtColor(outs[4],outs[4],CV_RGBA2BGRA);
    outs[3] = 255.0*outs[3];
    outs[4] = 255.0*outs[4];
    outs[3].convertTo(out, CV_8UC4);
    cv::imwrite(filename.substr(0,filename.find_last_of("/")) + "/render_diff.png", out);
    outs[4].convertTo(out, CV_8UC4);
    cv::imwrite(filename.substr(0,filename.find_last_of("/")) + "/render_spec.png", out);
    
    outs.clear();
    lsg_r->render(result);
    lsg_r->fb_->RetrieveFBO(outs);
    cv::cvtColor(outs[0],outs[0],CV_RGBA2BGRA);
    outs[0] = 255.0*outs[0];
    outs[0].convertTo(out, CV_8UC4);
    cv::imwrite(filename.substr(0,filename.find_last_of("/")) + "/render_disp.png", out);
}


void GUI::loop()
{
    GLsync tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

    session.capture_thread = std::thread([&](){ session.capture_module_->Process(); });
    if(FLAGS_mode.find("opt") != std::string::npos)
        session.preprocess_thread = std::thread([&](){ session.preprocess_module_->Process(); });
    session.face_thread = std::thread([&](){ session.face_module_->Process(); });
    
    // this makes sure result has some value
    while(!session.result_queue_->front())
        ;
    // update lookat
    session.result_ = *session.result_queue_->front();
    lookat = Eigen::ApplyTransform(session.result_.fd[frame_id_].RT,getCenter(session.result_.fd[frame_id_].pts_));
    bool init_frame = true;
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
            
            if(session.result_.processed_ && (FLAGS_mode == "brender" || FLAGS_mode == "bopt")){
                if(session.result_.frame_id == 0){
                    if(init_frame)
                        init_frame = false;
                    else
                        break;
                }
                if(FLAGS_mode == "brender")
                    save_render(session.result_);
                if(FLAGS_mode == "bopt")
                    save_result(session.result_);
            }
        }
        
        char title[256];
        sprintf(title, "Main Window [fps: %.1f]", fps_.count());
        glfwSetWindowTitle(session.windows_[MAIN], title);

        clearBuffer(COLOR::COLOR_GREY);
        int w, h;
        glfwGetFramebufferSize(session.windows_[MAIN], &w, &h);
        glViewport(0, 0, w, h);
        
        session.renderer_.draw(session.result_,cam_id_,frame_id_);
        
#ifdef WITH_IMGUI
        if(!FLAGS_no_imgui){
            ImGui_ImplGlfwGL3_NewFrame();
            ImGui::Begin("Control Panel", &show_control_panel_);
            if(FLAGS_mode.find("opt") != std::string::npos){
                session.pp_param_->updateIMGUI();
                session.p2d_param_->updateIMGUI();
                session.f2f_param_->updateIMGUI();
            }
            session.renderer_.updateIMGUI();
            session.result_.cameras[cam_id_].updateIMGUI();
            session.result_.fd[frame_id_].updateIMGUI();
            
            ImGui::End();
            ImGui::Render();
        }
#endif        
        session.windows_[MAIN].flush();
        
        if(FLAGS_fd_record && session.result_.processed_){
            cv::Mat img;
            screenshot(img, session.windows_[MAIN]);
            cv::imwrite(std::to_string(session.result_.frame_id) + ".png", img);
        }
        if(session.result_.frame_id == FLAGS_fd_end_id) break;
        
        glDeleteSync(tsync);
        tsync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

        glfwPollEvents();
    }
    
#ifdef WITH_IMGUI
    ImGui_ImplGlfwGL3_Shutdown();
#endif
    
    glfwTerminate();
}

