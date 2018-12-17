// contains basic module functionality

// std includes
#include <vector>

// internal includes
#include "capture_module.h"

#include <gflags/gflags.h>
DEFINE_string(camera_file, "", "camera file name");
DEFINE_uint32(camera_fov, 60, "default camera fov");
DEFINE_bool(weak_persp, false, "use weak perspective model");
DEFINE_bool(cam_c2w, false, "convert camera space to world");

// initializes this module and the basic module
CaptureModule::CaptureModule(const std::string &name)
: Module(name)
{
    // nothing to do
}

// default destructor
CaptureModule::~CaptureModule()
{
    // nothing to do
}

// stub
void CaptureModule::Process()
{
    frame_loader_->init();
    std::string command = "";
    while(command != "stop")
    {
        CaptureResult cap;
        frame_loader_->load_frame(cap.data.img_, frame_id_, cap.data.name_, command);
        cap.camera = camera_;
        cap.data.frame_id_ = frame_id_;
        output_frame_queue_->push(cap);
        
        if(command_queue_->front()){
            command = *command_queue_->front();
            command_queue_->pop();
        }
    }
}

// the interruption point
void CaptureModule::Stop()
{
    // nothing to do
}

void CaptureModule::set_output_queue(CapQueueHandle queue)
{
    output_frame_queue_ = queue;
}

void CaptureModule::set_command_queue(CmdQueueHandle queue)
{
    command_queue_ = queue;
}

void CaptureModule::set_frame_loader(FrameLoaderPtr loader)
{
    frame_loader_ = loader;
    frame_id_ = -1;
}

// Factory method for basic module.
// Creates a new modules and returns its handle to user
ModuleHandle CaptureModule::Create(const std::string &name,
                                   const std::string &data_dir,
                                   int &w,
                                   int &h,
                                   FrameLoaderPtr frame_loader,
                                   CapQueueHandle out_frame_queue,
                                   CmdQueueHandle command_queue)
{
    auto module = new CaptureModule(name);
    // add this module to the global registry
    
    module->set_frame_loader(frame_loader);
    module->set_output_queue(out_frame_queue);
    module->set_command_queue(command_queue);
    if(FLAGS_camera_file.empty() && w > 0 && h > 0)
        module->camera_ = Camera::craeteFromFOV(w, h, FLAGS_camera_fov);
    else if (FLAGS_camera_file.empty()){
        cv::Mat img;
        std::string name;
        int frame_id = 0;
        frame_loader->load_frame(img, frame_id, name, "");
        module->camera_ = Camera::craeteFromFOV(img.cols, img.rows, FLAGS_camera_fov);
        w = img.cols;
        h = img.rows;
    }
    else
        module->camera_ = Camera::parseCameraParams(data_dir + FLAGS_camera_file, FLAGS_cam_c2w);
    
    module->camera_.weakPersp_ = FLAGS_weak_persp;
    
    std::cout << module->camera_ << std::endl;

    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}

