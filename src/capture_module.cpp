// contains basic module functionality

// std includes
#include <unordered_map>
#include <vector>

// internal includes
#include "capture_module.h"

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
    video_capture_.open(0);
    
    std::string command = "";
    cv::Mat frame;
    bool pause = false;
    while(command != "stop")
    {
        if(!pause)
            video_capture_ >> frame;
        CaptureResult cap;
        cv::flip(frame, cap.img, 1);
        cap.camera = camera_;
        output_frame_queue_->push(cap);
        
        if(command_queue_->front()){
            command = *command_queue_->front();
            if(command == "pause")
                pause = !pause;
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

// Factory method for basic module.
// Creates a new modules and returns its handle to user
ModuleHandle CaptureModule::Create(const std::string &name,
                                   const std::string &data_dir,
                                   CapQueueHandle out_frame_queue,
                                   CmdQueueHandle command_queue)
{
    auto module = new CaptureModule(name);
    // add this module to the global registry
    
    module->set_output_queue(out_frame_queue);
    module->set_command_queue(command_queue);
    module->camera_ = Camera::parseCameraParams(data_dir + "data/KRT.txt");
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}
