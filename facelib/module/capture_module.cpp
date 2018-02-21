// contains basic module functionality

// std includes
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
    frame_loader_->init();
    std::string command = "";
    while(command != "stop")
    {
        CaptureResult cap;
        frame_loader_->load_frame(cap.img, command);
        cap.camera = camera_;
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
}

// Factory method for basic module.
// Creates a new modules and returns its handle to user
ModuleHandle CaptureModule::Create(const std::string &name,
                                   const std::string &data_dir,
                                   FrameLoaderPtr frame_loader,
                                   CapQueueHandle out_frame_queue,
                                   CmdQueueHandle command_queue)
{
    auto module = new CaptureModule(name);
    // add this module to the global registry
    
    module->set_frame_loader(frame_loader);
    module->set_output_queue(out_frame_queue);
    module->set_command_queue(command_queue);
    module->camera_ = Camera::parseCameraParams(data_dir + "KRT.txt");
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}

