// contains a base module class
#pragma once

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include <utility/frame_loader.h>
#include <gl_utility/camera.h>

#include "module.h"

struct CaptureResult
{
    cv::Mat img;
    Camera camera;
};

typedef std::shared_ptr<SPSCQueue<CaptureResult>> CapQueueHandle;

class CaptureModule : public Module
{
public:
    // initializes a module
    CaptureModule(const std::string &name);
    // destructor
    ~CaptureModule();

    // does module specific work
    virtual void Process();

    // Provides a way to interrupt the process.
    // Default implementation does nothing.
    virtual void Stop();
    
    // Set output queue. The output queue is automatically set in Create().
    // Not thread safe.
    void set_output_queue(CapQueueHandle queue);
    
    // Set control queue. The control queue is automatically set in Create().
    // Not thread safe.
    void set_command_queue(CmdQueueHandle queue);
    
    void set_frame_loader(FrameLoaderPtr loader);

    // construct a default module
    static ModuleHandle Create(const std::string &name,
                               const std::string &data_dir,
                               FrameLoaderPtr frame_loader,
                               CapQueueHandle out_frame_queue,
                               CmdQueueHandle command_queue);
    
private:
    CapQueueHandle output_frame_queue_;
    CmdQueueHandle command_queue_;
    
    FrameLoaderPtr frame_loader_;
    Camera camera_;
};