// contains a base module class
#pragma once

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <utility/frame_loader.h>
#include <utility/capture_data.h>
#include <gl_utility/camera.h>

#include "module.h"

struct CaptureResult
{
    Camera camera;
    CaptureData data;

    int mode = 0; // 0: nothing, 1: p2d, 2: seg, 3: both 
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
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI(){}
#endif
    
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
                               int &w,
                               int &h,
                               FrameLoaderPtr frame_loader,
                               CapQueueHandle out_frame_queue,
                               CmdQueueHandle command_queue);
    
private:
    CapQueueHandle output_frame_queue_;
    CmdQueueHandle command_queue_;
    
    FrameLoaderPtr frame_loader_;
    Camera camera_;
    int frame_id_;

    bool run_track_ = false;
};
