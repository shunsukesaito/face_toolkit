// contains a base module class
#pragma once

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include <face2d-detector/face2d_detector.h>
#include <seg_stream.h>

#include "module.h"
#include "capture_module.h"

struct PProParams
{
    bool update_land_ = false;
    bool update_seg_ = false;
    
    bool onetime_land_ = false;
    bool onetime_seg_ = false;
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif

};

typedef std::shared_ptr<PProParams> PProParamsPtr;

class PreprocessModule : public Module
{
public:
    // initializes a module
    PreprocessModule(const std::string &name);
    // destructor
    ~PreprocessModule();

    void init(Face2DDetectorPtr face_detector);
    
    void update(CaptureResult& result);

    // does module specific work
    virtual void Process();

    // Provides a way to interrupt the process.
    // Default implementation does nothing.
    virtual void Stop();
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif

    // Set input queue. The input queue is automatically set in Create().
    // Not thread safe.
    void set_input_queue(CapQueueHandle queue);
    
    // Set output queue. The output queue is automatically set in Create().
    // Not thread safe.
    void set_output_queue(CapQueueHandle queue);
    
    // Set control queue. The control queue is automatically set in Create().
    // Not thread safe.
    void set_command_queue(CmdQueueHandle queue);
    
    // construct a default module
    static ModuleHandle Create(const std::string &name,
                               Face2DDetectorPtr face_detector,
                               CapQueueHandle in_frame_queue,
                               CapQueueHandle out_frame_queue,
                               CmdQueueHandle command_queue);
    
private:
    CapQueueHandle input_frame_queue_;
    CapQueueHandle output_frame_queue_;
    CmdQueueHandle command_queue_;

    PProParams param_;
    
    Face2DDetectorPtr fdetector_;
    
    cv::Rect rect_;
    cv::Mat seg_;

    SegmentationTCPStreamPtr seg_tcp_;
    std::vector<Eigen::Vector3f> p2d_;
};
