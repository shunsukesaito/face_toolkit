//
//  face_module.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <gl_utility/camera.h>
#include <shape_model/face_model.h>

#include <f2f/f2f_renderer.h>

#include <optimizer/p2d_optimizer.h>
#include <f2f/f2f_optimizer.h>
#include <optimizer/face_result.h>

#include "module.h"
#include "capture_module.h"

typedef std::shared_ptr<SPSCQueue<FaceResult>> FaceQueueHandle;

class FaceOptModule : public Module
{
public:
    // initializes a module
    FaceOptModule(const std::string &name);
    // destructor
    ~FaceOptModule();
    
    void init(std::string data_dir,
              FaceModelPtr face_model,
              P2DFitParamsPtr p2d_param,
              F2FParamsPtr f2f_param);
    
    void update(FaceResult& result);
    
    // does module specific work
    virtual void Process();
    
    // Provides a way to interrupt the process.
    // Default implementation does nothing.
    virtual void Stop();
    
    inline void reset(){ fd_.init(); }
    
    // Set input queue. The input queue is automatically set in Create().
    // Not thread safe.
    void set_input_queue(CapQueueHandle queue);
    
    // Set output queue. The output queue is automatically set in Create().
    // Not thread safe.
    void set_output_queue(FaceQueueHandle queue);
    
    // Set control queue. The control queue is automatically set in Create().
    // Not thread safe.
    void set_command_queue(CmdQueueHandle queue);

    
    // construct a default module
    static ModuleHandle Create(const std::string &name,
                               const std::string &data_dir,
                               FaceModelPtr face_model,
                               P2DFitParamsPtr p2d_param,
                               F2FParamsPtr f2f_param,
                               CapQueueHandle input_frame_queue,
                               FaceQueueHandle output_result_queue,
                               CmdQueueHandle command_queue);
private:
    std::string data_dir_;
    
    CapQueueHandle input_frame_queue_;
    FaceQueueHandle output_result_queue_;
    CmdQueueHandle command_queue_;
    
    F2FRenderer f2f_renderer_;
    
    FaceModelPtr face_model_;
    FaceData fd_;
        
    std::vector<P2P2DC> c_p2p_;
    std::vector<P2L2DC> c_p2l_;
    
    std::shared_ptr<spdlog::logger> logger_ = spdlog::stdout_color_mt("console");
    
    P2DFitParamsPtr p2d_param_;
    F2FParamsPtr f2f_param_;    
};

class FacePreviewModule : public Module
{
public:
    // initializes a module
    FacePreviewModule(const std::string &name);
    // destructor
    ~FacePreviewModule();
    
    void init(std::string data_dir,
              FaceModelPtr face_model,
              const std::vector<std::string>& flist = std::vector<std::string>());
    
    void update(FaceResult& result);
    
    // does module specific work
    virtual void Process();
    
    // Provides a way to interrupt the process.
    // Default implementation does nothing.
    virtual void Stop();
    
    inline void reset(){ fd_.init(); }
    
    // Set input queue. The input queue is automatically set in Create().
    // Not thread safe.
    void set_input_queue(CapQueueHandle queue);
    
    // Set output queue. The output queue is automatically set in Create().
    // Not thread safe.
    void set_output_queue(FaceQueueHandle queue);
    
    // Set control queue. The control queue is automatically set in Create().
    // Not thread safe.
    void set_command_queue(CmdQueueHandle queue);
    
    
    // construct a default module
    static ModuleHandle Create(const std::string &name,
                               const std::string &data_dir,
                               FaceModelPtr face_model,
                               CapQueueHandle input_frame_queue,
                               FaceQueueHandle output_result_queue,
                               CmdQueueHandle command_queue,
                               const std::string &file_fmt = "",
                               int begin_frame = 0,
                               int end_frame = 0);
    
    // construct a default module
    static ModuleHandle Create(const std::string &name,
                               const std::string &data_dir,
                               FaceModelPtr face_model,
                               CapQueueHandle input_frame_queue,
                               FaceQueueHandle output_result_queue,
                               CmdQueueHandle command_queue,
                               const std::string &root_dir,
                               const std::string &list_file);
    
private:
    std::string data_dir_;
    
    CapQueueHandle input_frame_queue_;
    FaceQueueHandle output_result_queue_;
    CmdQueueHandle command_queue_;
    
    FaceModelPtr face_model_;
    FaceData fd_;
    
    std::vector<std::string> flist_;
    
    std::vector<P2P2DC> c_p2p_;
    std::vector<P2L2DC> c_p2l_;
    
    std::shared_ptr<spdlog::logger> logger_ = spdlog::stdout_color_mt("console");
};


