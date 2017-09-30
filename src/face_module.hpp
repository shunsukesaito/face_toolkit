//
//  face_module.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef face_module_hpp
#define face_module_hpp

#include "camera.hpp"
#include "face_model.hpp"

#include "face_2dtracker.h"

#include "f2f_renderer.hpp"
#include "mesh_renderer.hpp"
#include "p3d_renderer.hpp"
#include "p2d_renderer.hpp"

#include "p2d_optimizer.h"
#include "f2f_optimizer.h"

#include "module.hpp"
#include "capture_module.hpp"

struct FaceResult
{
    bool processed_ = false;
    
    // assume single camera for now
    cv::Mat img;
    Camera camera;
    
    FaceParams fParam;
    
    std::vector<P2P2DC> c_p2p;
    std::vector<P2L2DC> c_p2l;
    
    std::vector<Eigen::Vector2f> p2d;
};

std::vector<Eigen::Vector3f> getP3DFromP2PC(const Eigen::VectorXf& pts, const std::vector<P2P2DC>& c_p2p);
std::vector<Eigen::Vector3f> getP3DFromP2LC(const Eigen::VectorXf& pts, const std::vector<P2L2DC>& c_p2l);

typedef std::shared_ptr<SPSCQueue<FaceResult>> FaceQueueHandle;

class FaceModule : public Module
{
public:
    // initializes a module
    FaceModule(const std::string &name);
    // destructor
    ~FaceModule();
    
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
    
    inline void reset(){ fParam_.init(*face_model_); }
    
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
    CapQueueHandle input_frame_queue_;
    FaceQueueHandle output_result_queue_;
    CmdQueueHandle command_queue_;
    
    Face2DDetectorPtr fdetector_;
    
    F2FRenderer f2f_renderer_;
    
    FaceModelPtr face_model_;
    FaceParams fParam_;

    std::vector<Eigen::Vector2f> p2d_;
    
    std::vector<P2P2DC> c_p2p_;
    std::vector<P2L2DC> c_p2l_;
    
    std::shared_ptr<spdlog::logger> logger_ = spdlog::stdout_color_mt("console");
    
    P2DFitParamsPtr p2d_param_;
    F2FParamsPtr f2f_param_;    
};

#endif /* face_module_hpp */
