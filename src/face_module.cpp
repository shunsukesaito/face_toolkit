//
//  face_module.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "face_module.hpp"

static std::vector<std::vector<Eigen::Vector3f>> convPoint(const std::vector<Eigen::Vector2f>& in)
{
    std::vector<std::vector<Eigen::Vector3f>> out(1);
    out[0].resize(in.size());
    
    for(int i = 0; i < in.size(); ++i)
    {
        out[0][i] = Eigen::Vector3f(in[i](0),in[i](1),1.0f);
    }
    
    return out;
}

std::vector<Eigen::Vector3f> getP3DFromP2PC(const Eigen::VectorXf& pts, const std::vector<P2P2DC>& c_p2p)
{
    std::vector<Eigen::Vector3f> out;
    
    for(auto&& c : c_p2p)
    {
        out.push_back(pts.b3(c.v_idx));
    }
    
    return out;
}

std::vector<Eigen::Vector3f> getP3DFromP2LC(const Eigen::VectorXf& pts, const std::vector<P2L2DC>& c_p2l)
{
    std::vector<Eigen::Vector3f> out;
    
    for(auto&& c : c_p2l)
    {
        out.push_back(pts.b3(c.v_idx));
    }
    
    return out;
}

// initializes this module and the basic module
FaceModule::FaceModule(const std::string &name)
: Module(name)
{
    // nothing to do
}

// default destructor
FaceModule::~FaceModule()
{
    // nothing to do
}

// stub
void FaceModule::Process()
{
    std::string command = "";
    while(command != "stop")
    {
        if(input_frame_queue_->front()){
            FaceResult result;
            result.img = input_frame_queue_->front()->img;
            result.camera = input_frame_queue_->front()->camera;
            if(result.img.empty()){
                std::cout << "Warning: Frame drop!" << std::endl;
                input_frame_queue_->pop();
                continue;
            }
            update(result);
            output_result_queue_->push(result);
            input_frame_queue_->pop();
        }
        
        if(command_queue_->front()){
            command = *command_queue_->front();
            command_queue_->pop();
        }
    }
}

// the interruption point
void FaceModule::Stop()
{
    // nothing to do
}

void FaceModule::init(std::string data_dir,
                      FaceModelPtr face_model,
                      P2DFitParamsPtr p2d_param,
                      F2FParamsPtr f2f_param)
{
    face_model_ = face_model;
    p2d_param_ = p2d_param;
    f2f_param_ = f2f_param;

    fParam_.init(*face_model_);

    P2P2DC::parseConstraints(data_dir + "data/p2p_const.txt", c_p2p_);
    P2L2DC::parseConstraints(data_dir + "data/p2l_const.txt", c_p2l_);
    
    f2f_renderer_.init(data_dir, *face_model_);
    
    fdetector_ = std::make_shared<Face2DDetector>(data_dir);
}

void FaceModule::update(FaceResult& result)
{
    cv::Rect rect;
    
    if(p2d_param_->run_){
        fdetector_->GetFaceLandmarks(result.img, result.p2d, rect);
        std::vector<Camera> cameras(1, result.camera);
        // it's not completely thread safe, but copy should be brazingly fast so hopefully it dones't matter
        P2DFitParams opt_param = *p2d_param_;
        P2DGaussNewtonMultiView(fParam_, cameras, *face_model_, c_p2p_, c_p2l_, convPoint(result.p2d), opt_param);
        
        result.processed_ = true;
    }

    if(f2f_param_->run_){
        fdetector_->GetFaceLandmarks(result.img, result.p2d, rect);
        std::vector<Camera> cameras(1, result.camera);
        // it's not completely thread safe, but copy should be brazingly fast so hopefully it dones't matter
        F2FParams opt_param = *f2f_param_;
        
        std::vector<cv::Mat_<cv::Vec4f>> inputRGBs(1);
        cv::Mat tmp;
        cv::cvtColor(result.img, tmp, CV_BGR2RGBA);
        tmp.convertTo(inputRGBs[0], CV_32F);
        inputRGBs[0] *= 1.f / 255.f;
        F2FHierarchicalGaussNewtonMultiView(fParam_, cameras, f2f_renderer_, *face_model_, inputRGBs, c_p2p_, c_p2l_, convPoint(result.p2d), opt_param, logger_);
        
        result.processed_ = true;
    }
    
    result.fParam = fParam_;
    result.c_p2p = c_p2p_;
    result.c_p2l = c_p2l_;
}

void FaceModule::set_input_queue(CapQueueHandle queue)
{
    input_frame_queue_ = queue;
}

void FaceModule::set_output_queue(FaceQueueHandle queue)
{
    output_result_queue_ = queue;
}

void FaceModule::set_command_queue(CmdQueueHandle queue)
{
    command_queue_ = queue;
}

ModuleHandle FaceModule::Create(const std::string &name,
                                const std::string &data_dir,
                                FaceModelPtr face_model,
                                P2DFitParamsPtr p2d_param,
                                F2FParamsPtr f2f_param,
                                CapQueueHandle input_frame_queue,
                                FaceQueueHandle output_result_queue,
                                CmdQueueHandle command_queue)
{
    auto module = new FaceModule(name);
    // add this module to the global registry
    
    module->init(data_dir,face_model,p2d_param,f2f_param);
    module->set_input_queue(input_frame_queue);
    module->set_output_queue(output_result_queue);
    module->set_command_queue(command_queue);
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}
