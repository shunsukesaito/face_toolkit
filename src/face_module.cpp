//
//  face_module.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "face_module.hpp"

static std::vector<Eigen::Vector3f> convPoint(const std::vector<Eigen::Vector2f>& in)
{
    std::vector<Eigen::Vector3f> out(in.size());
    
    for(int i = 0; i < in.size(); ++i)
    {
        out[i] = Eigen::Vector3f(in[i](0),in[i](1),1.0f);
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

#include "renderer.hpp"

// stub
void FaceModule::Process()
{
    std::string command = "";
    
    // OpenGL cannot share context across different threads...
    glfwWindowHint(GLFW_VISIBLE, false);
    glfwWindowHint(GLFW_FOCUSED, false);
    auto window = Window(1, 1, 1, "F2F Window");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
    f2f_renderer_.init(data_dir_, face_model_);
    
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
    data_dir_ = data_dir;
    
    face_model_ = face_model;
    p2d_param_ = p2d_param;
    f2f_param_ = f2f_param;
    
    p2d_param_->dof.ID = std::min(p2d_param_->dof.ID, face_model->n_id());
    p2d_param_->dof.EX = std::min(p2d_param_->dof.EX, face_model->n_exp());
    
    f2f_param_->dof.ID = std::min(f2f_param_->dof.ID, face_model->n_id());
    f2f_param_->dof.EX = std::min(f2f_param_->dof.EX, face_model->n_exp());
    f2f_param_->dof.AL = std::min(f2f_param_->dof.AL, face_model->n_clr());
    
    fd_.setFaceModel(face_model_);

    P2P2DC::parseConstraints(data_dir + "data/p2p_const_bv.txt", c_p2p_);
    P2L2DC::parseConstraints(data_dir + "data/p2l_const_bv.txt", c_p2l_);
    
    fdetector_ = std::make_shared<Face2DDetector>(data_dir);
    CHECK_GL_ERROR();
}

void FaceModule::update(FaceResult& result)
{
    cv::Rect rect;
    
    if(p2d_param_->run_){
        fdetector_->GetFaceLandmarks(result.img, result.p2d, rect);
        // it's not completely thread safe, but copy should be brazingly fast so hopefully it dones't matter
        P2DFitParams opt_param = *p2d_param_;
        if(result.p2d.size() != 0){
            P2DGaussNewton(fd_, result.camera, c_p2p_, c_p2l_, convPoint(result.p2d), opt_param);
            result.processed_ = true;
        }
    }
    if(f2f_param_->run_){
        fdetector_->GetFaceLandmarks(result.img, result.p2d, rect);
        // it's not completely thread safe, but copy should be brazingly fast so hopefully it dones't matter
        F2FParams opt_param = *f2f_param_;
        
        if(result.p2d.size() != 0){
            cv::Mat_<cv::Vec4f> inputRGB;
            cv::Mat tmp;
            cv::cvtColor(result.img, tmp, CV_BGR2RGBA);
            tmp.convertTo(inputRGB, CV_32F);
            inputRGB *= 1.f / 255.f;
            F2FHierarchicalGaussNewton(fd_, result.camera, f2f_renderer_, inputRGB, c_p2p_, c_p2l_, convPoint(result.p2d), opt_param, logger_);
            result.processed_ = true;
        }
    }
    
    result.fd = fd_;
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
