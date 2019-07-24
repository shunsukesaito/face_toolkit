//
//  face_module.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#include "face_module.h"
#include "face_renderer.h"

#include <face2d-detector/face2d_detector.h>

// initializes this module and the basic module
FaceOptModule::FaceOptModule(const std::string &name)
: Module(name)
{
    // nothing to do
}

// default destructor
FaceOptModule::~FaceOptModule()
{
    // nothing to do
}

// stub
void FaceOptModule::Process()
{
    std::string command = "";
    
    // optimizers need to be initialized here for thread-depedent functions
    for(auto&& opt : optimizers_)
        opt->init(data_dir_, fm_);

    while(command != "stop")
    {
        if(input_frame_queue_->front()){
            FaceResult result;
            // TODO: make it general so that we can take multi-view/multi-frame input
            result.cameras[0] = input_frame_queue_->front()->camera;
            result.cap_data[0][0] = input_frame_queue_->front()->data;

            if(result.cap_data[0][0].img_.empty()){
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
void FaceOptModule::Stop()
{
    // nothing to do
}

void FaceOptModule::init(std::string data_dir,
                         FaceModelPtr fm,
                         std::vector<OptimizerHandle> optimizers)
{
    data_dir_ = data_dir;
    
    fm_ = fm;
    
    optimizers_ = optimizers;
    
    fd_[0].setFaceModel(fm_);

    P2P2DC::parseConstraints(data_dir + "opt/p2p_const_" + fm->fm_type_ + ".txt", c_p2p_);
    P2L2DC::parseConstraints(data_dir + "opt/p2l_const_" + fm->fm_type_ + ".txt", c_p2l_);
    
    fm_->loadContourList(data_dir + "opt/cont_list_" + fm->fm_type_ + ".txt");
}

void FaceOptModule::update(FaceResult& result)
{
    for(int i = 0; i < fd_.size(); ++i)
    {
        if(std::isnan(fd_[i].idCoeff.sum()) ||
           std::isnan(fd_[i].exCoeff.sum()) ||
           std::isnan(fd_[i].alCoeff.sum()) ||
           std::isnan(fd_[i].RT().sum()) ||
           std::isnan(fd_[i].SH().sum())){
            std::cerr << "FaceOptModule::update() - face gets nan..." << std::endl;
            fd_[i].init();
        }
        
        // update contour
        // TODO: support multu-view contour update
        fd_[i].updateContour(result.cameras[0].intrinsic_, result.cameras[0].extrinsic_);
        for(int j = 0; j < fd_[i].cont_idx_.size(); ++j)
        {
            c_p2l_[j].v_idx = fd_[i].cont_idx_[j];
        }
    }
    result.fd = fd_;
    result.c_p2p = c_p2p_;
    result.c_p2l = c_p2l_;    
    
    for(auto&& opt : optimizers_)
        opt->solve(result);
    
    fd_ = result.fd;
}

void FaceOptModule::set_input_queue(CapQueueHandle queue)
{
    input_frame_queue_ = queue;
}

void FaceOptModule::set_output_queue(FaceQueueHandle queue)
{
    output_result_queue_ = queue;
}

void FaceOptModule::set_command_queue(CmdQueueHandle queue)
{
    command_queue_ = queue;
}

#ifdef WITH_IMGUI
void FaceOptModule::updateIMGUI()
{
    if (ImGui::CollapsingHeader("Optimization Module"))
    {
        for(auto&& opt : optimizers_)
            opt->updateIMGUI();
    }
}
#endif

ModuleHandle FaceOptModule::Create(const std::string &name,
                                   const std::string &data_dir,
                                   FaceModelPtr fm,
                                   std::vector<OptimizerHandle> optimizers,
                                   CapQueueHandle input_frame_queue,
                                   FaceQueueHandle output_result_queue,
                                   CmdQueueHandle command_queue)
{
    auto module = new FaceOptModule(name);
    // add this module to the global registry
    
    module->init(data_dir,fm,optimizers);
    module->set_input_queue(input_frame_queue);
    module->set_output_queue(output_result_queue);
    module->set_command_queue(command_queue);
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}

// initializes this module and the basic module
FacePreviewModule::FacePreviewModule(const std::string &name)
: Module(name)
{
    // nothing to do
}

// default destructor
FacePreviewModule::~FacePreviewModule()
{
    // nothing to do
}

// stub
void FacePreviewModule::Process()
{
    std::string command = "";
    
    while(command != "stop")
    {
        if(input_frame_queue_->front()){
            FaceResult result;
            // TODO: make it general so that we can take multi-view/multi-frame input
            result.cameras[0] = input_frame_queue_->front()->camera;
            result.cap_data[0][0] = input_frame_queue_->front()->data;

            if(result.cap_data[0][0].img_.empty()){
                std::cout << "Warning: Frame drop!" << std::endl;
                input_frame_queue_->pop();
                continue;
            }
            update(result);
            output_result_queue_->push(result);
            input_frame_queue_->pop();
            usleep(30000);
        }
        if(command_queue_->front()){
            command = *command_queue_->front();
            command_queue_->pop();
        }
    }
}

// the interruption point
void FacePreviewModule::Stop()
{
    // nothing to do
}

void FacePreviewModule::init(std::string data_dir,
                             FaceModelPtr face_model,
                             const std::vector<std::string>& flist)
{
    data_dir_ = data_dir;
    
    flist_ = flist;
    face_model_ = face_model;
    fd_[0].setFaceModel(face_model_);
}

void FacePreviewModule::init(std::string data_dir,
                             FaceModelPtr face_model,
                             const std::string& ip,
                             int port,
                             const std::vector<std::pair<std::string, int>>& dof,
                             int input_img_size,
                             bool sendImage)
{
    data_dir_ = data_dir;
    
    flist_ = std::vector<std::string>();
    face_model_ = face_model;
    fd_[0].setFaceModel(face_model_);
    
    send_image_ = sendImage;
    
    int total_dof = 0;
    for(auto iter = dof.begin(); iter != dof.end(); ++iter)
    {
        total_dof += iter->second;
    }
    param_tcp_ = std::make_shared<ParamTCPStream>(ip,port,total_dof,input_img_size,dof);
}

void FacePreviewModule::update(FaceResult& result)
{
    result.fd = fd_;
    // TODO: make it general so that we can take multi-view/multi-frame input
    const int idx = result.cap_data[0][0].frame_id_;
    if(flist_.size() != 0 && idx >= 0 && dof_.empty()){
        result.loadFromTXT(flist_[idx%(int)flist_.size()]);
        result.processed_ = true;
    }
    if(flist_.size() != 0 && idx >= 0 && !dof_.empty()){
        std::ifstream infile(flist_[idx%(int)flist_.size()]);
        if(!infile.is_open()){
            std::cout << "Warning: failed parsing face data from " << flist_[idx%(int)flist_.size()] << std::endl;
            return;
        }
        std::string line;
        std::vector<float> tmp;
        // identity
        std::getline(infile, line);
        tmp = string2arrayf(line);
        result.loadFromVec(dof_, tmp);
        result.processed_ = true;
    }
    if(param_tcp_ != NULL){
        auto ret = param_tcp_->getParams();
        result.loadFromVec(ret->dof, ret->param);
        result.processed_ = true;
    }
    result.c_p2p = c_p2p_;
    result.c_p2l = c_p2l_;
}

void FacePreviewModule::set_input_queue(CapQueueHandle queue)
{
    input_frame_queue_ = queue;
}

void FacePreviewModule::set_output_queue(FaceQueueHandle queue)
{
    output_result_queue_ = queue;
}

void FacePreviewModule::set_command_queue(CmdQueueHandle queue)
{
    command_queue_ = queue;
}

ModuleHandle FacePreviewModule::Create(const std::string &name,
                                       const std::string &data_dir,
                                       FaceModelPtr face_model,
                                       CapQueueHandle input_frame_queue,
                                       FaceQueueHandle output_result_queue,
                                       CmdQueueHandle command_queue,
                                       const std::string &file_fmt,
                                       int begin_frame,
                                       int end_frame)
{
    auto module = new FacePreviewModule(name);
    // add this module to the global registry
    
    if(file_fmt.empty()){
        module->init(data_dir,face_model);
    }
    else{
        char tmp[256];
        std::vector<std::string> file_list;
        for(int i = begin_frame; i <= end_frame; ++i)
        {
            sprintf(tmp, file_fmt.c_str(), i);
            std::ifstream dummy(tmp);
            if (dummy.good())
                file_list.push_back(tmp);
        }
        if(file_list.size() == 0){
            std::cout << "Error: face parameters does not exist. " << file_fmt << std::endl;
            throw std::runtime_error("Error: face parameters does not exist. ");
        }
        module->init(data_dir,face_model,file_list);
    }
    
    module->set_input_queue(input_frame_queue);
    module->set_output_queue(output_result_queue);
    module->set_command_queue(command_queue);
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}

ModuleHandle FacePreviewModule::Create(const std::string &name,
                                       const std::string &data_dir,
                                       FaceModelPtr face_model,
                                       CapQueueHandle input_frame_queue,
                                       FaceQueueHandle output_result_queue,
                                       CmdQueueHandle command_queue,
                                       const std::vector<std::pair<std::string, int>>& dof,
                                       const std::string &file_fmt,
                                       int begin_frame,
                                       int end_frame)
{
    auto module = new FacePreviewModule(name);
    // add this module to the global registry
    
    if(file_fmt.empty()){
        module->init(data_dir,face_model);
    }
    else{
        char tmp[256];
        std::vector<std::string> file_list;
        for(int i = begin_frame; i <= end_frame; ++i)
        {
            sprintf(tmp, file_fmt.c_str(), i);
            std::ifstream dummy(tmp);
            if (dummy.good())
                file_list.push_back(tmp);
        }
        if(file_list.size() == 0){
            std::cout << "Error: face parameters does not exist. " << file_fmt << std::endl;
            throw std::runtime_error("Error: face parameters does not exist. ");
        }
        module->init(data_dir,face_model,file_list);
        module->dof_ = dof;
    }
    
    module->set_input_queue(input_frame_queue);
    module->set_output_queue(output_result_queue);
    module->set_command_queue(command_queue);
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}

ModuleHandle FacePreviewModule::Create(const std::string &name,
                                       const std::string &data_dir,
                                       FaceModelPtr face_model,
                                       CapQueueHandle input_frame_queue,
                                       FaceQueueHandle output_result_queue,
                                       CmdQueueHandle command_queue,
                                       const std::string &root_dir,
                                       const std::string &list_file)
{
    auto module = new FacePreviewModule(name);
    // add this module to the global registry
    
    std::vector<std::string> file_list;
    std::ifstream fin(list_file);
    if(!fin.is_open()){
        std::cout << "Warning: failed parsing image sequence from " << list_file << std::endl;
        throw std::runtime_error("Error: face parameters does not exist. ");
    }
    
    std::string f;
    while(std::getline(fin, f))
    {
        if (f.empty())
            continue;
        std::ifstream dummy(root_dir + "/" + f.substr(0,f.find_last_of("/")) + "/params.txt");
    
            if (dummy.good())
                file_list.push_back(root_dir + "/" + f.substr(0,f.find_last_of("/")) + "/params.txt");
//        std::ifstream dummy(root_dir + "/" + f.substr(0,f.size()-4) + "_params.txt");
//
//        if (dummy.good())
//            file_list.push_back(root_dir + "/" + f.substr(0,f.size()-4) + "_params.txt");
    }
    
    if(file_list.size() == 0){
        std::cout << "Error: face parameters does not exist. " << root_dir + "/" + f.substr(0,f.find_last_of("/")) + "/params.txt" << std::endl;
        throw std::runtime_error("Error: face parameters does not exist. ");
    }

    module->init(data_dir,face_model,file_list);
    
    module->set_input_queue(input_frame_queue);
    module->set_output_queue(output_result_queue);
    module->set_command_queue(command_queue);
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}

ModuleHandle FacePreviewModule::Create(const std::string &name,
                                       const std::string &data_dir,
                                       FaceModelPtr face_model,
                                       CapQueueHandle input_frame_queue,
                                       FaceQueueHandle output_result_queue,
                                       CmdQueueHandle command_queue,
                                       const std::string& ip, int port,
                                       const std::vector<std::pair<std::string, int>>& dof,
                                       int input_img_size,
                                       bool sendImage)
{
    auto module = new FacePreviewModule(name);
    // add this module to the global registry
    
    module->init(data_dir, face_model, ip, port, dof, input_img_size, sendImage);
    
    module->set_input_queue(input_frame_queue);
    module->set_output_queue(output_result_queue);
    module->set_command_queue(command_queue);
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}
