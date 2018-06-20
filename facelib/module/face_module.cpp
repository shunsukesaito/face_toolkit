//
//  face_module.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#include "face_module.h"
#include "renderer.h"

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
    
    // OpenGL cannot share context across different threads...
    glfwWindowHint(GLFW_VISIBLE, false);
    glfwWindowHint(GLFW_FOCUSED, false);
    auto window = Window(1, 1, 1, "F2F Window");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
    f2f_renderer_.init(data_dir_, data_dir_ + "shaders", face_model_);
    
    while(command != "stop")
    {
        if(input_frame_queue_->front()){
            FaceResult result;
            result.img = input_frame_queue_->front()->img;
            result.camera = input_frame_queue_->front()->camera;
            result.p2d = input_frame_queue_->front()->p2d;
            result.seg = input_frame_queue_->front()->seg;
            result.name = input_frame_queue_->front()->name;
            result.frame_id = input_frame_queue_->front()->frame_id;

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
void FaceOptModule::Stop()
{
    // nothing to do
}

void FaceOptModule::init(std::string data_dir,
                      FaceModelPtr face_model,
                      P2DFitParamsPtr p2d_param,
                      F2FParamsPtr f2f_param)
{
    data_dir_ = data_dir;
    
    face_model_ = face_model;
    p2d_param_ = p2d_param;
    f2f_param_ = f2f_param;
    
    p2d_param_->loadParamFromTxt("p2d.ini");
    f2f_param_->loadParamFromTxt("f2f.ini");
    
    p2d_param_->dof.ID = std::min(p2d_param_->dof.ID, face_model->n_id());
    p2d_param_->dof.EX = std::min(p2d_param_->dof.EX, face_model->n_exp());
    
    f2f_param_->dof.ID = std::min(f2f_param_->dof.ID, face_model->n_id());
    f2f_param_->dof.EX = std::min(f2f_param_->dof.EX, face_model->n_exp());
    f2f_param_->dof.AL = std::min(f2f_param_->dof.AL, face_model->n_clr());
    
    fd_.setFaceModel(face_model_);

    P2P2DC::parseConstraints(data_dir + "p2p_const_" + face_model->fm_type_ + ".txt", c_p2p_);
    P2L2DC::parseConstraints(data_dir + "p2l_const_" + face_model->fm_type_ + ".txt", c_p2l_);
    
    face_model_->loadContourList(data_dir + "cont_list_" + face_model->fm_type_ + ".txt");
    CHECK_GL_ERROR();
}

void FaceOptModule::update(FaceResult& result)
{
    if(p2d_param_->run_ || p2d_param_->onetime_run_){
        if(std::isnan(fd_.idCoeff.sum()) ||
           std::isnan(fd_.exCoeff.sum()) ||
           std::isnan(fd_.alCoeff.sum()) ||
           std::isnan(fd_.RT.sum()) ||
           std::isnan(fd_.SH.sum())){
            std::cout << "Warning: face gets nan..." << std::endl;
            fd_.init();
        }
        // update contour
        fd_.updateContour(result.camera.intrinsic_, result.camera.extrinsic_);
        for(int i = 0; i < fd_.cont_idx_.size(); ++i)
        {
            c_p2l_[i].v_idx = fd_.cont_idx_[i];
        }
        // it's not completely thread safe, but copy should be brazingly fast so hopefully it dones't matter
        P2DFitParams opt_param = *p2d_param_;
        if(result.p2d.size() != 0){
            P2DGaussNewton(fd_, result.camera, c_p2p_, c_p2l_, result.p2d, opt_param);
            result.processed_ = true;
        }
        if(p2d_param_->onetime_run_) p2d_param_->onetime_run_ = false;
    }
    if(f2f_param_->run_ || f2f_param_->onetime_run_){
        // update contour
        fd_.updateContour(result.camera.intrinsic_, result.camera.extrinsic_);
        for(int i = 0; i < fd_.cont_idx_.size(); ++i)
        {
            c_p2l_[i].v_idx = fd_.cont_idx_[i];
        }
        
        // it's not completely thread safe, but copy should be brazingly fast so hopefully it dones't matter
        F2FParams opt_param = *f2f_param_;

        // update segmentation mask
        if(!result.seg.empty())
            f2f_renderer_.updateSegment(result.seg);
        
        if(result.p2d.size() != 0){
            cv::Mat_<cv::Vec4f> inputRGB;
            cv::Mat tmp;
            cv::cvtColor(result.img, tmp, CV_BGR2RGBA);
            tmp.convertTo(inputRGB, CV_32F);
            inputRGB *= 1.f / 255.f;
            F2FHierarchicalGaussNewton(fd_, result.camera, f2f_renderer_, inputRGB, c_p2p_, c_p2l_, result.p2d, opt_param, logger_);
            result.processed_ = true;
        }
        if(f2f_param_->onetime_run_) f2f_param_->onetime_run_ = false;
    }
    {
        cv::Mat tmp = result.img.clone();
        DrawLandmarks(tmp, result.p2d);
        cv::imwrite("land.png", tmp);
    }

    
//    std::ifstream infile("/Users/shunsuke/Documents/contour_index.txt");
//    std::string line;
//    for(int i = 0; i < 17; ++i)
//    {
//        std::ofstream ofile("/Users/shunsuke/Documents/cont" + std::to_string(i) + ".txt");
//        std::vector<int> tmp;
//        // identity
//        std::getline(infile, line);
//        std::istringstream iss(line);
//        int tmp_i;
//        while (iss >> tmp_i)
//        {
//            ofile << face_model_->uvs_(tmp_i,0) << " " << 1.0-face_model_->uvs_(tmp_i,1) << std::endl;
//        }
//        ofile.close();
//    }
//    exit(0);
    
//    for(auto&& c : c_p2p_)
//    {
//        std::cout << face_model_->uvs_(c.v_idx,0) << " " << 1.0-face_model_->uvs_(c.v_idx,1) << std::endl;
//    }
//    std::cout << std::endl;
    
    result.fd = fd_;
    result.c_p2p = c_p2p_;
    result.c_p2l = c_p2l_;
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

ModuleHandle FaceOptModule::Create(const std::string &name,
                                const std::string &data_dir,
                                FaceModelPtr face_model,
                                P2DFitParamsPtr p2d_param,
                                F2FParamsPtr f2f_param,
                                CapQueueHandle input_frame_queue,
                                FaceQueueHandle output_result_queue,
                                CmdQueueHandle command_queue)
{
    auto module = new FaceOptModule(name);
    // add this module to the global registry
    
    module->init(data_dir,face_model,p2d_param,f2f_param);
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
    
    // OpenGL cannot share context across different threads...
    glfwWindowHint(GLFW_VISIBLE, false);
    glfwWindowHint(GLFW_FOCUSED, false);
    auto window = Window(1, 1, 1, "F2F Window");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
    while(command != "stop")
    {
        if(input_frame_queue_->front()){
            FaceResult result;
            result.img = input_frame_queue_->front()->img;
            result.camera = input_frame_queue_->front()->camera;
            result.frame_id = input_frame_queue_->front()->frame_id;
            result.p2d = input_frame_queue_->front()->p2d;
            result.seg = input_frame_queue_->front()->seg;
            result.name = input_frame_queue_->front()->name;
            
            if(result.img.empty()){
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
    fd_.setFaceModel(face_model_);
}

void FacePreviewModule::update(FaceResult& result)
{
    result.fd = fd_;
    if(flist_.size() != 0 && result.frame_id >= 0){
        result.loadFromTXT(flist_[result.frame_id%(int)flist_.size()]);
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
