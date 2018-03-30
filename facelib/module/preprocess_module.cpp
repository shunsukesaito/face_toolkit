#include "preprocess_module.h"

#include <utility/pts_loader.h>

#include <gflags/gflags.h>
DEFINE_string(land_type, "cpm", "landmark type");
DEFINE_string(seg_save_path, "", "segmentation file path");
DEFINE_string(seg_ip, "csloadbalancer-dev-746798469.us-east-1.elb.amazonaws.com", "IP for hair segmentation net");
DEFINE_uint32(prob_size, 340, "size of probability map");

#ifdef WITH_IMGUI
void PProParams::updateIMGUI()
{
    if (ImGui::Button("OneTime ALL")){
        onetime_land_ = true;
        onetime_seg_ = true;
    }
    if (ImGui::Button("OneTime P2D"))
        onetime_land_ = true;
    if (ImGui::Button("OneTime Seg"))
        onetime_seg_ = true;

    ImGui::Checkbox("update land", &update_land_);
    ImGui::Checkbox("update seg", &update_seg_);
}
#endif

// initializes this module and the basic module
PreprocessModule::PreprocessModule(const std::string &name)
: Module(name)
{
    // nothing to do
}

// default destructor
PreprocessModule::~PreprocessModule()
{
    // nothing to do
}

// stub
void PreprocessModule::Process()
{
    std::string command = "";
        
    while(command != "stop")
    {
        if(input_frame_queue_->front()){
            CaptureResult result;
            result.img = input_frame_queue_->front()->img;
            result.camera = input_frame_queue_->front()->camera;
            result.frame_id = input_frame_queue_->front()->frame_id;
            if(result.img.empty()){
                std::cout << "Warning: Frame drop!" << std::endl;
                input_frame_queue_->pop();
                continue;
            }
            update(result);
            output_frame_queue_->push(result);
            input_frame_queue_->pop();
        }
        if(command_queue_->front()){
            command = *command_queue_->front();
            command_queue_->pop();
        }
    }
}

// the interruption point
void PreprocessModule::Stop()
{
    // nothing to do
}

void PreprocessModule::init(PProParamsPtr param,
                            Face2DDetectorPtr face_detector)
{
    seg_tcp_ = std::make_shared<SegmentationTCPStream>(FLAGS_seg_ip, FLAGS_prob_size);
    param_ = param;
    
    fdetector_ = face_detector;    
}

void PreprocessModule::update(CaptureResult& result)
{
    if(param_->update_land_ || param_->onetime_land_){
        if(FLAGS_land_type.find("cpm") != std::string::npos){
            fdetector_->GetFaceLandmarks(result.img, result.p2d, rect_, false, true);
        }
        else if(FLAGS_land_type.find("dlib") != std::string::npos){
            fdetector_->GetFaceLandmarks(result.img, result.p2d, rect_, false, false);
        }
        else if(FLAGS_land_type.find("pts") != std::string::npos){
            result.p2d = load_pts(FLAGS_land_type);
            rect_ = GetBBoxFromLandmarks(result.p2d);
        }
        p2d_ = result.p2d;
        
        if(param_->onetime_land_) param_->onetime_land_ = false;
    }
    else{
        result.p2d = p2d_;
    }
    
    bool hasface = true;
    if(param_->update_seg_ || param_->onetime_seg_){
        if (!param_->update_land_)
            hasface = fdetector_->GetFaceRect(result.img, rect_, false);
        
        if(hasface){
            seg_tcp_->sendImage(result.img, rect_, 1.8);
            seg_tcp_->getSegmentation(result.seg);
            seg_ = result.seg.clone();
        }
        if(param_->onetime_seg_) param_->onetime_seg_ = false;
    }
}

void PreprocessModule::set_input_queue(CapQueueHandle queue)
{
    input_frame_queue_ = queue;
}

void PreprocessModule::set_output_queue(CapQueueHandle queue)
{
    output_frame_queue_ = queue;
}

void PreprocessModule::set_command_queue(CmdQueueHandle queue)
{
    command_queue_ = queue;
}

ModuleHandle PreprocessModule::Create(const std::string &name,
                                      PProParamsPtr param,
                                      Face2DDetectorPtr face_detector,
                                      CapQueueHandle input_frame_queue,
                                      CapQueueHandle output_frame_queue,
                                      CmdQueueHandle command_queue)
{
    auto module = new PreprocessModule(name);
    // add this module to the global registry
    
    module->init(param,face_detector);
    module->set_input_queue(input_frame_queue);
    module->set_output_queue(output_frame_queue);
    module->set_command_queue(command_queue);
    
    ModuleHandle handle(module);
    ModuleRegistry::RegisterModule(handle);
    return handle;
}