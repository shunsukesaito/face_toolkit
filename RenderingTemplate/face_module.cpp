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

static std::vector<Eigen::Vector3f> getP3DFromP2PC(const Eigen::VectorXf& pts, const std::vector<P2P2DC>& c_p2p)
{
    std::vector<Eigen::Vector3f> out;
    
    for(auto&& c : c_p2p)
    {
        out.push_back(pts.b3(c.v_idx));
    }
    
    return out;
}

static std::vector<Eigen::Vector3f> getP3DFromP2LC(const Eigen::VectorXf& pts, const std::vector<P2L2DC>& c_p2l)
{
    std::vector<Eigen::Vector3f> out;
    
    for(auto&& c : c_p2l)
    {
        out.push_back(pts.b3(c.v_idx));
    }
    
    return out;
}


void FaceModule::init(std::string data_dir)
{
    cameras_.push_back(Camera::parseCameraParams(data_dir + "data/KRT.txt"));

    fModel_.loadBinaryModel(data_dir + "data/PinModel.bin");
    fParam_.init(fModel_);

    P2P2DC::parseConstraints(data_dir + "data/p2p_const.txt", c_p2p_);
    P2L2DC::parseConstraints(data_dir + "data/p2l_const.txt", c_p2l_);
    
    f2f_renderer_.init(data_dir, cameras_[0], fModel_);
    mesh_renderer_.init(data_dir, cameras_[0], fParam_.pts_, fParam_.nml_, fModel_.tri_pts_);
    p3d_renderer_.init(data_dir, cameras_[0], getP3DFromP2PC(fParam_.pts_, c_p2p_));
    p2d_renderer_.init(data_dir);
    
    fdetector_ = std::make_shared<Face2DDetector>(data_dir);
}

void FaceModule::update(cv::Mat& img)
{
    cv::Rect rect;
    fdetector_->GetFaceLandmarks(img, p2d_, rect);
    //DrawLandmarks(img, p2d);
    //cv::rectangle(img, rect, cv::Scalar(255,0,0));
    
    if(enable_p2pfit_)
        P2DGaussNewtonMultiView(fParam_, cameras_, fModel_, c_p2p_, c_p2l_, convPoint(p2d_), p2d_param_);
    if(enable_f2f_){
        std::vector<cv::Mat_<cv::Vec4f>> inputRGBs(cameras_.size());
        cv::Mat tmp;
        cv::cvtColor(img, tmp, CV_BGR2RGBA);
        tmp.convertTo(inputRGBs[0], CV_32F);
        inputRGBs[0] *= 1.f / 255.f;
        F2FHierarchicalGaussNewtonMultiView(fParam_, cameras_, f2f_renderer_, fModel_, inputRGBs, c_p2p_, c_p2l_, convPoint(p2d_), f2f_param_, logger_);
    }
    
    fParam_.updateAll(fModel_);
}

void FaceModule::preview()
{
    if(show_mesh_)
        mesh_renderer_.render(cameras_[0], fParam_.RT, fParam_.pts_, fParam_.nml_);
    if(show_f2f_)
        f2f_renderer_.render(cameras_[0], fParam_);
    if(show_p3d_){
        p3d_renderer_.render(cameras_[0], fParam_.RT, getP3DFromP2PC(fParam_.pts_, c_p2p_));
        p3d_renderer_.render(cameras_[0], fParam_.RT, getP3DFromP2LC(fParam_.pts_, c_p2l_));
    }
    if(show_p2d_)
        p2d_renderer_.render(cameras_[0].width_, cameras_[0].height_, p2d_);
}

#ifdef WITH_IMGUI
void FaceModule::updateIMGUI()
{
    ImGui::Checkbox("p2d fit", &enable_p2pfit_);
    ImGui::Checkbox("f2f fit", &enable_f2f_);
    ImGui::Checkbox("show mesh", &show_mesh_);
    ImGui::Checkbox("show f2f", &show_f2f_);
    ImGui::Checkbox("show p2d", &show_p2d_);
    ImGui::Checkbox("show p3d", &show_p3d_);
    cameras_[0].updateIMGUI();
    fParam_.updateIMGUI();
    f2f_renderer_.updateIMGUI();
    p2d_param_.updateIMGUI();
    f2f_param_.updateIMGUI();
}
#endif
