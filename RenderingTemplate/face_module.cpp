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
    
    fdetector_ = std::make_shared<Face2DDetector>(data_dir);
}

void FaceModule::update(cv::Mat& img)
{
    std::vector<Eigen::Vector2f> p2d;
    cv::Rect rect;
    fdetector_->GetFaceLandmarks(img, p2d, rect);
    DrawLandmarks(img, p2d);
    cv::rectangle(img, rect, cv::Scalar(255,0,0));
    
    if(enable_p2pfit_)
        P2DFittingMultiView(fParam_, cameras_, fModel_, c_p2p_, c_p2l_, convPoint(p2d), p2d_param_);
    
    fParam_.updateAll(fModel_);
}

void FaceModule::preview()
{
    mesh_renderer_.render(cameras_[0], fParam_.RT, fParam_.pts_, fParam_.nml_);
    f2f_renderer_.render(cameras_[0], fParam_);
    
    p3d_renderer_.render(cameras_[0], fParam_.RT, getP3DFromP2PC(fParam_.pts_, c_p2p_));
}

#ifdef WITH_IMGUI
void FaceModule::updateIMGUI()
{
    ImGui::Checkbox("p2d fit", &enable_p2pfit_);
    cameras_[0].updateIMGUI();
    fParam_.updateIMGUI();
    f2f_renderer_.updateIMGUI();
    p2d_param_.updateIMGUI();
}
#endif
