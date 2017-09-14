//
//  face_module.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "face_module.hpp"

void FaceModule::init(std::string data_dir)
{
    camera_ = Camera::parseCameraParams(data_dir + "data/KRT.txt");

    facemodel_.loadBinaryModel(data_dir + "data/PinModel.bin");
    fParam_.init(facemodel_);

    f2f_renderer_.init(data_dir, camera_, facemodel_);
    mesh_renderer_.init(data_dir, camera_, fParam_.pts_, fParam_.nml_, facemodel_.tri_pts_);
    
    fdetector_ = std::make_shared<Face2DDetector>(data_dir);
}

void FaceModule::update(cv::Mat& img)
{
    std::vector<Eigen::Vector2f> p2d;
    cv::Rect rect;
    fdetector_->GetFaceLandmarks(img, p2d, rect);
    DrawLandmarks(img, p2d);
    cv::rectangle(img, rect, cv::Scalar(255,0,0));

    fParam_.updateAll(facemodel_);
}

void FaceModule::preview()
{
    mesh_renderer_.render(camera_, fParam_.RT, fParam_.pts_, fParam_.nml_);
    f2f_renderer_.render(camera_, fParam_);
}

#ifdef WITH_IMGUI
void FaceModule::updateIMGUI()
{
    camera_.updateIMGUI();
    fParam_.updateIMGUI();
    f2f_renderer_.updateIMGUI();
    p2d_param_.updateIMGUI();
}
#endif
