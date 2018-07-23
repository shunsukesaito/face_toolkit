//
//  test_regressor.hpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 10/5/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//
#pragma once

#include <opencv2/flann/flann.hpp>

#include <gl_utility/camera.h>

#include "regressor.h"

namespace cao{

struct Model
{
    int T_ = 10;
    int k_ = 8; // for k-NN
    
    std::vector<Regressor> regressors_;
    
    cv::Mat_<float> gt_x_;
    
    // test
    int l_eye_id_, r_eye_id_;
    cv::Mat_<int> knn_;
    cv::flann::KDTreeIndexParams indexParams_;
    cv::flann::Index kdTree_;
    cv::Mat_<float> aligned_p2d_;
        
    void loadCVModel(std::string file_path);
    void loadBinary(std::string file_path);
    void writeCVModel(std::string file_path);
    void writeCVModel(std::string file_path, const TrainParams& params);
    void writeBinary(std::string file_path);
    
    bool test(cv::Mat_<float>& X,
              Data& data,
              const cv::Mat_<uchar>& img,
              const cv::Mat_<cv::Vec2f>& p2d,
              const cv::Mat_<int>& tri,
              const DOF& dof);
    
    bool test(cv::Mat_<float>& X,
              Data& data,
              const cv::Mat_<uchar>& img,
              const cv::Mat_<cv::Vec2f>& p2d,
              const cv::Mat_<bool> pmap,
              const cv::Rect &rect,
              const cv::Mat_<int>& tri,
              const DOF& dof);
    
    bool train(std::string file_path,
               std::vector<Data>& data,
               const TrainParams& params);
};

}
