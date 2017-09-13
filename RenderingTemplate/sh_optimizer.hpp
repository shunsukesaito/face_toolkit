//
//  sh_optimizer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/11/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef sh_optimizer_hpp
#define sh_optimizer_hpp

#include <opencv2/opencv.hpp>

#include "EigenHelper.h"

void evaluateSHLeastSquare(Eigen::VectorXf& Crgb, Eigen::MatrixXf& CCrgb, const cv::Mat_<cv::Vec4f>& input, const cv::Mat_<cv::Vec4f>& albedo, const cv::Mat_<cv::Vec4f>& normal, unsigned int sampling_rate = 1);

void estimateSH(Eigen::Vector3f SHCoeffs[], const cv::Mat_<cv::Vec4f>& input, const cv::Mat_<cv::Vec4f>& albedo, const cv::Mat_<cv::Vec4f>& normal, int SH);

#endif /* sh_optimizer_hpp */
