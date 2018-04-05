//
//  exr_loader.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 8/25/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <opencv2/opencv.hpp>
#include <tinyexr.h>

void loadEXRToCV(std::string filename, cv::Mat& mat);

void saveEXRFromCV(std::string filename, const cv::Mat& mat);
