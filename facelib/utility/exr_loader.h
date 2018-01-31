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

inline void loadEXRToCV(std::string filename, cv::Mat& mat)
{
    TinyExrImage tmp;
    const char* err;
    int ret = LoadEXR(&tmp.buf, &tmp.width, &tmp.height, filename.c_str(), &err);
    if (ret != 0)
    {
        std::cout << "Error: exr file isn't loaded correctly... " << filename << " " << err << std::endl;
        throw std::runtime_error("Error: diffEnv file isn't loaded correctly...");
    }
    mat = cv::Mat(tmp.width,tmp.height,CV_32FC4,(float*)tmp.buf).clone();
    cv::flip(mat, mat, 0);
    cv::cvtColor(mat,mat,CV_RGBA2BGRA);
}

inline void saveEXRFromCV(std::string filename, const cv::Mat& mat)
{
    cv::Mat tmp = mat.clone();
    cv::flip(tmp,tmp,-1);
    if(mat.channels() == 4)
        cv::cvtColor(tmp,tmp,CV_BGRA2RGBA);
    else if(mat.channels() == 3)
        cv::cvtColor(tmp,tmp,CV_BGR2RGB);
    int ret = SaveEXR((float*)tmp.ptr(), tmp.cols, tmp.rows, mat.channels(), 0, filename.c_str());
    if (ret != 0)
    {
        std::cout << "Error: exr file isn't saved correctly... " << filename << std::endl;
        throw std::runtime_error("Error: exr file isn't saved correctly...");
    }
}
