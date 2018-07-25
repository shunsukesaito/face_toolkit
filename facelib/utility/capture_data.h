//
//  frame_loader.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 12/4/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <iostream>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

struct BaseCaptureData
{
    int type = 0; // 0: single frame, 1: multi-frame, 2: multi-view, 3: multi-view + multi-frame
};

// single frame capture data
struct CaptureData
{
    std::vector<Eigen::Vector3f> q2V_;
    std::vector<Eigen::Vector4f> q3V_;
    cv::Mat img_;
    cv::Mat seg_;
    
    int frame_id_ = 0;
    std::string name_ = "";
};

// multi-frame capture data
struct MVCaptureData
{
    std::vector<CaptureData> val_;
    MVCaptureData(){
        val_.resize(1);
    }
    MVCaptureData(int i){
        val_.resize(i);
    }
    
    inline const CaptureData& operator[] (size_t i) const
    {
        return val_[i];
    }
    inline CaptureData& operator[] (size_t i)
    {
        return val_[i];
    }
};

// multi-view + multi-frame capture data
struct MFMVCaptureData
{
    std::vector<MVCaptureData> frames_;
    
    MFMVCaptureData(){
        frames_.resize(1);
    }
    MFMVCaptureData(int i){
        frames_.resize(i);
    }
    MFMVCaptureData(int i, int j){
        frames_.assign(i, MVCaptureData(j));
    }
    
    inline const MVCaptureData& operator[] (size_t i) const
    {
        return frames_[i];
    }
    inline MVCaptureData& operator[] (size_t i)
    {
        return frames_[i];
    }
};
