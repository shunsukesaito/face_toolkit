/*
 MIT License
 
 Copyright (c) 2018 Shunsuke Saito
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
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
