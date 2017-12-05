//
//  frame_loader.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 12/4/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef frame_loader_h
#define frame_loader_h

#include <iostream>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

class FrameLoader;
typedef std::shared_ptr<FrameLoader> FrameLoaderPtr;

class FrameLoader
{
public:
    FrameLoader(){}
    ~FrameLoader(){}
    
    virtual void load_frame(cv::Mat& frame, std::string command){ throw std::runtime_error( "Error: Base class is called..."); }
    virtual void init(){ throw std::runtime_error( "Error: Base class is called..."); }
};

class VideoLoader : public FrameLoader
{
public:
    // initializes a module
    VideoLoader(const std::string &video_path);
    VideoLoader(int device_id);
    
    // destructor
    ~VideoLoader();
    
    virtual void load_frame(cv::Mat& frame, std::string command);
    virtual void init();
    
    static FrameLoaderPtr Create(const std::string &video_path);
    static FrameLoaderPtr Create(int device_id);
    
private:
    std::string video_path_ = "";
    int device_id_ = -1;
    
    cv::VideoCapture video_capture_;
};

class SingleImageLoader : public FrameLoader
{
public:
    // initializes a module
    SingleImageLoader(const std::string &image_path);
    
    // destructor
    ~SingleImageLoader();
    
    virtual void load_frame(cv::Mat& frame, std::string command);
    virtual void init();
    
    static FrameLoaderPtr Create(const std::string &image_path);
    
private:
    cv::Mat frame_;
};

#endif /* frame_loader_hpp */
