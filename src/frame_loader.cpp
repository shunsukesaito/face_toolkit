//
//  frame_loader.cpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 12/4/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "frame_loader.h"

// initializes this module and the basic module
VideoLoader::VideoLoader(const std::string &video_path) : FrameLoader(), video_path_(video_path)
{
    // nothing to do
}

// initializes this module and the basic module
VideoLoader::VideoLoader(int device_id) : FrameLoader(), device_id_(device_id)
{
    // nothing to do
}

// default destructor
VideoLoader::~VideoLoader()
{
    // nothing to do
}

void VideoLoader::init()
{
    if(video_path_ != ""){
        if(!video_capture_.open(video_path_)){
            std::cout << "Error: video file does not exist. " << video_path_ << std::endl;
            throw std::runtime_error("Error: video file does not exist. ");
        }
    }
    else if(device_id_ != -1){
        if(!video_capture_.open(device_id_)){
            std::cout << "Error: video device cannot open. " << device_id_ << std::endl;
            throw std::runtime_error("Error: video device cannot open.");
        }
    }
}

void VideoLoader::load_frame(cv::Mat& frame, std::string command)
{
    if(command != "pause"){
        video_capture_ >> frame;
        // flip image if it's streaming
        if(device_id_ != -1)
            cv::flip(frame, frame, 1);
    }
}

FrameLoaderPtr VideoLoader::Create(const std::string &video_path)
{
    auto loader = new VideoLoader(video_path);
    
    return FrameLoaderPtr(loader);
}

FrameLoaderPtr VideoLoader::Create(int device_id)
{
    auto loader = new VideoLoader(device_id);
    
    return FrameLoaderPtr(loader);
}

SingleImageLoader::SingleImageLoader(const std::string &image_path) : FrameLoader()
{
    frame_ = cv::imread(image_path);
    if(frame_.empty()){
        std::cout << "Error: image file does not exist. " << image_path << std::endl;
        throw std::runtime_error("Error: image file does not exist. ");
    }
}

// default destructor
SingleImageLoader::~SingleImageLoader()
{
    // nothing to do
}

void SingleImageLoader::init()
{
    // nothing to do
}

void SingleImageLoader::load_frame(cv::Mat& frame, std::string command)
{
    frame = frame_.clone();
}

FrameLoaderPtr SingleImageLoader::Create(const std::string &image_path)
{
    auto loader = new SingleImageLoader(image_path);
    
    return FrameLoaderPtr(loader);
}



