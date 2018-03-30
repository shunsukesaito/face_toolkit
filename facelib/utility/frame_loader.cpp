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
    frame = frame_;
}

FrameLoaderPtr SingleImageLoader::Create(const std::string &image_path)
{
    auto loader = new SingleImageLoader(image_path);
    
    return FrameLoaderPtr(loader);
}

ImageSequenceLoader::ImageSequenceLoader(const std::string &imgseq_fmt, int begin_id, int end_id) : FrameLoader()
{
    char file_name[256];
    file_list_.clear();
    for (int i = begin_id; i <= end_id; ++i)
    {
        sprintf(file_name, imgseq_fmt.c_str(), i);
        std::ifstream dummy(file_name);
        if (dummy.good())
            file_list_.push_back(file_name);
    }
    
    if(file_list_.size() == 0){
        std::cout << "Error: image sequence does not exist. " << imgseq_fmt << std::endl;
        throw std::runtime_error("Error: image sequence does not exist. ");
    }
}

// default destructor
ImageSequenceLoader::~ImageSequenceLoader()
{
    // nothing to do
}

void ImageSequenceLoader::init()
{
    // nothing to do
}

void ImageSequenceLoader::load_frame(cv::Mat& frame, std::string command)
{
    
}

FrameLoaderPtr ImageSequenceLoader::Create(const std::string &imgseq_fmt, int begin_id, int end_id)
{
    auto loader = new ImageSequenceLoader(imgseq_fmt, begin_id, end_id);
    
    return FrameLoaderPtr(loader);
}




