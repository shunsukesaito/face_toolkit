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

#include "frame_loader.h"

// initializes this module and the basic module
VideoLoader::VideoLoader(const std::string &video_path, float scale) : FrameLoader(), video_path_(video_path), scale_(scale)
{
    // nothing to do
}

// initializes this module and the basic module
VideoLoader::VideoLoader(int device_id, float scale) : FrameLoader(), device_id_(device_id), scale_(scale)
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

void VideoLoader::load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command)
{
    if(command != "pause"){
        video_capture_ >> frame_;
        if(scale_ != 1.0)
            cv::resize(frame_, frame_, cv::Size(), scale_, scale_);
        frame_id++;
        // flip image if it's streaming
        if(device_id_ != -1)
            cv::flip(frame_, frame_, 1);
    }
    frame = frame_;
}

FrameLoaderPtr VideoLoader::Create(const std::string &video_path, float scale)
{
    auto loader = new VideoLoader(video_path,scale);
    
    return FrameLoaderPtr(loader);
}

FrameLoaderPtr VideoLoader::Create(int device_id, float scale)
{
    auto loader = new VideoLoader(device_id,scale);
    
    return FrameLoaderPtr(loader);
}

void EmptyLoader::load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command)
{
    if(command != "pause"){
        frame_id++;
    }
    frame = frame_;
}

SingleImageLoader::SingleImageLoader(const std::string &image_path, float scale) : FrameLoader()
{
    image_path_ = image_path;
    scale_ = scale;
    frame_ = cv::imread(image_path);
    if(scale_ != 1.0)
        cv::resize(frame_, frame_, cv::Size(), scale_, scale_);
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

void SingleImageLoader::load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command)
{
    frame = frame_;
    name = image_path_;
    frame_id = 0;
}

FrameLoaderPtr SingleImageLoader::Create(const std::string &image_path, float scale)
{
    auto loader = new SingleImageLoader(image_path, scale);
    
    return FrameLoaderPtr(loader);
}

ImageSequenceLoader::ImageSequenceLoader(const std::string &root_dir, const std::string &imgseq_fmt, int begin_id, int end_id, float scale) : FrameLoader()
{
    root_dir_ = root_dir;
    scale_ = scale;
    char file_name[256];
    file_list_.clear();
    for (int i = begin_id; i <= end_id; ++i)
    {
        sprintf(file_name, imgseq_fmt.c_str(), i);
        std::ifstream dummy(root_dir_ + "/" + file_name);
        if (dummy.good())
            file_list_.push_back(file_name);
    }
    
    if(file_list_.size() == 0){
        std::cout << "Error: image sequence does not exist. " << imgseq_fmt << std::endl;
        throw std::runtime_error("Error: image sequence does not exist. ");
    }
}

ImageSequenceLoader::ImageSequenceLoader(const std::string &root_dir, const std::string &list_file, float scale) : FrameLoader()
{
    root_dir_ = root_dir;
    scale_ = scale;
    file_list_.clear();
    std::ifstream fin(list_file);
    if(!fin.is_open()){
        std::cout << "Warning: failed parsing image sequence from " << list_file << std::endl;
        throw std::runtime_error("Error: image sequence does not exist. ");
    }
    
    std::string f;
    while(std::getline(fin, f))
    {
        if (f.empty())
            continue;
        std::ifstream dummy(root_dir_ + "/" + f);
        //std::cout <<root_dir_ + "/" + f << std::endl;
        if (dummy.good())
            file_list_.push_back(f);
        //std::cout << file_list_.size() << std::endl;
    }
    
    if(file_list_.size() == 0){
        std::cout << "Error: image sequence does not exist. " << root_dir << std::endl;
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

void ImageSequenceLoader::load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command)
{
    if(command != "pause"){
        if(++frame_id >= file_list_.size() || frame_id < 0)
            frame_id = 0;
        
        frame_ = cv::imread(root_dir_ + "/" + file_list_[frame_id]);
        if(scale_ != 1.0)
            cv::resize(frame_, frame_, cv::Size(), scale_, scale_);
        name = root_dir_ + "/" + file_list_[frame_id];
        if(frame_.empty()){
            std::cout << "Error: image file does not exist. " << root_dir_ + "/" + file_list_[frame_id] << std::endl;
            throw std::runtime_error("Error: image file does not exist. ");
        }
    }
    frame = frame_;
}

FrameLoaderPtr ImageSequenceLoader::Create(const std::string &root_dir, const std::string &imgseq_fmt, int begin_id, int end_id, float scale)
{
    auto loader = new ImageSequenceLoader(root_dir, imgseq_fmt, begin_id, end_id, scale);
    
    return FrameLoaderPtr(loader);
}

FrameLoaderPtr ImageSequenceLoader::Create(const std::string &root_dir, const std::string &list_file, float scale)
{
    auto loader = new ImageSequenceLoader(root_dir, list_file, scale);
    
    return FrameLoaderPtr(loader);
}



