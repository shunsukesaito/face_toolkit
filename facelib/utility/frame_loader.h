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

class FrameLoader;
typedef std::shared_ptr<FrameLoader> FrameLoaderPtr;

class FrameLoader
{
public:
    FrameLoader(){}
    ~FrameLoader(){}
    
    virtual void load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command){ throw std::runtime_error( "Error: Base class (FrameLoader) is called..."); }
    virtual void init(){ throw std::runtime_error( "Error: Base class (FrameLoader) is called..."); }
};

class EmptyLoader : public FrameLoader
{
public:
    EmptyLoader(){}
    ~EmptyLoader(){}
    
    virtual void load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command);
    virtual void init(){}
    
    static FrameLoaderPtr Create(){
        return FrameLoaderPtr(new EmptyLoader());
    }

private:
    cv::Mat frame_ = cv::Mat(100,100,CV_8UC3,cv::Vec3b(0,0,0));
};

class VideoLoader : public FrameLoader
{
public:
    // initializes a module
    VideoLoader(const std::string &video_path, float scale = 1.0);
    VideoLoader(int device_id, float scale = 1.0);
    
    // destructor
    ~VideoLoader();
    
    virtual void load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command);
    virtual void init();
    
    static FrameLoaderPtr Create(const std::string &video_path, float scale = 1.0);
    static FrameLoaderPtr Create(int device_id, float scale = 1.0);
    
private:
    cv::Mat frame_;
    std::string video_path_ = "";
    int device_id_ = -1;
    float scale_ = 1.0;
    
    cv::VideoCapture video_capture_;
};

class SingleImageLoader : public FrameLoader
{
public:
    // initializes a module
    SingleImageLoader(const std::string &image_path, float scale = 1.0);
    
    // destructor
    ~SingleImageLoader();
    
    virtual void load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command);
    virtual void init();
    
    static FrameLoaderPtr Create(const std::string &image_path, float scale = 1.0);
    
private:
    cv::Mat frame_;
    std::string image_path_;
    float scale_ = 1.0;
};

class ImageSequenceLoader : public FrameLoader
{
public:
    // initializes a module
    ImageSequenceLoader(const std::string &root_dir, const std::string &imgseq_fmt, int begin_id, int end_id, float scale = 1.0);
    ImageSequenceLoader(const std::string &root_dir, const std::string& list_file, float scale = 1.0);

    // destructor
    ~ImageSequenceLoader();
    
    virtual void load_frame(cv::Mat& frame, int& frame_id, std::string& name, std::string command);
    virtual void init();
    
    static FrameLoaderPtr Create(const std::string &root_dir, const std::string &imgseq_fmt, int begin_id, int end_id, float scale = 1.0);
    static FrameLoaderPtr Create(const std::string &root_dir, const std::string& list_file, float scale = 1.0);

private:
    cv::Mat frame_;
    std::vector<std::string> file_list_;
    std::string root_dir_;
    float scale_ = 1.0;
};
