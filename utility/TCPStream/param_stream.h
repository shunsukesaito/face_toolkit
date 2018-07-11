//
//  param_stream.h
//  faceview
//
//  Created by Shunsuke Saito on 6/20/18.
//

#pragma once

#include <memory>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "tcp_stream.h"

struct ParamTCPStream : public TCPStreamSync {
    std::vector<std::pair<std::string, int>> param_dof;
    int total_dof;
    int img_size;
    ParamTCPStream(std::string _ip,
                   int _port,
                   int _total_dof,
                   int _img_size,
                   const std::vector<std::pair<std::string, int>>& _param_dof) :
    TCPStreamSync(_ip, _port, _img_size*_img_size*3, _total_dof * sizeof(float)),
    total_dof(_total_dof),
    img_size(_img_size),
    param_dof(_param_dof){}
    
    inline std::vector<char> encode(FramePtr frame) {
        auto img_frame = std::static_pointer_cast<ImageFrame>(frame);
        cv::Mat img = img_frame->image;
        std::vector<char> ret(img_size * img_size * 3);
        cv::Mat img_resize(img_size, img_size, CV_8UC3, ret.data());
        cv::resize(img, img_resize, cv::Size(img_size, img_size));
        return ret;
    }
    
    inline FramePtr decode(std::vector<char> buffer) {
        float* param = (float*)buffer.data();
        ParamFramePtr ret = std::make_shared<ParamFrame>();
        ret->param.resize(total_dof);
        ret->dof = param_dof;
        auto& val = ret->param;
        for (int i = 0; i < val.size(); i++) {
            val[i] = param[i];
        }
        
        return ret;
    }
    
    inline void sendImage(const cv::Mat& img, const cv::Rect& rect){
        cv::Mat img_crop;
        cv::Rect inter = (rect & cv::Rect(0, 0, img.cols, img.rows));
        if( inter != rect)
        {
            img_crop = cv::Mat::zeros(rect.size(), img.type());
            img(inter).copyTo(img_crop(inter-cv::Point(rect.tl())));
        }
        else
        {
            img_crop = img(rect).clone();
        }
        
        auto ret = std::make_shared<ImageFrame>();
        ret->image = img_crop;
        setInput(ret);
    }
    inline ParamFramePtr getParams(){
        auto frame = getOutput(10000);
        while(frame == nullptr){
            std::cout << "frame null? " << (frame == nullptr) << std::endl;
            frame = getOutput(10000);
        }
        auto ret = std::static_pointer_cast<ParamFrame>(frame);
        return ret;
    }
};

