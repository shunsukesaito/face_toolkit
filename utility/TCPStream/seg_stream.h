#pragma once

// std includes
#include <memory>

#include <opencv2/opencv.hpp>

#include "tcp_stream.h"

#include "cv_utils.h"

struct SegmentationTCPStream : public TCPStreamSync {
    const int map_size;
    cv::Size img_size;
    cv::Mat img;
    cv::Rect img_rect;
    SegmentationTCPStream(std::string ip,
                         int probmap_size = 256) :
        TCPStreamSync(ip, 1153,
              probmap_size * probmap_size * 3,
              probmap_size * probmap_size),
        map_size(probmap_size) {
        
    }
    
    std::vector<char> encode(FramePtr frame) {
        auto img_frame = std::static_pointer_cast<ImageFrame>(frame);
        img = img_frame->image;
        std::vector<char> ret(map_size * map_size * 3);
        cv::Mat img_resize(map_size, map_size, CV_8UC3, ret.data());
        cv::resize(img, img_resize, cv::Size(map_size, map_size));
        return ret;
    }
    
    FramePtr decode(std::vector<char> buffer) {
        cv::Mat seg(map_size, map_size, CV_8UC1, buffer.data());
        ImageFramePtr ret = std::make_shared<ImageFrame>();
        seg = seg*255;
        cv::resize(seg, seg, img.size(), cv::INTER_NEAREST);
        
        ret->image = cv::Mat(img_size.height,img_size.width,CV_8UC1,cv::Scalar(0));
        insert_image(seg, ret->image, img_rect);
        
#ifdef _PSC_DEBUG_
        cv::imwrite("segmentation.png", seg);
        cv::Mat img_copy = img.clone();
        for(int y = 0; y < img_copy.cols; ++y)
        {
            for(int x = 0; x < img_copy.rows; ++x)
            {
                img_copy.at<cv::Vec3b>(y,x)[2] = ret->image.at<uchar>(y,x);
            }
        }
        
        cv::imwrite("overlay.png",img_copy);
#endif        
        return ret;
    }
    
    inline void sendImage(const cv::Mat& img){
        auto ret = std::make_shared<ImageFrame>();
        ret->image = img;
        img_size = img.size();
        img_rect = cv::Rect(0,0,img.cols,img.rows);
        setInput(ret);
    }

    inline void sendImage(const cv::Mat& img, const cv::Rect& rect, float rect_scale = 1.0){
        auto ret = std::make_shared<ImageFrame>();
        img_size = img.size();
        img_rect = scale_rect(rect, rect_scale);
        crop_image(img, ret->image, img_rect);
        setInput(ret);
    }

    inline void getSegmentation(cv::Mat& img){
        auto frame = getOutput(100000);
        auto ret = std::static_pointer_cast<ImageFrame>(frame);
        img = ret->image.clone();
    }
};

typedef std::shared_ptr<SegmentationTCPStream> SegmentationTCPStreamPtr;
