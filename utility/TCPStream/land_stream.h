#pragma once

#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "tcp_stream.h"

struct LandmarkCpmTCPStream : public TCPStreamSync {
    const int map_size;
    const int cpm_num;
    const int precrop_size;
    float scale, shift;
    cv::Rect crop;
    int img_width;
    int img_height;
    cv::Rect face_rect;
    LandmarkCpmTCPStream(std::string ip,
                         int cpm_width = 256,
                         int cpm_precrop_size = 500,
                         int cpm_crop_width = 500,
                         int cpm_crop_pos = 0,
                         int cpm_num = 68) :
            TCPStreamSync(ip, 2233,
                      cpm_width * cpm_width * 3,
                      3 * cpm_num * sizeof(float)),
            map_size(cpm_width),
            cpm_num(cpm_num),
            precrop_size(cpm_precrop_size) {
        int crop_pos = cpm_crop_pos;
        int crop_width = cpm_crop_width;
        scale = float(crop_width) / cpm_width;
        shift = float(crop_pos);
        crop = cv::Rect(crop_pos, crop_pos, crop_width, crop_width);
    }
    
    inline std::vector<char> encode(FramePtr frame) {
        auto img_frame = std::static_pointer_cast<ImageFrame>(frame);
        cv::Mat img = img_frame->image;
        cv::Mat tmp;
        img_width = img.cols;
        img_height = img.rows;
        cv::resize(img, tmp, cv::Size(precrop_size, precrop_size));
        cv::Mat crop_img = tmp(crop);
        std::vector<char> ret(map_size * map_size * 3);
        cv::Mat img_resize(map_size, map_size, CV_8UC3, ret.data());
        cv::resize(crop_img, img_resize, cv::Size(map_size, map_size));
        return ret;
    }
    
    inline FramePtr decode(std::vector<char> buffer) {
        float* lands = (float*)buffer.data();
        auto ret = std::make_shared<LandmarkFrame>();
        ret->lands.resize(cpm_num);
        auto& p2d = ret->lands;
        float outer_scale_x = float(img_width) / precrop_size;
        float outer_scale_y = float(img_height) / precrop_size;
        for (int i = 0; i < p2d.size(); i++) {
            p2d[i] = Eigen::Vector3f(lands[i*3+0], lands[i*3+1], lands[i*3+2]);
            
            float x=p2d[i][0]*scale + shift;
            float y=p2d[i][1]*scale + shift;
            
            p2d[i][0] = x * outer_scale_x + face_rect.x;
            p2d[i][1] = y * outer_scale_y + face_rect.y;
        }
        
        return ret;
    }

    inline void sendImage(const cv::Mat& img, const cv::Rect& rect){
        face_rect = rect;
        cv::Mat img_crop;
        cv::Rect inter = (face_rect & cv::Rect(0, 0, img.cols, img.rows));
        if( inter != face_rect)
        {
            img_crop = cv::Mat::zeros(face_rect.size(), img.type());
            img(inter).copyTo(img_crop(inter-cv::Point(face_rect.tl())));
        }
        else
        {
            img_crop = img(face_rect).clone();
        }
        
        auto ret = std::make_shared<ImageFrame>();
        ret->image = img_crop;
        check_uv(uv_run(uv_default_loop(), UV_RUN_NOWAIT));
        setInput(ret);
        check_uv(uv_run(uv_default_loop(), UV_RUN_NOWAIT));
    }
    inline std::vector<Eigen::Vector3f> getLandmarks(){
        auto frame = getOutput(10000);
        while(frame == nullptr){
            std::cout << "frame null? " << (frame == nullptr) << std::endl;
            frame = getOutput(10000);
        }
        auto ret = std::static_pointer_cast<LandmarkFrame>(frame);
        return ret->lands;
    }
};
