//
//  face_2dtracker.h
//  FaceFitting
//
//  Created by SaitoShunsuke on 9/25/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#ifndef face_landmark2d_hpp
#define face_landmark2d_hpp

// std includes
#include <memory>

#include <opencv2/opencv.hpp>

//#define ENABLE_ASSERTS

#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>

#include <Eigen/Core>

#include <tcp_stream.h>

#include "NPD/LearnGAB.h"

// face detection
#define DLIB_68_FACEALIGNMENT_MODEL ("shape_predictor_68_face_landmarks.dat")
#define DLIB_73_FACEALIGNMENT_MODEL ("shape_predictor_73_face_landmarks.dat")
#define NPD_MODEL ("1226model.bin")

class Face2DDetector;
typedef std::shared_ptr<Face2DDetector> Face2DDetectorPtr;

cv::Rect ScaleRect(const cv::Rect& rect,float scale);
void DrawLandmarks(cv::Mat& img, const std::vector<Eigen::Vector2f>& p2d);

struct LandmarkFrame : public Frame {
    std::vector<Eigen::Vector3f> lands;
};
struct ImageFrame : public Frame {
    cv::Mat image;
};

struct LandmarkCpmTCPStream : public TCPStream {
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
            TCPStream(ip, 2233,
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
        auto img_frame = static_pointer_cast<ImageFrame>(frame);
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
        auto ret = make_shared<LandmarkFrame>();
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
        if(rect.width != rect.height){
            int len = 1.1*std::max(rect.width,rect.height);
            int cx = rect.x + rect.width/2;
            int cy = rect.y + rect.height/2;
            face_rect.x = cx - len/2;
            face_rect.y = cy - len/2;
            face_rect.width = len;
            face_rect.height = len;
        }
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
        
        auto ret = make_shared<ImageFrame>();
        ret->image = img_crop;
        setInput(ret);
    }
    inline std::vector<Eigen::Vector3f> getLandmarks(){
        auto frame = getOutput(100000);
        auto ret = static_pointer_cast<LandmarkFrame>(frame);
        return ret->lands;
    }
};

class Face2DDetector
{
public:
    dlib::frontal_face_detector detector_;
    dlib::shape_predictor sp_; // Kazemi CVPR2014
    
    std::shared_ptr<LandmarkCpmTCPStream> cpm_tcp_;
    
    GAB gab_detector_;
    
public:
    Face2DDetector(std::string data_dir);
    
    void        GetFaceRects(const cv::Mat &img,
                             std::vector<cv::Rect>& rects,
                             bool enable_dlib,
                             int max_size = 600,
                             int min_size = 60);
    
    bool        GetFaceLandmarks(const cv::Mat &img,
                                 std::vector<Eigen::Vector2f>& p2d,
                                 cv::Rect& rect,
                                 bool enable_dlib = false,
                                 bool enable_cpm = false);
    
    bool        GetFaceLandmarks(const cv::Mat &img,
                                 const cv::Rect &rect,
                                 std::vector<Eigen::Vector2f>& p2d,
                                 bool enable_cpm = false);
    
    bool        CropFaceImage(cv::Mat& output,
                              cv::Rect& rect,
                              const cv::Mat &img,
                              float width = -1);
    
    bool        CropFaceImage(cv::Mat& output,
                              cv::Rect& rect,
                              cv::Rect& rectCPM,
                              const cv::Mat &img,
                              cv::Mat& imgCPM,
                              float cpm_width,
                              float cpm_crop_pos,
                              float cpm_crop_width,
                              float width = -1);
    
    void detect(cv::Mat& frame, cv::Rect &r, cv::Mat& output);
};

// it doesn't work well atm.
class Face2DBoxTracker
{
public:
    Face2DBoxTracker(Face2DDetectorPtr faceDetector) : faceDetector_(faceDetector){};
    Face2DBoxTracker(){};
    
    Face2DDetectorPtr faceDetector_;
    
    // for tracking
    cv::Mat faceTemplate_;
    cv::Mat matchingResult_;
    cv::Mat matchingResultNorm_;
    bool templateMatchingRunning_;
    cv::Rect trackedFace_;
    cv::Rect faceRoi_;
    bool foundFace_ = false;
    int resizedWidth_ = 320;
    double scale_;
    cv::Point facePosition_;
    double templateMatchingMaxDuration_ = 10;
    int64 templateMatchingStartTime_ = 0;
    int64 templateMatchingCurrentTime_ = 0;
    
    // for face patch
    cv::Mat matchingResultFace_;
    cv::Mat matchingResultFaceNorm_;
    cv::Mat faceROIImage_;
    cv::Rect faceRoiOld_;
    cv::Rect trackedFaceOld_;
public:
    inline void SetDetector(Face2DDetectorPtr faceDetector){faceDetector_ = faceDetector;}
    bool        getFrameAndDetect(cv::Mat &frame,const std::vector<Eigen::Vector2f>& Pcur,cv::Point& dp,cv::Rect& rect);
    bool        getFrameAndDetect(cv::Mat &frame,cv::Rect& rect);
    bool        detectFaceAroundRoi(const cv::Mat &frame);
    bool        detectFacesTemplateMatching(const cv::Mat &frame);
    cv::Rect    doubleRectSize(const cv::Rect &inputRect, const cv::Rect &frameSize) const;
    cv::Rect    biggestFace(std::vector<cv::Rect> &faces) const;
    cv::Rect    getFaceTemplateRect(const std::vector<Eigen::Vector2f>& p2d);
    cv::Rect    getFaceTemplateRect(const cv::Rect& rect);
    cv::Point   centerOfRect(const cv::Rect &rect) const;
    cv::Mat     getFaceTemplate(const cv::Mat &frame, cv::Rect face);
    bool        detectFaceAllSizes(const cv::Mat &frame);
    void        setInitialFace(const cv::Mat &frame,const cv::Rect& rect);
    
    float detectFaceTemplateMatching(const cv::Mat &faceTemplate,
                                     const cv::Mat &frame,
                                     cv::Point& p);
    
    cv::Rect    getFaceROI();
};


#endif /* face_landmark2d_hpp */
