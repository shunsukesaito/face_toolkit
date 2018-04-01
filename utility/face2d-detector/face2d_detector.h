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

#include <land_stream.h>

#include "NPD/LearnGAB.h"

// face detection
#define DLIB_68_FACEALIGNMENT_MODEL ("shape_predictor_68_face_landmarks.dat")
#define DLIB_73_FACEALIGNMENT_MODEL ("shape_predictor_73_face_landmarks.dat")
#define NPD_MODEL ("1226model.bin")

class Face2DDetector;
typedef std::shared_ptr<Face2DDetector> Face2DDetectorPtr;

cv::Rect ScaleRect(const cv::Rect& rect,float scale);
void DrawLandmarks(cv::Mat& img, const std::vector<Eigen::Vector3f>& p2d);
cv::Rect GetBBoxFromLandmarks(std::vector<Eigen::Vector3f>& shape);

class Face2DDetector
{
public:
    dlib::frontal_face_detector detector_;
    dlib::shape_predictor sp_; // Kazemi CVPR2014
    
    std::shared_ptr<LandmarkCpmTCPStream> cpm_tcp_;
    
    GAB gab_detector_;
    
public:
    Face2DDetector(std::string data_dir);
    
    bool        GetFaceRect(const cv::Mat &img,
                            cv::Rect& rect,
                            bool enable_dlib = false);

    void        GetFaceRects(const cv::Mat &img,
                             std::vector<cv::Rect>& rects,
                             bool enable_dlib,
                             int max_size = 600,
                             int min_size = 60);
    
    bool        GetFaceLandmarks(const cv::Mat &img,
                                 std::vector<Eigen::Vector3f>& p2d,
                                 cv::Rect& rect,
                                 bool enable_dlib = false,
                                 bool enable_cpm = false);
    
    bool        GetFaceLandmarks(const cv::Mat &img,
                                 const cv::Rect &rect,
                                 std::vector<Eigen::Vector3f>& p2d,
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
