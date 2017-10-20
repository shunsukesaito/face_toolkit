//
//  train_util.hpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 9/18/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef train_util_hpp
#define train_util_hpp

#include <iostream>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include "face_model.h"

namespace cao{
    
struct ContourPoint
{
    ContourPoint(){};
    ContourPoint(const int32_t _p1,const int32_t _p2, const float _a)
    : p1(_p1), p2(_p2), a(_a){};
    
    int32_t p1;
    int32_t p2;
    float a;
};
    
struct DOF
{
    int EX;
    int ROT;
    int TR;
    int DISP;
    
    inline cv::Rect roiEX() const { return cv::Rect(0, 0, EX, 1);}
    inline cv::Rect roiROT() const { return cv::Rect(EX, 0, ROT, 1);}
    inline cv::Rect roiTR() const {return cv::Rect(EX+ROT, 0, TR, 1);}
    inline cv::Rect roiRT() const {return cv::Rect(EX, 0, ROT+TR, 1);}
    inline cv::Rect roiDISP() const {return cv::Rect(EX+ROT+TR, 0, DISP, 1);}
};
    
struct TrainParams
{
    bool with_oh = false; // with occlusion handling
    int l_eye_idx = -1;
    int r_eye_idx = -1;
    
    int T = 5;
    int K = 100;
    int P = 500;
    float kappa = -1;
    std::vector<float> kappas;
    int F = 5;
    int beta = -1;
    int n_aug_exp = 15;
    int n_aug_other = 5;
    float dR = 0.05; // purtabation scale of rotation
    float dTxy = 50.0; // purtabation scale of translation
    float dTz = 100.0;
    float dFl = 200.0; // purtabation scale of focal length
    
    cv::Mat_<int> tri;
    DOF dof;
};

struct Data
{
    cv::Mat_<float> gt_x;
    cv::Mat_<cv::Vec2f> gt_p2d;
    
    cv::Mat_<float> cur_x;
    cv::Mat_<cv::Vec2f> cur_p2d;
    
    float fl;
    Eigen::VectorXf idCoeff;
    
    cv::Rect rect;
    cv::Mat_<uchar> mask;
    cv::Mat_<uchar> img;
    
    int id = -1;
    
    Data(const Data& other){
        // shallow copy
        gt_x = other.gt_x;
        gt_p2d = other.gt_p2d;
        img = other.img;
        mask = other.mask;
        
        // deep copy
        cur_x = other.cur_x.clone();
        cur_p2d = other.cur_p2d.clone();
        
        rect = other.rect;
        fl = other.fl;
        idCoeff = other.idCoeff;
        
        id = other.id;
    }
    
    const Data &operator = (const Data &other){
        // check for self-assignment
        if(&other == this)
            return *this;
        // reuse storage when possible
        
        // shallow copy
        this->gt_x = other.gt_x;
        this->gt_p2d = other.gt_p2d;
        this->img = other.img;
        this->mask = other.mask;
        
        // deep copy
        this->cur_x = other.cur_x.clone();
        this->cur_p2d = other.cur_p2d.clone();
        
        this->rect = other.rect;
        this->fl = other.fl;
        this->idCoeff = other.idCoeff;
        
        this->id = other.id;
        
        return *this;
    }
};
  
    void augmentRotation(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, float dR, const DOF& dof);
    void augmentTranslation(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, float dTxy, float dTz, const DOF& dof);
    void augmentExpression(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, const DOF& dof);
    void augmentFocalLength(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, float dF, const DOF& dof);
    void augmentIdentity(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, const DOF& dof);
    void augmentOcclusion(std::vector<Data>& in);
    
    void augmentData(std::vector<Data>& out, const std::vector<Data>& in, const TrainParams& params);
    
}

#endif /* train_util_hpp */
