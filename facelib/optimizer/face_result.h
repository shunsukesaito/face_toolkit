//
//  face_result.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 12/4/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <fstream>
#include <gl_utility/camera.h>

#include <shape_model/face_model.h>
#include "constraints.h"

inline void getVectorFromString(const std::string &line, std::vector<float>& vec)
{
    std::istringstream iss(line);
    float tmp_f;
    vec.clear();
    while (iss >> tmp_f)
    {
        vec.push_back(tmp_f);
    }
}

struct FaceResult
{
    bool processed_ = false;
    int frame_id = 0;
    
    // assume single camera for now
    cv::Mat img;
    Camera camera;
    
    FaceData fd;
    
    std::vector<P2P2DC> c_p2p;
    std::vector<P2L2DC> c_p2l;
    
    std::vector<Eigen::Vector3f> p2d;
    
    inline void loadFromTXT(std::string filename){
        std::ifstream infile(filename);
        std::string line;
        
        std::vector<float> tmp;
        // identity
        std::getline(infile, line);
        getVectorFromString(line, tmp);
        fd.idCoeff.segment(0,tmp.size()) = Eigen::Map<Eigen::VectorXf>(&tmp[0],tmp.size());
        // expression
        std::getline(infile, line);
        getVectorFromString(line, tmp);
        fd.exCoeff.segment(0,tmp.size()) = Eigen::Map<Eigen::VectorXf>(&tmp[0],tmp.size());
        // albedo
        std::getline(infile, line);
        getVectorFromString(line, tmp);
        fd.alCoeff.segment(0,tmp.size()) = Eigen::Map<Eigen::VectorXf>(&tmp[0],tmp.size());
        // rotation
        std::getline(infile, line);
        getVectorFromString(line, tmp);
        camera.extrinsic_.block(0,0,3,3) = Eigen::Map<Eigen::Matrix3f>(&tmp[0]).transpose();
        // translation
        std::getline(infile, line);
        getVectorFromString(line, tmp);
        camera.extrinsic_.block(0,3,3,1) = Eigen::Map<Eigen::Vector3f>(&tmp[0]);
        // spherical hamonics
        std::getline(infile, line);
        getVectorFromString(line, tmp);
        fd.SH = Eigen::Map<Eigen::MatrixXf>(&tmp[0],9,3).transpose();
        // camera intrinsic
        std::getline(infile, line);
        getVectorFromString(line, tmp);
        camera.intrinsic_.block(0,0,3,3) = Eigen::Map<Eigen::Matrix3f>(&tmp[0]).transpose();
        
//        std::cout << fd.idCoeff.transpose() << std::endl;
//        std::cout << fd.exCoeff.transpose() << std::endl;
//        std::cout << fd.alCoeff.transpose() << std::endl;
//        std::cout << fd.RT << std::endl;
//        std::cout << fd.SH << std::endl;
//        std::cout << camera.extrinsic_ << std::endl;
//        std::cout << camera.intrinsic_ << std::endl;
    }
};

inline std::vector<Eigen::Vector3f> getP3DFromP2PC(const Eigen::VectorXf& pts, const std::vector<P2P2DC>& c_p2p)
{
    std::vector<Eigen::Vector3f> out;
    
    for(auto&& c : c_p2p)
    {
        out.push_back(pts.b3(c.v_idx));
    }
    
    return out;
}

inline std::vector<Eigen::Vector3f> getP3DFromP2LC(const Eigen::VectorXf& pts, const std::vector<P2L2DC>& c_p2l)
{
    std::vector<Eigen::Vector3f> out;
    
    for(auto&& c : c_p2l)
    {
        out.push_back(pts.b3(c.v_idx));
    }
    
    return out;
}
