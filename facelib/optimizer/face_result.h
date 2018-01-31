//
//  face_result.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 12/4/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <gl_utility/camera.h>

#include <shape_model/face_model.h>
#include "constraints.h"

struct FaceResult
{
    bool processed_ = false;
    
    // assume single camera for now
    cv::Mat img;
    Camera camera;
    
    FaceData fd;
    
    std::vector<P2P2DC> c_p2p;
    std::vector<P2L2DC> c_p2l;
    
    std::vector<Eigen::Vector2f> p2d;
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
