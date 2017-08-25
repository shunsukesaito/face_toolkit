//
//  camera.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef camera_hpp
#define camera_hpp

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gl_core.hpp"

struct Camera
{
    Eigen::Matrix4f extrinsic_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f intrinsic_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f perspective_;
    
    float zNear_ = 0.1f;
    float zFar_ = 10000.f;
    int width_;
    int height_;
    
    Camera() {}
    Camera(const Eigen::Matrix4f& RT, const Eigen::Matrix4f& K, int w, int h, float zN, float zF, bool c2w = false);
    Camera(const Camera&);
    
    void intializeUniforms(GLProgram& programs);
    void updateUniforms(GLProgram& program);
    void updateUniforms(GLProgram& program, const Eigen::Matrix4f& RT);
};

#endif /* camera_hpp */
