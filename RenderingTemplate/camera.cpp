//
//  camera.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "camera.hpp"

Camera::Camera(const Eigen::Matrix4f& RT, const Eigen::Matrix4f& K, int w, int h, float zN, float zF, bool c2w)
: intrinsic_(K), width_(w), height_(h), zNear_(zN), zFar_(zF)
{
    if(c2w){
        Eigen::Matrix3f Rt = RT.block<3,3>(0,0).transpose();
        extrinsic_.block(0, 3, 3, 1) = -Rt * RT.block(0, 3, 3, 1);
        extrinsic_.block(0, 0, 3, 3) = Rt;
        
        Eigen::Matrix4f flip; flip.setIdentity();
        flip.block(0, 0, 3, 3) = Eigen::Quaternion<float>(0, 1, 0, 0).toRotationMatrix();
        
        extrinsic_ = flip * extrinsic_;
    }
    else{
        extrinsic_ = RT;
    }
}

Camera::Camera(const Camera& other)
{
    extrinsic_ = other.extrinsic_;
    intrinsic_ = other.intrinsic_;
    perspective_ = other.perspective_;
    
    zNear_ = other.zNear_;
    zFar_ = other.zFar_;
    
    width_ = other.width_;
    height_ = other.height_;
}

void Camera::intializeUniforms(GLProgram& program)
{
    program.createUniform("u_mvp", DataType::MATRIX44);
    program.createUniform("u_modelview", DataType::MATRIX44);
    program.createUniform("u_shadow_mvp", DataType::MATRIX44);
}

void Camera::updateUniforms(GLProgram& program)
{
    static const Eigen::Matrix4f biasMatrix =
    (Eigen::Matrix4f() << 0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0, 0.5, 0.5, 0, 0, 0, 1.0).finished();
    
    Eigen::Matrix4f MVP;
    perspective_ = PerspectiveFromVision(intrinsic_, width_, height_, zNear_, zFar_);
    
    MVP = perspective_ * extrinsic_;
    
    Eigen::Matrix4f shadowMVP = biasMatrix * MVP;
    
    program.setUniformData("u_mvp", MVP);
    program.setUniformData("u_modelview", extrinsic_);
    program.setUniformData("u_shadow_mvp", shadowMVP);
}

void Camera::updateUniforms(GLProgram& program, const Eigen::Matrix4f& RT)
{
    static const Eigen::Matrix4f biasMatrix =
    (Eigen::Matrix4f() << 0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0, 0.5, 0.5, 0, 0, 0, 1.0).finished();

    Eigen::Matrix4f MVP;
    perspective_ = PerspectiveFromVision(intrinsic_, width_, height_, zNear_, zFar_);

    Eigen::Matrix4f MV = extrinsic_ * RT;
    MVP = perspective_ * MV;
    
    Eigen::Matrix4f shadowMVP = biasMatrix * MVP;
    
    program.setUniformData("u_mvp", MVP);
    program.setUniformData("u_modelview", extrinsic_);
    program.setUniformData("u_shadow_mvp", shadowMVP);
}

