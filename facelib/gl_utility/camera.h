//
//  camera.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <fstream>
#include <iostream>
#include <memory>

#include <utility/EigenHelper.h>

#include "gl_core.h"

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

struct Camera;
typedef std::shared_ptr<Camera> CameraPtr;

// flags for attributes
static const int U_CAMERA_MVP    = 0x0001;
static const int U_CAMERA_MV     = 0x0002;
static const int U_CAMERA_WORLD  = 0x0004;
static const int U_CAMERA_SHADOW = 0x0008;
static const int U_CAMERA_POS    = 0x0010;

struct Camera
{
    std::string name_;
    
    Eigen::Matrix4f extrinsic_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f intrinsic_ = Eigen::Matrix4f::Identity();
    
    Eigen::VectorXf distCoeff_;
    
    float zNear_ = 1.f;
    float zFar_ = 400.f;
    int width_;
    int height_;
    
    bool weakPersp_ = false;
    
    Camera() {}
    Camera(const Eigen::Matrix4f& RT, const Eigen::Matrix4f& K, int w, int h, float zN, float zF, bool c2w = false);
    Camera(const Camera&);
    
    static void initializeUniforms(GLProgram& programs, int flag);
    void updateUniforms(GLProgram& program, int flag) const;
    void updateUniforms(GLProgram& program, Eigen::Matrix4f RT, int flag) const;
    
    void updateUniforms4Sphere(GLProgram& program, int flag) const;
    
    static Camera craeteFromFOV(int w, int h, int FOV);
    static Camera parseCameraParams(std::string filename, bool c2w = false);
    static Eigen::Matrix4f loadKFromTxt(std::string filename);
    static Eigen::Matrix4f loadRTFromTxt(std::string filename);
    
    friend std::ostream& operator<<(std::ostream& os, const Camera& cam)
    {
        os << "Camera Info: " << cam.name_ << std::endl;
        os << "Intrinsic:" << std::endl;
        os << cam.intrinsic_ << std::endl;
        os << "Extrinsic:" << std::endl;
        os << cam.extrinsic_ << std::endl;
        os << "Width: " << cam.width_ << " Height: " << cam.height_ << " zNear: " << cam.zNear_ << " zFar: " << cam.zFar_ << std::endl;
        os << "distCoeff: " << cam.distCoeff_.transpose();
        return os;
    }
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};
