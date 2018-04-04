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
    
    float zNear_ = 10.f;
    float zFar_ = 100.f;
    int width_;
    int height_;
    
    Camera() {}
    Camera(const Eigen::Matrix4f& RT, const Eigen::Matrix4f& K, int w, int h, float zN, float zF, bool c2w = false);
    Camera(const Camera&);
    
    static void initializeUniforms(GLProgram& programs, int flag);
    void updateUniforms(GLProgram& program, int flag) const;
    void updateUniforms(GLProgram& program, const Eigen::Matrix4f& RT, int flag) const;
    
    void updateUniforms4Sphere(GLProgram& program, int flag) const;
    
    static Camera craeteFromFOV(int w, int h, int FOV);
    static Camera parseCameraParams(std::string filename);
    static Eigen::Matrix4f loadKFromTxt(std::string filename);
    static Eigen::Matrix4f loadRTFromTxt(std::string filename);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};
