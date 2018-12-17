/*
 MIT License
 
 Copyright (c) 2018 Shunsuke Saito
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
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
    
    float zNear_ = 0.1f;
    float zFar_ = 400.f;
    int width_;
    int height_;
    
    bool weakPersp_ = false;
    
    Camera() {}
    Camera(const Eigen::Matrix4f& RT, const Eigen::Matrix4f& K, int w, int h, float zN, float zF, bool c2w = false);
    Camera(const Camera&);
    
    static void initializeUniforms(GLProgram& programs, int flag);
    
    Eigen::Matrix4f getMVP(Eigen::Matrix4f RT) const;
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
