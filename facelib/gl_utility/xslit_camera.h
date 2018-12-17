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

struct XSlitCamera;
typedef std::shared_ptr<XSlitCamera> XSlitCameraPtr;

// flags for attributes
static const int U_XSCAMERA_MV    = 0x0001;

void createCircleXSlits(float radius, const Eigen::Vector3f& q1, const Eigen::Vector3f& q2, int nCam,
                        int width, int height, std::vector<XSlitCamera>& cameras);

struct XSlitCamera
{
    std::string name_;
  
    Eigen::Vector3f p1_, p2_; // first slit two end-points
    Eigen::Vector3f q1_, q2_; // second slit two end-points
    
    float zNear_ = 0.1f; // normalized
    float zFar_ = 2.0f; // normalized
    int width_ = 100;
    int height_ = 100;
    
    XSlitCamera() {}
    XSlitCamera(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
                const Eigen::Vector3f& q1, const Eigen::Vector3f& q2, float zNear, float zFar, int w, int h);
    XSlitCamera(const XSlitCamera&);
    
    void updateTransform(Eigen::Matrix4f& T, Eigen::Vector2f& d1, Eigen::Vector2f& d2, Eigen::Vector2f& d3) const;
    
    static void initializeUniforms(GLProgram& programs, int flag);
    void updateUniforms(GLProgram& program, int flag) const;
    void updateUniforms(GLProgram& program, Eigen::Matrix4f RT, int flag) const;
    
    void updateUniforms4Sphere(GLProgram& program, int flag) const;
    
    static XSlitCamera parseCameraParams(std::string filename);
    
    friend std::ostream& operator<<(std::ostream& os, const XSlitCamera& cam)
    {
        os << "Camera Info: " << cam.name_ << std::endl;
        os << "P1:" << std::endl;
        os << cam.p1_.transpose() << std::endl;
        os << "P2:" << std::endl;
        os << cam.p2_.transpose() << std::endl;
        os << "Q1:" << std::endl;
        os << cam.q1_.transpose() << std::endl;
        os << "Q2:" << std::endl;
        os << cam.q2_.transpose() << std::endl;
        os << " zNear: " << cam.zNear_ << " zFar: " << cam.zFar_ << std::endl;
        return os;
    }
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};
