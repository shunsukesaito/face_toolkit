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

struct GLCCamera;
typedef std::shared_ptr<GLCCamera> GLCCameraPtr;

// flags for attributes
static const int U_GLCCAMERA_MV    = 0x0001;

void createGLCFromSphere(float radius, unsigned int rings, unsigned int sectors, const Eigen::Vector3f& p,
                         float zN, float zF, int width, int height, std::vector<GLCCamera>& cameras);

struct GLCCamera
{
    std::string name_;
  
    Eigen::Vector3f p1_, p2_, p3_; // image plane triangle (world space)
    Eigen::Vector2f uv1_, uv2_, uv3_; // image plane triangle (screen space)
    Eigen::Vector3f q1_, q2_, q3_; // target plane triangle
    
    float zNear_ = 0.1f; // world scale
    float zFar_ = 2.0f; // world scale
    int width_ = 100;
    int height_ = 100;
    
    GLCCamera() {}
    GLCCamera(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3,
              const Eigen::Vector2f& uv1, const Eigen::Vector2f& uv2, const Eigen::Vector2f& uv3,
              const Eigen::Vector3f& q1, const Eigen::Vector3f& q2, const Eigen::Vector3f& q3,
              float zNear, float zFar, int w, int h);
    GLCCamera(const GLCCamera&);
    
    void updateTransform(Eigen::Matrix4f& T, Eigen::Matrix2f& d1, Eigen::Matrix2f& d2, Eigen::Matrix2f& d3) const;
    
    static void initializeUniforms(GLProgram& programs, int flag);
    void updateUniforms(GLProgram& program, int flag) const;
    void updateUniforms(GLProgram& program, Eigen::Matrix4f RT, int flag) const;
    
    void updateUniforms4Sphere(GLProgram& program, int flag) const;
    
    static GLCCamera parseCameraParams(std::string filename);
    
    friend std::ostream& operator<<(std::ostream& os, const GLCCamera& cam)
    {
        os << "Camera Info: " << cam.name_ << std::endl;
        os << " zNear: " << cam.zNear_ << " zFar: " << cam.zFar_ << std::endl;
        os << "P1: ";
        os << cam.p1_.transpose() << " uv: " << cam.uv1_.transpose() << std::endl;
        os << "P2: ";
        os << cam.p2_.transpose() << " uv: " << cam.uv2_.transpose() << std::endl;
        os << "P3: ";
        os << cam.p3_.transpose() << " uv: " << cam.uv3_.transpose() << std::endl;
        os << "Q1: ";
        os << cam.q1_.transpose() << std::endl;
        os << "Q2: ";
        os << cam.q2_.transpose() << std::endl;
        os << "Q3: ";
        os << cam.q3_.transpose() << std::endl;
        Eigen::Matrix4f T;
        Eigen::Matrix2f d1, d2, d3;
        cam.updateTransform(T, d1, d2, d3);
        os << "T: " << std::endl;
        os << T << std::endl;
        os << "d1: " << std::endl;
        os << d1 << std::endl;
        os << "d2: " << std::endl;
        os << d2 << std::endl;
        os << "d3: " << std::endl;
        os << d3 << std::endl;
        return os;
    }
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};
