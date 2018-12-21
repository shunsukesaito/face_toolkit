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

#include <iostream>
#include <memory>

#include <Eigen/Dense>

struct Triangle;
typedef std::shared_ptr<Triangle> TrianglePtr;

struct Triangle
{
    Eigen::Vector3f v0_, v1_, v2_;
    Eigen::Vector3f n_;
    int idx_ = -1;
    
    Triangle(){}
    Triangle(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, int idx)
    : v0_(v0), v1_(v1), v2_(v2), idx_(idx){ n_ = (v1_-v0_).cross(v2_-v0_); n_.normalize(); }
};

struct Ray
{
    Eigen::Vector3f o_; // origin
    Eigen::Vector3f dir_; // direction
    Eigen::Vector3f inv_;
    
    float tmin_;
    float tmax_;
    
    float t_; // interection param
    int idx_; // interection index
    
    Ray(){}
    Ray(const Eigen::Vector3f& o, const Eigen::Vector3f& d, float tmin=0.0f, float tmax=1.e10f)
    : o_(o), dir_(d), tmin_(tmin), tmax_(tmax)
    {
        normalize();
    }

    inline void normalize()
    {
        inv_ = dir_.array().inverse();
        
        t_ = 1.e10f;
        float l = dir_.norm();
        
        if(fabs(l) <= 1.e-10f)
            return;
        
        dir_ /= l;
    }
    
    static bool rayTriangle(Ray& ray, const Triangle& f);
};
