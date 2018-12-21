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

#include "ray.h"

bool Ray::rayTriangle(Ray& ray, const Triangle& f)
{
    // if ray and triangle are facing in the same direction, don't count interection
    if(ray.dir_.dot(f.n_) > 1.e-6)
        return false;
    
    Eigen::Vector3f e1 = f.v1_ - f.v0_;
    Eigen::Vector3f e2 = f.v2_ - f.v0_;
    
    Eigen::Vector3f cross = ray.dir_.cross(e2);
    
    float dot = e1.dot(cross);
    
    if(fabs(dot) < 1.e-10)
        return false;
    
    float inv = 1.0f/dot;
    
    Eigen::Vector3f diff = ray.o_ - f.v0_;
    
    float u = diff.dot(cross) * inv;
    if( u < 0.0f || u > 1.0f)
        return false;
    
    Eigen::Vector3f q = diff.cross(e1);
    
    float v = ray.dir_.dot(q) * inv;
    if( v < 0.0f || v > 1.0f)
        return false;
    
    float t = e2.dot(q) * inv;
    if( t <= 0.0f)
        return false;
    
    if( t < ray.t_)
    {
        ray.t_ = t;
        ray.idx_ = f.idx_;
    }
    
    return true;
}
