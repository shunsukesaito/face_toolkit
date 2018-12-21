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

#include "bbox.h"

BBox::BBox(const Triangle& f)
{
    Eigen::Matrix3f A;
    A << f.v0_, f.v1_, f.v2_;
    
    v_[0] = A.colwise().minCoeff().transpose();
    v_[1] = A.rowwise().maxCoeff().transpose();
    
    this->setCenter();
}

BBox::BBox(const std::vector<Triangle>& tri)
{
    if(tri.size() == 0)
    {
        std::cerr << "BBox::Bbox - tri is not set properly" << std::endl;
    }
    v_[0] = tri[0].v0_;
    v_[1] = tri[0].v0_;
    
    for(int i = 0; i < tri.size(); ++i)
    {
        BBox btemp(tri[i]);
        
        for(int j = 0; j < 3; ++j)
        {
            if(btemp.v_[0][j] < v_[0][j])
            {
                v_[0][j] = btemp.v_[0][j];
            }
            if(btemp.v_[1][j] > v_[1][j])
            {
                v_[1][j] = btemp.v_[1][j];
            }
        }
    }
    
    this->setCenter();
}

// the ray should be normalized first
bool BBox::rayIntersect(Ray &r)
{
    float tmin = 0.0f;
    float tmax = 1.e10f;
    
    for(int i = 0; i < 3; ++i)
    {
        int porN = (r.dir_[i] >= 0 )? 0:1;
        
        float t0 = (v_[porN][i] - r.o_[i]) * r.inv_[i];
        float t1 = (v_[1 - porN][i] - r.o_[i]) * r.inv_[i];
        
        if(t0 > tmin) tmin = t0;
        if(t1 < tmax) tmax = t1;
        if(tmin > tmax) return false;
    }
    
    return true;
}

void BBox::setCenter()
{
    c_ = 0.5f*(v_[0] + v_[1]);
}

float BBox::area()
{
    Eigen::Vector3f diff = v_[1] - v_[0];
    return 2.0f * (diff(0)*diff(1)+diff(1)*diff(2)+diff(0)*diff(2));
}

float BBox::volume()
{
    Eigen::Vector3f diff = v_[1] - v_[0];
    return diff(0) * diff(1) * diff(2);
}

BBox BBox::merge(BBox &b1, BBox &b2)
{
    BBox result;
    result.v_[0](0) = std::min(b1.v_[0](0), b2.v_[0](0));
    result.v_[0](1) = std::min(b1.v_[0](1), b2.v_[0](1));
    result.v_[0](2) = std::min(b1.v_[0](2), b2.v_[0](2));
    
    result.v_[1](0) = std::max(b1.v_[1](0),b2.v_[1](0));
    result.v_[1](1) = std::max(b1.v_[1](1),b2.v_[1](1));
    result.v_[1](2) = std::max(b1.v_[1](2),b2.v_[1](2));
    
    result.setCenter();
    
    return result;
};


