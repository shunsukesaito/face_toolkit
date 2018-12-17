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
#include "trackball.h"

static double QuatD(double w, double h)
{
    using namespace std;
    return (std::abs(w) < std::abs(h) ? std::abs(w) : std::abs(h)) - 4;
}

static double QuatIX(double x, double w, double h)
{
    return (2.0*x - w - 1.0)/QuatD(w, h);
}

static double QuatIY(double y, double w, double h)
{
    return (-2.0*y + h - 1.0)/QuatD(w, h);
}

void trackball(const double w,
               const double h,
               const double speed_factor,
               const double down_mouse_x,
               const double down_mouse_y,
               const double mouse_x,
               const double mouse_y,
               Eigen::Quaternion<float>& quat)
{
    assert(speed_factor > 0);
    
    double original_x = QuatIX(speed_factor*(down_mouse_x-w/2)+w/2, w, h);
    double original_y = QuatIY(speed_factor*(down_mouse_y-h/2)+h/2, w, h);
    
    double x = QuatIX(speed_factor*(mouse_x-w/2)+w/2, w, h);
    double y = QuatIY(speed_factor*(mouse_y-h/2)+h/2, w, h);
    
    double z = 1;
    double n0 = sqrt(original_x*original_x + original_y*original_y + z*z);
    double n1 = sqrt(x*x + y*y + z*z);
    if(n0>1.e-8 && n1>1.e-8)
    {
        Eigen::Vector3d v0(original_x/n0, -original_y/n0, z/n0);
        Eigen::Vector3d v1(x/n1, -y/n1, z/n1);
        Eigen::Vector3d axis = v0.cross(v1);
        double sa = axis.norm();
        double ca = v0.dot(v1);
        double angle = atan2(sa, ca);
        if( x*x+y*y>1.0 )
        {
            angle *= 1.0 + 0.2f*(sqrt(x*x+y*y)-1.0);
        }
        Eigen::Matrix3f R = Eigen::AngleAxisf((float)angle,axis.normalized().cast<float>()).toRotationMatrix();
        quat = R;
    }
}
