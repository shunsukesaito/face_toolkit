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
#include "basic_gradient.h"

void gradRotEuler(float rx, float ry, float rz, Eigen::Matrix3f dR[3])
{
    // Warning: it's not aligned with wiki (https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions)
    // looks like there's weird convenction in euler angle in mLib
    // this part could be problematic when moving to other coordinate system

    float sPhi = sin(rx);
    float cPhi = cos(rx);
    float sTheta = sin(ry);
    float cTheta = cos(ry);
    float sPsi = sin(rz);
    float cPsi = cos(rz);
    
    dR[0] << 0, sPhi*sPsi + cPhi*sTheta*cPsi, cPhi*sPsi - sPhi*sTheta*cPsi,
    0, -sPhi*cPsi + cPhi*sTheta*sPsi, -cPhi*cPsi - sPhi*sTheta*sPsi,
    0, cPhi*cTheta, -sPhi*cTheta;
    dR[1] << -sTheta*cPsi, sPhi*cTheta*cPsi, cPhi*cTheta*cPsi,
    -sTheta*sPsi, sPhi*cTheta*sPsi, cPhi*cTheta*sPsi,
    -cTheta, -sPhi*sTheta, -cPhi*sTheta;
    dR[2] << -cTheta*sPsi, -cPhi*cPsi - sPhi*sTheta*sPsi, sPhi*cPsi - cPhi*sTheta*sPsi,
    cTheta*cPsi, -cPhi*sPsi + sPhi*sTheta*cPsi, sPhi*sPsi + cPhi*sTheta*cPsi,
    0, 0, 0;
}

void gradV(Eigen::Ref<Eigen::MatrixXf> dout,
           const Eigen::MatrixXf& din,
           const Eigen::MatrixXf& dv)
{
    assert(dv.rows() == din.cols());
    assert(dout.rows() == din.rows());
    assert(dout.cols() == dv.cols());
    if(din.size() != 0)
        dout = din * dv;
    else
        dout = dv;
}

void gradROT(Eigen::Ref<Eigen::MatrixXf> dout,
             const Eigen::MatrixXf& din,
             const Eigen::Matrix3f dR[3],
             const Eigen::Vector3f& v)
{
    assert(dout.rows() == din.rows());
    assert(dout.cols() == 3);
    
    if(din.size() != 0){
        dout.col(0) = din * dR[0] * v;
        dout.col(1) = din * dR[1] * v;
        dout.col(2) = din * dR[2] * v;
    }
    else{
        dout.col(0) = dR[0] * v;
        dout.col(1) = dR[1] * v;
        dout.col(2) = dR[2] * v;
    }
}

void gradTR(Eigen::Ref<Eigen::MatrixXf> dout,
            const Eigen::MatrixXf& din)
{
    assert(dout.rows() == din.rows());
    assert(dout.cols() == 3);
    
    if(din.size() != 0)
        dout = din * Eigen::Matrix3f::Identity();
    else
        dout = Eigen::Matrix3f::Identity();
}

void gradCAM(Eigen::Ref<Eigen::MatrixXf> dout,
             const Eigen::MatrixXf& din,
             const Eigen::Vector3f& v,
             unsigned int size)
{
    assert(dout.rows() == din.rows());
    assert(din.cols() == 3);
    
    switch (size)
    {
        case 1:
            dout.col(0) = v(0)*din.col(0) + v(1)*din.col(1);
            break;
        case 2:
            dout.col(0) = v(0)*din.col(0);
            dout.col(1) = v(1)*din.col(1);
            break;
        case 3:
            dout.col(0) = v(0)*din.col(0) + v(1)*din.col(1);
            dout.col(1) = v(2)*din.col(0);
            dout.col(2) = v(2)*din.col(1);
            break;
        case 4:
            dout.col(0) = v(0)*din.col(0);
            dout.col(1) = v(1)*din.col(1);
            dout.col(2) = v(2)*din.col(0);
            dout.col(3) = v(2)*din.col(1);
            break;
        case 0:
            break;
    }
}

void gradF(Eigen::Ref<Eigen::RowVectorXf> result,
           const Eigen::Vector3f& v1_v0,
           const Eigen::Vector3f& v2_v0,
           const Eigen::Matrix3Xf& i1_i0,
           const Eigen::Matrix3Xf& i2_i0,
           const Eigen::Matrix3Xf& e1_e0,
           const Eigen::Matrix3Xf& e2_e0,
           const int index0,
           const int index1,
           const int nid,
           const int nex)
{
    for (int i = 0; i < nid; ++i)
    {
        result[i] = v1_v0[index0] * i2_i0.bc3(i)[index1] + i1_i0.bc3(i)[index0] * v2_v0[index1]
        - v1_v0[index1] * i2_i0.bc3(i)[index0] - i1_i0.bc3(i)[index1] * v2_v0[index0];
    }
    for (int i = 0; i < nex; ++i)
    {
        result[nex + i] = v1_v0[index0] * e2_e0.bc3(i)[index1] + e1_e0.bc3(i)[index0] * v2_v0[index1]
        - v1_v0[index1] * e2_e0.bc3(i)[index0] - e1_e0.bc3(i)[index1] * v2_v0[index0];
    }
}
