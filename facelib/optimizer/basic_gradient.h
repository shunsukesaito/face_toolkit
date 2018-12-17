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
#include <vector>

#include <utility/EigenHelper.h>

void gradRotEuler(float rx, float ry, float rz, Eigen::Matrix3f dR[3]);

void gradV(Eigen::Ref<Eigen::MatrixXf> dout,
           const Eigen::MatrixXf& din,
           const Eigen::MatrixXf& dv);

void gradROT(Eigen::Ref<Eigen::MatrixXf> dout,
             const Eigen::MatrixXf& din,
             const Eigen::Matrix3f dR[3],
             const Eigen::Vector3f& v);

void gradTR(Eigen::Ref<Eigen::MatrixXf> dout, const Eigen::MatrixXf& din);

void gradCAM(Eigen::Ref<Eigen::MatrixXf> dout,
             const Eigen::MatrixXf& din,
             const Eigen::Vector3f& v,
             unsigned int size);

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
           const int nex);
