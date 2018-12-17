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

float computeJacobianPCAReg(Eigen::Ref<Eigen::VectorXf> Jtr,
                            Eigen::Ref<Eigen::MatrixXf> JtJ,
                            const Eigen::VectorXf& X,
                            const Eigen::VectorXf& sigma,
                            unsigned int start,
                            unsigned int size,
                            const float& w);

float computeJacobianL1Reg(Eigen::Ref<Eigen::VectorXf> Jtr,
                           Eigen::Ref<Eigen::MatrixXf> JtJ,
                           const Eigen::VectorXf& X,
                           unsigned int start,
                           unsigned int size,
                           const float& w);

float computeJacobianL2Reg(Eigen::Ref<Eigen::VectorXf> Jtr,
                           Eigen::Ref<Eigen::MatrixXf> JtJ,
                           const Eigen::VectorXf& X,
                           unsigned int start,
                           unsigned int size,
                           const float& w);

float computeJacobianLMixReg(Eigen::Ref<Eigen::VectorXf> Jtr,
                             Eigen::Ref<Eigen::MatrixXf> JtJ,
                             const Eigen::VectorXf& X,
                             float l,
                             float u,
                             float lambda,
                             unsigned int start,
                             unsigned int size,
                             const float& w);

float computeJacobianTikhonovReg(Eigen::Ref<Eigen::VectorXf> Jtr,
                                 Eigen::Ref<Eigen::MatrixXf> JtJ,
                                 const Eigen::VectorXf& X,
                                 const Eigen::VectorXf& X0,
                                 unsigned int start,
                                 unsigned int size,
                                 const float w);
