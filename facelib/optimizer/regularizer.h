//
//  regularizer.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 8/25/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
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
                           const Eigen::VectorXf& sigma,
                           unsigned int start,
                           unsigned int size,
                           const float& w);

float computeJacobianL2Reg(Eigen::Ref<Eigen::VectorXf> Jtr,
                           Eigen::Ref<Eigen::MatrixXf> JtJ,
                           const Eigen::VectorXf& X,
                           const Eigen::VectorXf& sigma,
                           unsigned int start,
                           unsigned int size,
                           const float& w);

float computeJacobianLMixReg(Eigen::Ref<Eigen::VectorXf> Jtr,
                             Eigen::Ref<Eigen::MatrixXf> JtJ,
                             const Eigen::VectorXf& X,
                             const Eigen::VectorXf& sigma,
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
