#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "EigenHelper.h"

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
