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
#include <vector>

#include <utility/EigenHelper.h>

// point-to-point 2d constraint
struct P2P2DC
{
    int v_idx; // index on the mesh
    int idx; // index on the landmarks
    
    P2P2DC(int _v_idx, int _idx) : v_idx(_v_idx), idx(_idx){};
    
    static void getIndexList(const std::vector<P2P2DC>& C,
                             std::vector<int>& idxs);
    static void updateConstraints(const std::vector<P2P2DC>& C,
                                  const std::vector<Eigen::Vector3f>& qinV,
                                  std::vector<Eigen::Vector3f>& qoutV);
    static void parseConstraints(std::string file_path,
                                 std::vector<P2P2DC>& C);
};

// point-to-point 3d constraint
struct P2P3DC
{
    int v_idx; // index on the mesh
    int idx; // index on point cloud
    
    P2P3DC(int _v_idx, int _idx) : v_idx(_v_idx), idx(_idx){};
    
    static void getIndexList(const std::vector<P2P3DC>& C,
                             std::vector<int>& idxs);
    static void updateConstraints(const std::vector<P2P3DC>& C,
                                  const std::vector<Eigen::Vector4f>& qinV,
                                  std::vector<Eigen::Vector4f>& qoutV);
    static void parseConstraints(std::string file_path,
                                 std::vector<P2P3DC>& C);
};

// point-to-line 2d constraint
struct P2L2DC
{
    int v_idx; // index on the mesh
    int start_idx; // starting index on the landmarks
    int end_idx; // ending index on the landmarks
    
    P2L2DC(int _v_idx, int _s_idx, int _e_idx) : v_idx(_v_idx), start_idx(_s_idx), end_idx(_e_idx){};
    
    static void getIndexList(const std::vector<P2L2DC>& C,
                             std::vector<int>& idxs);
    static void updateConstraints(const std::vector<P2L2DC>& C,
                                  const std::vector<Eigen::Vector3f>& qinV,
                                  const std::vector<Eigen::Vector2f>& pV,
                                  std::vector<Eigen::Vector3f>& qoutV,
                                  std::vector<Eigen::Vector2f>& nV);
    static void parseConstraints(std::string file_path,
                                 std::vector<P2L2DC>& C);
};

float computeJacobianPoint2Point3D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const std::vector<Eigen::Vector3f>& pV,
                                   const std::vector<Eigen::Matrix3Xf>& dpV,
                                   const std::vector<Eigen::Vector4f>& qV,
                                   const float& w,
                                   bool robust);

float computeJacobianPoint2Plane3D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const std::vector<Eigen::Vector3f>& pV,
                                   const std::vector<Eigen::Matrix3Xf>& dpV,
                                   const std::vector<Eigen::Vector4f>& qV,
                                   const std::vector<Eigen::Vector3f>& nV,
                                   const float& w,
                                   bool robust);

float computeJacobianPoint2Point2D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const std::vector<Eigen::Vector2f>& pV,
                                   const std::vector<Eigen::Matrix2Xf>& dpV,
                                   const std::vector<Eigen::Vector3f>& qV,
                                   const float& w,
                                   bool robust);

float computeJacobianPoint2Point2D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const std::vector<Eigen::Vector2f>& pV,
                                   const std::vector<Eigen::Matrix2Xf>& dpV,
                                   const std::vector<Eigen::Vector3f>& qV,
                                   const float& w,
                                   bool robust,
                                   std::vector<int>& idx);

float computeJacobianPoint2Line2D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                  Eigen::Ref<Eigen::MatrixXf> JtJ,
                                  const std::vector<Eigen::Vector2f>& pV,
                                  const std::vector<Eigen::Matrix2Xf>& dpV,
                                  const std::vector<Eigen::Vector3f>& qV,
                                  const std::vector<Eigen::Vector2f>& nV,
                                  const float& w,
                                  bool robust);

float computeJacobianPoint2Line2D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                  Eigen::Ref<Eigen::MatrixXf> JtJ,
                                  const std::vector<Eigen::Vector2f>& pV,
                                  const std::vector<Eigen::Matrix2Xf>& dpV,
                                  const std::vector<Eigen::Vector3f>& qV,
                                  const std::vector<Eigen::Vector2f>& nV,
                                  const float& w,
                                  bool robust,
                                  std::vector<int>& idx);

