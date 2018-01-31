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

