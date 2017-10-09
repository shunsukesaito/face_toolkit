#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "EigenHelper.h"

struct DOF
{
    int ID = 0;
    int EX = 0;
    int AL = 0;
    int fROT = 0;
    int fTR = 0;
    int cROT = 0;
    int cTR = 0;
    int CAM = 0;
    int SH = 0;
    
    DOF() : ID(0), EX(0), AL(0), fROT(0), fTR(0), cROT(0), cTR(0), CAM(0), SH(0){};
    
    DOF(int _ID,
        int _EX,
        int _AL,
        int _fROT,
        int _fTR,
        int _cROT,
        int _cTR,
        int _CAM,
        int _SH,
        int _N_CAM = 1) :
    ID(_ID), EX(_EX), AL(_AL), fROT(_fROT), fTR(_fTR), cROT(_cROT), cTR(_cTR), CAM(_CAM), SH(_SH){};
    
    inline int all() const {
        return ID + EX + AL + fROT + fTR + cROT + cTR + CAM + SH;
    }
    inline int face() const {
        return ID + EX + AL + fROT + fTR + SH;
    }
    inline int camera() const {
        return cROT + cTR + CAM;
    }
};

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

