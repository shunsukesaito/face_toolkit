#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "base_gradient.h"
#include "regularizer.h"
#include "constraints.h"
#include "face_model.h"
#include "camera.h"

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
    inline int pos() const {
        return ID + EX + fROT + fTR + cROT + cTR + CAM;
    }
    inline int shape() const {
        return ID + EX;
    }
    inline int face() const {
        return ID + EX + AL + fROT + fTR + SH;
    }
    inline int camera() const {
        return cROT + cTR + CAM;
    }
    inline int tinvar() const {
        return ID + AL + cROT + cTR + CAM;
    }
    inline int ftinvar() const {
        return ID + AL;
    }
    inline int tvar() const {
        return EX + fROT + fTR + SH;
    }
};

void computeV(const FaceData& fd,
              const std::vector<int>& idx,
              std::vector<Eigen::Vector3f>& V);

void computeVertexWiseGradPosition2D(std::vector<Eigen::Vector2f>& pV,
                                     std::vector<Eigen::Matrix2Xf>& dpV,
                                     const FaceData& fd,
                                     const Eigen::Vector6f& rtc,
                                     const Eigen::Vector6f& rtf,
                                     const Eigen::Matrix4f& I,
                                     const DOF& dof,
                                     const std::vector<int>& vert_list = std::vector<int>());

void computeVertexWiseGradPosition2D(std::vector<Eigen::Vector2f>& pV,
                                     std::vector<Eigen::Matrix2Xf>& dpV,
                                     const std::vector<Eigen::Vector3f>& V,
                                     const FaceData& fd,
                                     const Eigen::Vector6f& rtc,
                                     const Eigen::Vector6f& rtf,
                                     const Eigen::Matrix4f& I,
                                     const DOF& dof,
                                     const std::vector<int>& vert_list);

void computeVertexWiseGradPosition3D(std::vector<Eigen::Vector3f>& pV,
                                     std::vector<Eigen::Matrix3Xf>& dpV,
                                     const FaceData& fd,
                                     const Eigen::Vector6f& rt,
                                     const DOF& dof,
                                     const std::vector<int>& vert_list);

void computeVertexWiseGradNormal(std::vector<Eigen::Vector3f>& nV,
                                 std::vector<Eigen::Matrix3Xf>& dnV,
                                 const FaceData& fd,
                                 const DOF& dof);

float computeJacobianSymmetry(Eigen::Ref<Eigen::VectorXf> Jtr,
                              Eigen::Ref<Eigen::MatrixXf> JtJ,
                              const FaceData& fd,
                              const DOF& dof,
                              const float& w,
                              bool withexp);

float computeJacobianPoint2Point3D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const FaceData& fd,
                                   const std::vector< int >& index_list,
                                   const DOF& dof,
                                   const float w);

void setFaceVector(Eigen::Ref<Eigen::VectorXf> X,
                   Eigen::Vector6f& rt,
                   const FaceData& fd,
                   const DOF& dof);

void setCameraVector(Eigen::Ref<Eigen::VectorXf> X,
                     Eigen::Vector6f& rt,
                     const Camera& camera,
                     const DOF& dof);

void loadFaceVector(const Eigen::VectorXf& X,
                    Eigen::Vector6f& rt,
                    FaceData& fd,
                    const DOF& dof);

void loadCameraVector(const Eigen::VectorXf& X,
                      Eigen::Vector6f& rt,
                      Camera& camera,
                      const DOF& dof);
