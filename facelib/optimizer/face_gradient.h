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

#include <gl_utility/camera.h>
#include <shape_model/face_model.h>

#include "basic_gradient.h"
#include "regularizer.h"
#include "constraints.h"

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
    inline int tinv() const {
        return ID + AL + cROT + cTR + CAM;
    }
    inline int ftinv() const {
        return ID + AL;
    }
    inline int tvar() const {
        return EX + fROT + fTR + SH;
    }
    
    friend std::ostream& operator<<(std::ostream& os, const DOF& dof)
    {
        os << "ID: " << dof.ID << " EX: " << dof.EX << " AL: " << dof.AL << " fROT: " << dof.fROT << " fTR: " << dof.fTR;
        os << "cROT: " << dof.cROT << " cTR: " << dof.cTR << " SH: " << dof.SH << " CAM: " << dof.CAM;
        return os;
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
