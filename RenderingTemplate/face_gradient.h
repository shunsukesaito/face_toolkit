#pragma once

#include <iostream>
#include <vector>


#include <opencv2/opencv.hpp>

#include "EigenHelper.h"
#include "face_model.hpp"
#include "camera.hpp"

//#define DUMP_JACOBIAN_DATA
//#define ALBEDO_MODE

/*****************************************************************************************************************
jacobian memo
y = f/g - m_objectiveNormal;
y' = (f'g-fg') / g^2

f/g = |  n0  |
|  n1  |
|  n2  |
f =   |  n0  |  =  |    (v1[1]-v0[1])*(v2[2]-v0[2]) - (v1[2]-v0[2])*(v2[1]-v0[1])    |
|  n1  |     |    (v1[2]-v0[2])*(v2[0]-v0[0]) - (v1[0]-v0[0])*(v2[2]-v0[2])    |
|  n2  |     |    (v1[0]-v0[0])*(v2[1]-v0[1]) - (v1[1]-v0[1])*(v2[0]-v0[0])    |

g = sqrt(((v1[1]-v0[1])*(v2[2]-v0[2]) - (v1[2]-v0[2])*(v2[1]-v0[1])) * ((v1[1]-v0[1])*(v2[2]-v0[2]) - (v1[2]-v0[2])*(v2[1]-v0[1]))
+((v1[2]-v0[2])*(v2[0]-v0[0]) - (v1[0]-v0[0])*(v2[2]-v0[2])) * ((v1[2]-v0[2])*(v2[0]-v0[0]) - (v1[0]-v0[0])*(v2[2]-v0[2]))
+((v1[0]-v0[0])*(v2[1]-v0[1]) - (v1[1]-v0[1])*(v2[0]-v0[0])) * ((v1[0]-v0[0])*(v2[1]-v0[1]) - (v1[1]-v0[1])*(v2[0]-v0[0])))

v1[0]-v0[0] = (m_neutral1[0] + sum(iCoeff*m_identity1[0]) + sum(eCoeff*m_expression1[0]) -
(m_neutral0[0] + sum(iCoeff*m_identity0[0]) + sum(eCoeff*m_expression0[0]);
v1[1]-v0[1] = (m_neutral1[1] + sum(iCoeff*m_identity1[1]) + sum(eCoeff*m_expression1[1]) -
(m_neutral0[1] + sum(iCoeff*m_identity0[1]) + sum(eCoeff*m_expression0[1]);
v1[2]-v0[2] = (m_neutral1[2] + sum(iCoeff*m_identity1[2]) + sum(eCoeff*m_expression1[2]) -
(m_neutral0[2] + sum(iCoeff*m_identity0[2]) + sum(eCoeff*m_expression0[2]);
v2[0]-v0[0] = (m_neutral2[0] + sum(iCoeff*m_identity2[0]) + sum(eCoeff*m_expression2[0]) -
(m_neutral0[0] + sum(iCoeff*m_identity0[0]) + sum(eCoeff*m_expression0[0]);
v2[1]-v0[1] = (m_neutral2[1] + sum(iCoeff*m_identity2[1]) + sum(eCoeff*m_expression2[1]) -
(m_neutral0[1] + sum(iCoeff*m_identity0[1]) + sum(eCoeff*m_expression0[1]);
v2[2]-v0[2] = (m_neutral2[2] + sum(iCoeff*m_identity2[2]) + sum(eCoeff*m_expression2[2]) -
(m_neutral0[2] + sum(iCoeff*m_identity0[2]) + sum(eCoeff*m_expression0[2]);
******************************************************************************************************************/


struct TriPoint
{
	TriPoint(const Eigen::Vector3i& _idx, const Eigen::Vector3f& _ust, float _w) : idx(_idx), ust(_ust), w(_w){};

	Eigen::Vector3i idx;
	Eigen::Vector3f ust;
	float w;
};

struct DOF
{
    unsigned int ID = 0;
    unsigned int EX = 0;
    unsigned int AL = 0;
    unsigned int ROT = 0;
    unsigned int TR = 0;
    unsigned int CAM = 0;
    unsigned int SH = 0;
    
    DOF() : ID(0), EX(0), AL(0), ROT(0), TR(0), CAM(0), SH(0){};
    
    DOF(unsigned int _ID,
        unsigned int _EX,
        unsigned int _AL,
        unsigned int _ROT,
        unsigned int _TR,
        unsigned int _CAM,
        unsigned int _SH,
        unsigned int _N_CAM = 1) :
    ID(_ID), EX(_EX), AL(_AL), ROT(_ROT), TR(_TR), CAM(_CAM), SH(_SH){};
    
    inline unsigned int all() const {
        return ID + EX + AL + ROT + TR + CAM + SH;
    }
};

void jacobianID(Eigen::MatrixX2f& result,
                const Eigen::Vector3f& pp,
                const float& zsq,
                const Eigen::Matrix3Xf& id,
                const Eigen::Matrix4f& I,
                const Eigen::Matrix3f& R,
                unsigned int start,
                unsigned int size);

void jacobianEX(Eigen::MatrixX2f& result,
                const Eigen::Vector3f& pp,
                const float& zsq,
                const Eigen::Matrix3Xf& ex,
                const Eigen::Matrix4f& I,
                const Eigen::Matrix3f& R,
                unsigned int start,
                unsigned int size);

void jacobianROT(Eigen::MatrixX2f& result,
                 const Eigen::Vector3f& pp,
                 const Eigen::Vector3f& p3,
                 const float& zsq,
                 const Eigen::Matrix4f& I,
                 const std::vector<Eigen::Matrix3f>& RdRs,
                 unsigned int start,
                 unsigned int size);

void jacobianTR(Eigen::MatrixX2f& result,
                const Eigen::Vector3f& pp,
                const Eigen::Vector3f& p3,
                const float& zsq,
                const Eigen::Matrix4f& I,
                const Eigen::Matrix4f& RTc,
                unsigned int start,
                unsigned int size);

void jacobianCAM(Eigen::MatrixX2f& result,
                 const Eigen::Vector3f& pp,
                 const float& z,
                 const float& scale,
                 unsigned int start,
                 unsigned int size);

void computeJacobiansP2D(Eigen::MatrixX2f& result,
                         Eigen::Vector2f& p2,
                         const Eigen::Vector3f& p3,
                         const Eigen::Matrix3Xf& id,
                         const Eigen::Matrix3Xf& ex,
                         const Eigen::Matrix4f& RTall,//RTCam*RTFace
                         const Eigen::Matrix4f& RTc,
                         const std::vector<Eigen::Matrix3f>& RdRs,
                         const Eigen::Matrix4f& I,
                         const DOF& dof);

void computeJacobiansP3D(Eigen::MatrixX3f& result,
                         const Eigen::Vector3f& p3,
                         const Eigen::Matrix3Xf& id,
                         const Eigen::Matrix3Xf& ex,
                         const Eigen::Matrix4f& RTf,
                         const std::vector<Eigen::Matrix3f>& dRs,
                         const DOF& dof);

void computeJacobiansF(Eigen::VectorXf& result,
                       const Eigen::Vector3f& v1_v0,
                       const Eigen::Vector3f& v2_v0,
                       const Eigen::Matrix3Xf& i1_i0,
                       const Eigen::Matrix3Xf& i2_i0,
                       const Eigen::Matrix3Xf& e1_e0,
                       const Eigen::Matrix3Xf& e2_e0,
                       const int index0,
                       const int index1,
                       const DOF& dof);

float computeJacobianPCAReg(Eigen::VectorXf& Jtr,
                            Eigen::MatrixXf& JtJ,
                            const Eigen::VectorXf& X,
                            const Eigen::VectorXf& sigma,
                            unsigned int start,
                            unsigned int size,
                            const float& w);

float computeJacobianL1Reg(Eigen::VectorXf& Jtr,
                           Eigen::MatrixXf& JtJ,
                           const Eigen::VectorXf& X,
                           const Eigen::VectorXf& sigma,
                           unsigned int start,
                           unsigned int size,
                           const float& w);

float computeJacobianL2Reg(Eigen::VectorXf& Jtr,
                           Eigen::MatrixXf& JtJ,
                           const Eigen::VectorXf& X,
                           const Eigen::VectorXf& sigma,
                           unsigned int start,
                           unsigned int size,
                           const float& w);

float computeJacobianLMixReg(Eigen::VectorXf& Jtr,
                             Eigen::MatrixXf& JtJ,
                             const Eigen::VectorXf& X,
                             const Eigen::VectorXf& sigma,
                             unsigned int start,
                             unsigned int size,
                             const float& w);

float computeJacobianTikhonovReg(Eigen::VectorXf& Jtr,
                                 Eigen::MatrixXf& JtJ,
                                 const Eigen::VectorXf& X,
                                 const Eigen::VectorXf& X0,
                                 unsigned int start,
                                 unsigned int size,
                                 const float w);

// to be removed
void computeCorresContourLand2D(std::vector<TriPoint>& pBary,
                                const std::vector<Eigen::Vector3f>& lands,
                                const cv::Mat_<cv::Vec4f>& normal,
                                const cv::Mat_<cv::Vec4f>& baryc,
                                const cv::Mat_<cv::Vec4f>& triIdx,
                                unsigned int level);

// to be removed
void computeJacobianContour(Eigen::VectorXf& Jtr,
                            Eigen::MatrixXf& JtJ,
                            const std::vector<Eigen::Vector2f>& pV,
                            const std::vector<Eigen::MatrixX2f>& dpV,
                            const std::vector<TriPoint>& pBary,
                            const std::vector<Eigen::Vector3f>& land,
                            float w);

void computeJacobianSymmetry(Eigen::VectorXf& Jtr,
                             Eigen::MatrixXf& JtJ,
                             const Eigen::VectorXf& id_delta,
                             const Eigen::VectorXf& ex_delta,
                             const Eigen::MatrixXf& w_id,
                             const Eigen::MatrixXf& w_exp,
                             const std::vector< int > sym_list,
                             const DOF& dof,
                             const float& w,
                             bool withexp);

void computeJacobianPairClose(Eigen::VectorXf& Jtr,
                              Eigen::MatrixXf& JtJ,
                              const Eigen::VectorXf& V,
                              const Eigen::MatrixXf& w_id,
                              const Eigen::MatrixXf& w_exp,
                              const std::vector< int >& index_list,
                              const DOF& dof,
                              const float w);

float computeJacobianPoint2Point3D(Eigen::VectorXf& Jtr,
                                   Eigen::MatrixXf& JtJ,
                                   const std::vector<Eigen::Vector4f>& pV,
                                   const std::vector<Eigen::MatrixX3f>& dpV,
                                   const std::vector<Eigen::Vector3f>& qV,
                                   const float& w,
                                   bool robust);

float computeJacobianPoint2Plane3D(Eigen::VectorXf& Jtr,
                                   Eigen::MatrixXf& JtJ,
                                   const std::vector<Eigen::Vector4f>& pV,
                                   const std::vector<Eigen::MatrixX3f>& dpV,
                                   const std::vector<Eigen::Vector3f>& qV,
                                   const std::vector<Eigen::Vector3f>& nV,
                                   const float& w,
                                   bool robust);

float computeJacobianPoint2Point2D(Eigen::VectorXf& Jtr,
                                   Eigen::MatrixXf& JtJ,
                                   const std::vector<Eigen::Vector3f>& pV,
                                   const std::vector<Eigen::MatrixX2f>& dpV,
                                   const std::vector<Eigen::Vector2f>& qV,
                                   const float& w,
                                   bool robust);

float computeJacobianPoint2Line2D(Eigen::VectorXf& Jtr,
                                  Eigen::MatrixXf& JtJ,
                                  const std::vector<Eigen::Vector3f>& pV,
                                  const std::vector<Eigen::MatrixX2f>& dpV,
                                  const std::vector<Eigen::Vector2f>& qV,
                                  const std::vector<Eigen::Vector2f>& nV,
                                  const float& w,
                                  bool robust);

void computeVertexWiseNormalTerm(std::vector<Eigen::Vector3f>& nV,
                                 std::vector<Eigen::MatrixXf>& dnV,
                                 const Eigen::VectorXf& V,
                                 const Eigen::MatrixX3f& tri,
                                 const std::vector<std::array<Eigen::Matrix3Xf, 2>>& id_edge,
                                 const std::vector<std::array<Eigen::Matrix3Xf, 2>>& ex_edge,
                                 const DOF& dof);

void computeVertexWisePositionGradient2D(std::vector<Eigen::Vector2f>& pV,
                                         std::vector<Eigen::MatrixX2f>& dpV,
                                         const Eigen::VectorXf& V,
                                         const Eigen::MatrixXf& w_id,
                                         const Eigen::MatrixXf& w_exp,
                                         const Eigen::Matrix4f& RTc,
                                         const Eigen::Vector6f& rt,
                                         const Eigen::Matrix4f& I,
                                         const DOF& dof,
                                         const std::vector<int>& vert_list);

void computeVertexWisePositionGradient3D(std::vector<Eigen::Vector3f>& pV,
                                         std::vector<Eigen::MatrixX3f>& dpV,
                                         const Eigen::VectorXf& V,
                                         const Eigen::MatrixXf& w_id,
                                         const Eigen::MatrixXf& w_exp,
                                         const Eigen::Vector6f& rt,
                                         const DOF& dof,
                                         const std::vector<int>& vert_list);

void setFaceVector(Eigen::VectorXf& X,
                   std::vector<Eigen::Matrix4f>& Is,
                   Eigen::Vector6f& rt,
                   const FaceParams& param,
                   const std::vector<Camera>& cameras,
                   const DOF& dof);

void setFaceVector(Eigen::VectorXf& X,
                   Eigen::Matrix4f &I,
                   Eigen::Vector6f& rt,
                   const FaceParams& param,
                   const Camera& camera,
                   const DOF& dof);

void loadFaceVector(const Eigen::VectorXf& X,
                    std::vector<Eigen::Matrix4f>& Is,
                    Eigen::Vector6f& rt,
                    FaceParams& param,
                    std::vector<Camera>& cameras,
                    const DOF& dof);

void loadFaceVector(const Eigen::VectorXf& X,
                    Eigen::Matrix4f& I,
                    Eigen::Vector6f& rt,
                    FaceParams& param,
                    Camera& camera,
                    const DOF& dof);
