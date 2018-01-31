#pragma once

#include <utility/EigenHelper.h>
#include <gl_utility/camera.h>
#include <shape_model/face_model.h>

#include "face_gradient.h"

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

using namespace std;

struct P2DFitParams
{
    bool run_ = false;
    
    DOF dof = DOF(40, 20, 0, 3, 3, 0, 0, 0, 0);
    
    int maxIter_ = 10;
    
    bool robust_ = false;
    
    float gn_thresh_ = 1.0e-4f;
    float mclose_thresh_ = 4.0f;
    float angle_thresh_ = 8.0f;
    
    float w_p2p_ = 1.e-3f;
    float w_p2l_ = 1.e-3f;
    float w_reg_pca_id_ = 1.e-1f;
    float w_reg_pca_ex_ = 1.e-1f;
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

typedef std::shared_ptr<P2DFitParams> P2DFitParamsPtr;

bool RigidAlignment(const std::vector<Eigen::Vector3f> &q,
                    const std::vector<Eigen::Vector3f> &p,
                    Camera& camera);

void compute_rigid_motion(const Eigen::Matrix4f &intrinsic,
						  const std::vector<Eigen::Vector3f> &p3d,
						  const std::vector<Eigen::Vector3f> &q2d,
                          Eigen::Matrix4f& extrinsic);

void P2DGaussNewton(FaceData& fd,
                    Camera& camera,
                    const std::vector<P2P2DC>& CP2P,
                    std::vector<P2L2DC>& CP2L,
                    const std::vector<Eigen::Vector3f>& q2V,
                    const P2DFitParams& params = P2DFitParams());

void P2DGaussNewton(std::vector<FaceData>& fd,
                    Camera& camera,
                    const std::vector<P2P2DC>& CP2P,
                    std::vector<P2L2DC>& CP2L,
                    const std::vector<std::vector<Eigen::Vector3f>>& q2V,
                    const P2DFitParams& params = P2DFitParams());
