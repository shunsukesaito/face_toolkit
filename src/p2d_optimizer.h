
#ifndef LANDMARKFITTER_H
#define LANDMARKFITTER_H

#include "EigenHelper.h"

#include "camera.hpp"
#include "face_model.hpp"
#include "face_gradient.h"

#include <spdlog/spdlog.h>

#ifdef WITH_IMGUI
#include "imgui.h"
#endif

using namespace std;

struct P2DFitParams
{
    DOF dof = DOF(40, 20, 0, 3, 3, 0, 0);
    
    int maxIter_ = 10;
    
    bool robust_ = false;
    
    float gn_thresh_ = 1.0e-4f;
    float mclose_thresh_ = 4.0f;
    float angle_thresh_ = 8.0f;
    
    float w_p2p_ = 1.e-3f;
    float w_p2l_ = 1.e-3f;
    float w_reg_pca_id_ = 5.e-5f;
    float w_reg_pca_ex_ = 5.e-4f;
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

bool RigidAlignment(const std::vector<Eigen::Vector3f> &q,
                    const std::vector<Eigen::Vector3f> &p,
                    Camera& camera);

void compute_rigid_motion(const Eigen::Matrix4f &intrinsic,
						  const std::vector<Eigen::Vector3f> &p3d,
						  const std::vector<Eigen::Vector3f> &q2d,
                          Eigen::Matrix4f& extrinsic);

void P2DGaussNewtonMultiView(FaceParams& fParam,
                             std::vector< Camera >& cameras,
                             const FaceModel& fModel,
                             const std::vector<P2P2DC>& C_P2P,
                             std::vector<P2L2DC>& C_P2L,
                             const std::vector<std::vector<Eigen::Vector3f>>& p2d,
                             const P2DFitParams& params = P2DFitParams());

//void Landmark2DFittingWithContourMultiView(F2FFaceModel& faceModel,
//                                           std::vector<hfm::Camera>& cameras,
//                                           render::Renderer& faceModelRenderer,
//                                           const std::vector<std::vector<Eigen::Vector3f>>& p2d,
//                                           const Land2DFitParams& params = Land2DFitParams(),
//                                           std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("console"));

#endif /* defined(LANDMARKFITTER_H) */
