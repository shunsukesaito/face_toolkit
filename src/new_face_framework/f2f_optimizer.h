
#pragma once

#include <spdlog/spdlog.h>

#include "camera.hpp"
#include "face_model.hpp"
#include "face_gradient.h"
#include "f2f_renderer.hpp"

#ifdef WITH_IMGUI
#include "imgui.h"
#endif

struct F2FParams
{
    bool run_ = false;
    
    DOF dof = DOF( 40, 40, 40, 3, 3, 0, 0, 0, 27);

    std::vector<int> maxIter_ = {0, 3, 5, 5};
    
    bool verbose_ = false;
    bool robust_ = false;
    bool sym_with_exp_ = false;
    
    float gn_thresh_ = 1.e-6f; // gauss-newton threshold
    float mclose_thresh_ = 4.0f; // mouth close threshold
    float w_pix_ = 1.0f;
//    float w_mc_ = 1000.0f; // mouth close
    float w_reg_pca_id_ = 5.e-5f;
    float w_reg_pca_ex_ = 5.e-4f;
    float w_reg_pca_cl_ = 5.e-5f;
    float w_p2p_ = 8.e-3f;
    float w_p2l_ = 8.e-3f;
    float w_p3d_ = 100.f;
    
    //float w_head_ = 500.f;
    float w_sym_ = 1.e-1f;
    
    int smoothLev_ = 2;
    
    char* dampDataDir_ = 0;
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

typedef std::shared_ptr<F2FParams> F2FParamsPtr;

// renderTarget contains 
// positions,normals,colors,vIndices,vBarycentric,texCoords
void F2FGaussNewton(FaceData& fd,
                    Camera& camera,
                    F2FRenderer& renderer,
                    const cv::Mat_<cv::Vec4f>& inputRGB,
                    const std::vector<P2P2DC>& C_P2P,
                    std::vector<P2L2DC>& C_P2L,
                    const std::vector<Eigen::Vector3f>& q2V,
                    unsigned int level,
                    const F2FParams& params,
                    std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("console"));

void F2FHierarchicalGaussNewton(FaceData& fd,
                                Camera& camera,
                                F2FRenderer& renderer,
                                const cv::Mat_<cv::Vec4f>& inputRGB,
                                const std::vector<P2P2DC>& C_P2P,
                                std::vector<P2L2DC>& C_P2L,
                                const std::vector<Eigen::Vector3f>& q2V,
                                const F2FParams& params,
                                std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("console"));
