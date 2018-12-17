
#pragma once

#include <spdlog/spdlog.h>

#include <gl_utility/camera.h>
#include <shape_model/face_model.h>
#include <optimizer/face_gradient.h>
#include <optimizer/face_result.h>
#include <optimizer/base_optimizer.h>

#include "f2f_renderer.h"

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

struct F2FParams
{
    bool onetime_run_ = false;
    bool run_ = false;
    
    DOF dof = DOF( 40, 40, 40, 3, 3, 0, 0, 0, 27);

    std::vector<int> maxIter_ = {0, 3, 5, 5, 0, 0, 0, 0};
    
    bool verbose_ = false;
    bool robust_ = false;
    bool sym_with_exp_ = false;
    
    bool enable_seg_ = false;
    
    float gn_thresh_ = 1.e-6f; // gauss-newton threshold
    float mclose_thresh_ = 4.0f; // mouth close threshold
    float w_pix_ = 1.0f;

    float w_reg_pca_id_ = 1.e-1f;
    float w_reg_pca_ex_ = 1.e-1f;
    float w_reg_pca_cl_ = 1.e-3f;
    float w_p2p_ = 8.e-3f;
    float w_p2l_ = 8.e-3f;
    float w_p3d_ = 100.f;
    
    float w_sym_ = 1.e-1f;
    
    int smoothLev_ = 2;
    
    char* dampDataDir_ = 0;
    
    bool loadParamFromTxt(std::string file);
    bool saveParamToTxt(std::string file);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

typedef std::shared_ptr<F2FParams> F2FParamsPtr;

// renderTarget contains 
// positions,normals,colors,vIndices,vBarycentric,texCoords
void F2FGaussNewton(std::vector<FaceData>& fd,
                    std::vector<Camera>& cameras,
                    F2FRenderer& renderer,
                    const MFMVCaptureData& data,
                    const std::vector<P2P2DC>& C_P2P,
                    std::vector<P2L2DC>& C_P2L,
                    unsigned int level,
                    const F2FParams& params,
                    std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("console"));

void F2FHierarchicalGaussNewton(std::vector<FaceData>& fd,
                                std::vector<Camera>& cameras,
                                F2FRenderer& renderer,
                                const MFMVCaptureData& data,
                                const std::vector<P2P2DC>& C_P2P,
                                std::vector<P2L2DC>& C_P2L,
                                const F2FParams& params,
                                std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("console"));

struct F2FOptimizer : public BaseOptimizer
{
    F2FParams param_;
    FaceModelPtr fm_;
    F2FRenderer renderer_;
    
    std::shared_ptr<spdlog::logger> logger_ = spdlog::stdout_color_mt("console");
    
    F2FOptimizer(){}
    F2FOptimizer(std::string name) : BaseOptimizer(name){}

    virtual void init(std::string data_dir, FaceModelPtr fm);
    virtual void solve(FaceResult& result);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
    
    static OptimizerHandle Create(std::string name, bool run = false);
};
