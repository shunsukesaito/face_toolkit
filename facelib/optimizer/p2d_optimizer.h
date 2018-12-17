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

#include <utility/EigenHelper.h>
#include <gl_utility/camera.h>
#include <shape_model/face_model.h>

#include "base_optimizer.h"
#include "face_gradient.h"
#include "face_result.h"

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

using namespace std;

struct P2DFitParams
{
    bool onetime_run_ = false;
    bool run_ = false;
    
    DOF dof = DOF(40, 20, 0, 3, 3, 0, 0, 0, 0);
    
    int maxIter_ = 10;
    
    bool robust_ = false;
    bool verbose_ = false;
    
    float gn_thresh_ = 1.0e-4f;
    float mclose_thresh_ = 4.0f;
    float angle_thresh_ = 8.0f;
    
    float w_p2p_ = 1.e-3f;
    float w_p2l_ = 1.e-3f;
    float w_reg_pca_id_ = 1.e-1f;
    float w_reg_pca_ex_ = 1.e-1f;
    
    float w_reg_lmix_ = 1.e-1f;
    float lmix_l_ = 0.01;
    float lmix_u_ = 0.9;
    float lmix_lambda_ = 1.0;
    
    bool loadParamFromTxt(std::string file);
    bool saveParamToTxt(std::string file);
    
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

void P2DGaussNewton(std::vector<FaceData>& fd,
                    std::vector<Camera>& cameras,
                    const MFMVCaptureData& data,
                    const std::vector<P2P2DC>& CP2P,
                    std::vector<P2L2DC>& CP2L,
                    const P2DFitParams& params = P2DFitParams());

struct P2DOptimizer : public BaseOptimizer
{
    P2DFitParams param_;
    FaceModelPtr fm_;
    
    P2DOptimizer(){}
    P2DOptimizer(std::string name) : BaseOptimizer(name){}
    
    virtual void init(std::string data_dir, FaceModelPtr fm);
    virtual void solve(FaceResult& result);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
    
    static OptimizerHandle Create(std::string name, bool run = false);
};
