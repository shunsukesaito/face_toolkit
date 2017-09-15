//
//  face_module.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef face_module_hpp
#define face_module_hpp

#include "camera.hpp"
#include "face_model.hpp"

#include "face_2dtracker.h"

#include "f2f_renderer.hpp"
#include "mesh_renderer.hpp"
#include "p3d_renderer.hpp"

#include "p2d_optimizer.h"

struct FaceModule
{
    Face2DDetectorPtr fdetector_;

    F2FRenderer f2f_renderer_;
    MeshRenderer mesh_renderer_;
    P3DRenderer p3d_renderer_;

    std::vector<Camera> cameras_;

    FaceModel fModel_;
    FaceParams fParam_;
    
    std::vector<P2P2DC> c_p2p_;
    std::vector<P2L2DC> c_p2l_;
    
    P2DFitParams p2d_param_;
    
    bool enable_p2pfit_ = false;
    
    inline void reset(){ fParam_.init(fModel_); }
    
    void init(std::string data_dir);
    void update(cv::Mat& img);
    
    void preview();
    
    inline Camera& getCamera(){return cameras_[0];}
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

#endif /* face_module_hpp */
