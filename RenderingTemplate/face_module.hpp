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
#include "f2f_renderer.hpp"
#include "mesh_renderer.hpp"
#include "face_2dtracker.h"
#include "p2d_optimizer.h"

struct FaceModule
{
    Face2DDetectorPtr fdetector_;

    F2FRenderer f2f_renderer_;
    MeshRenderer mesh_renderer_;

    Camera camera_;

    FaceModel facemodel_;
    FaceParams fParam_;
    
    std::vector<P2P2DC> p2p_constraints_;
    std::vector<P2L2DC> p2l_constraints_;
    
    P2DFitParams p2d_param_;
    
    void init(std::string data_dir);
    void update(cv::Mat& img);
    
    void preview();
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

#endif /* face_module_hpp */
