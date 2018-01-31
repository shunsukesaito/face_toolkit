//
//  mesh_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include "base_renderer.h"

struct MeshRenderer : public BaseRenderer
{
    glMesh mesh_;
    int tessInner_ = 2;
    int tessOuter_ = 2;
    float tessAlpha_ = 1.0;
    
    MeshRenderer(){}
    MeshRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string data_dir,
              const Eigen::MatrixX3i& tri);
    
    void render(const Camera& camera,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml);
    
    void render(const Camera& camera,
                const Eigen::Matrix4f& RT,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml);
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, FaceModelPtr fm);
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
