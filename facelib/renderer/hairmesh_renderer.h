//
//  hairmesh_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/5/17.
//  Copyright © 2018 Shunsuke Saito. All rights reserved.
//
#pragma once

#include "base_renderer.h"
#include <gl_utility/framebuffer.h>

struct HairMeshRenderer : public BaseRenderer
{
    glMesh mesh_hair_;
    glMesh mesh_torso_;
    glPlane plane_;
    FramebufferPtr fb_;
    FramebufferPtr fb_depth_;
    
    float alpha_ = 1.0;
    
    float bias_ = 1.0;
    
    HairMeshRenderer(){}
    HairMeshRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string shader_dir,
              const Eigen::MatrixX3i& tri_t,
              const Eigen::MatrixX3i& tri_h);
    
    void render(const Camera& camera,
                const Eigen::VectorXf& pts_t,
                const Eigen::VectorXf& pts_h,
                const Eigen::MatrixX3f& nml_h);
    
    void render(const Camera& camera,
                const Eigen::Matrix4f& RT,
                const Eigen::VectorXf& pts_t,
                const Eigen::VectorXf& pts_h,
                const Eigen::MatrixX3f& nml);
        
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
