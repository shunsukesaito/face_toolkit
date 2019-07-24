//
//  mesh_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include "base_renderer.h"
#include <gl_utility/framebuffer.h>

struct MeshRenderer : public BaseRenderer
{
    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    
    int tessInner_ = 1;
    int tessOuter_ = 1;
    float tessAlpha_ = 1.0;
    
    bool useSpec_ = true;
    
    float alpha_ = 1.0;
    
    int sub_samp_ = 1;
    
    MeshRenderer(){}
    MeshRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string shader_dir,
              const Eigen::MatrixX3i& tri);
    
    void render(const Camera& camera,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml,
                const Eigen::VectorXf& clr,
                const Eigen::Matrix4f& RT = Eigen::Matrix4f::Identity());
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, std::string shader_dir, FaceModelPtr fm);
    virtual void render(const FaceResult& result, int cam_id = 0, int frame_id = 0);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
