//
//  sphere_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include "base_renderer.h"
#include <gl_utility/framebuffer.h>

struct SphereRenderer : public BaseRenderer
{
    glSphere mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    
    float alpha_ = 1.0;
    
    int sub_samp_ = 1;
    
    bool uvview_ = false;
    
    SphereRenderer(){}
    SphereRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string data_dir, std::string shader_dir);
    
    void render(const Camera& camera, const Eigen::Matrix4f& RT = Eigen::Matrix4f::Identity());
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, std::string shader_dir, FaceModelPtr fm);
    virtual void render(const FaceResult& result, int cam_id = 0, int frame_id = 0);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
