//
//  p3d_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include "base_renderer.h"

struct P3DRenderer : public BaseRenderer
{
    glMesh p3d_;
    
    P3DRenderer(){}
    P3DRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string shader_dir);
    
    void render(const Camera& camera,
                const std::vector<Eigen::Vector3f>& pts);
    
    void render(const Camera& camera,
                const Eigen::Matrix4f& RT,
                const std::vector<Eigen::Vector3f>& pts,
                const Eigen::Vector4f& color = Eigen::Vector4f(0,0,1,1));
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, std::string shader_dir, FaceModelPtr fm);
    virtual void render(const FaceResult& result, int cam_id, int frame_id);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
