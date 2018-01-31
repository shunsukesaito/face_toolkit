//
//  p2d_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include "base_renderer.h"

struct P2DRenderer : public BaseRenderer
{
    glPoint2D p2d_;
    
    P2DRenderer(){}
    P2DRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string data_dir);
    
    void render(int w, int h, const std::vector<Eigen::Vector2f>& pts);
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, FaceModelPtr fm);
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};

