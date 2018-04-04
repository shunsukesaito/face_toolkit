//
//  bg_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#pragma once

#include "base_renderer.h"

struct BGRenderer : public BaseRenderer
{
    glPlane plane_;
    int width_, height_;
    
    float alpha_ = 1.0;
    
    BGRenderer(){}
    BGRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string shader_dir,
              std::string img_path);
    
    void init(std::string shader_dir,
              const cv::Mat& img = cv::Mat());

    void render(const cv::Mat& img = cv::Mat(), bool mirror = false);
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, std::string shader_dir, FaceModelPtr fm);
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
