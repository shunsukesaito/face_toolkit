//
//  posemap_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <gl_utility/framebuffer.h>
#include <shape_model/face_model.h>

#include "base_renderer.h"

struct PosMapRenderer : public BaseRenderer
{
    glPlane plane_;
    glMesh mesh_;
    FramebufferPtr fb_;
    int location_ = 0; // 0: position, 1: normal
    int tessInner_ = 1;
    int tessOuter_ = 1;
    float tessAlpha_ = 1.0;
    float delta_ = 2.e-3;
    
    PosMapRenderer(){}
    PosMapRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    virtual void init(std::string data_dir,
                      std::string shader_dir,
                      FaceModelPtr model);
    
    void render(const FaceData& fd, std::string out_name = "pos.exr");
    void render(const Camera& camera, const FaceData& fd);
    void render(int w, int h, const FaceData& fd, std::vector<cv::Mat_<cv::Vec4f>>& out);
    
#ifdef FACE_TOOLKIT
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};

struct PosMapReconRenderer : public BaseRenderer
{
    glPlane plane_;
    glMesh mesh_;
    FramebufferPtr fb_;

    int tessInner_ = 1;
    int tessOuter_ = 1;
    float delta_ = 2.e-3;
    
    float alpha_ = 1.0;
    
    int sub_samp_ = 1;
    
    PosMapReconRenderer(){}
    PosMapReconRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    virtual void init(std::string data_dir,
                      std::string shader_dir, 
                      FaceModelPtr model);
    
    void render(const Camera& camera, const FaceData& fd);
    
#ifdef FACE_TOOLKIT
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
