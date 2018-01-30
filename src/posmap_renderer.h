//
//  posemap_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef posmap_renderer_hpp
#define posmap_renderer_hpp

#include "base_renderer.h"
#include "framebuffer.h"
#include "face_model.h"

struct PosMapRenderer : public BaseRenderer
{
    glPlane plane_;
    glMesh mesh_;
    FramebufferPtr fb_;
    int location_ = 0; // 0: position, 1: normal
    int tessInner_ = 21;
    int tessOuter_ = 21;
    float tessAlpha_ = 1.0;
    
    PosMapRenderer(){}
    PosMapRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    virtual void init(std::string data_dir,
                      FaceModelPtr model);
    
    void render(const FaceData& fd);
    void render(int w, int h, const FaceData& fd, std::vector<cv::Mat_<cv::Vec4f>>& out);
    
#ifdef FACE_TOOLKIT
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};

#endif /* mesh_renderer_hpp */
