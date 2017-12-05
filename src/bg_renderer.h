//
//  bg_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef bg_renderer_hpp
#define bg_renderer_hpp

#include "base_renderer.h"

struct BGRenderer : public BaseRenderer
{
    glPlane plane_;
    int width_, height_;
    
    BGRenderer(){}
    BGRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string data_dir,
              std::string img_path);
    
    void init(std::string data_dir,
              const cv::Mat& img = cv::Mat());

    void render(const cv::Mat& img = cv::Mat(), bool mirror = false);
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, FaceModelPtr fm);
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};


#endif /* bg_renderer_hpp */
