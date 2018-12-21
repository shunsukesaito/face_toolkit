//
//  prt_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include "base_renderer.h"
#include <gl_utility/framebuffer.h>
#include <utility/prt_data.h>

struct PRTRenderer : public BaseRenderer
{
    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    std::vector<std::vector<glm::vec3>> glPrt_;
    
    bool analytial_ = false;
    float alpha_ = 1.0;
    
    int sub_samp_ = 1;
    
    PRTRenderer(){}
    PRTRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string shader_dir,
              const Eigen::MatrixX3i& tri);

    void render(const Camera& camera,
                const PRTData& data);
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
