#pragma once

#include <opencv2/opencv.hpp>

#include <gl_utility/framebuffer.h>
#include <shape_model/face_model.h>

#include "base_renderer.h"
#include "LS_renderer.h"

struct DeepLSRenderer : public BaseRenderer
{
    enum RT_NAMES
    {        
        all = 0,
        diff,
        spec,
        diff_albedo,
        spec_albedo,
        spec_normal,
        diff_normal,
        count
    };

    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    FramebufferPtr fb_depth_;
    LSRenderParams param_;
    
    std::vector<GLuint> diff_env_locations_;
    std::vector<GLuint> spec_env1_locations_;
    std::vector<GLuint> spec_env2_locations_;
    
    DeepLSRenderer(){}
    DeepLSRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    virtual void init(std::string data_dir, FaceModelPtr model);
    void render(const Camera& camera,
                const FaceData& fd);
    
#ifdef FACE_TOOLKIT
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
