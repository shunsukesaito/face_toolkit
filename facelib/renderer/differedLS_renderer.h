#pragma once

#include <opencv2/opencv.hpp>

#include <gl_utility/framebuffer.h>
#include <shape_model/face_model.h>

#include "base_renderer.h"

struct DifferedLSRenderParams{
    bool enable_mask = 0;
    
    float mesomap_size = 6000.0;
    
    float alpha = 1.0;
    
    int location = 0;
    
    int sub_samp = 1; // subsampling rate for depth map (higher, more accurate, but maybe slower)
    
    void init(GLProgram& prog);
    void update(GLProgram& prog);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

struct DifferedLSRenderer : public BaseRenderer
{
    enum RT_NAMES
    {
        diff_albedo = 0,
        spec_albedo,
        normal,
        uv,
        count
    };

    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    DifferedLSRenderParams param_;
    
    DifferedLSRenderer(){}
    DifferedLSRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    virtual void init(std::string data_dir, std::string shader_dir, FaceModelPtr model);
    void render(const Camera& camera,
                const FaceData& fd);
    
#ifdef FACE_TOOLKIT
    virtual void render(const FaceResult& result, int cam_id = 0, int frame_id = 0);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
