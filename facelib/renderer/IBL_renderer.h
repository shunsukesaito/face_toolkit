#pragma once

#include "base_renderer.h"

#include <gl_utility/framebuffer.h>
#include <shape_model/face_model.h>


struct IBLRenderParams{
    int texture_mode = 0; // 0: none, 1: uv space, 2: image space
    bool enable_mask = 0;
    bool enable_cull_occlusion = 0;
    float cull_offset = 0.0;
    float light_rot = 0.0;
    bool uv_view = 0;

    float alpha = 1.0;
    
    int env_id = 0;
    int env_size = 0;
    
    int sub_samp = 2; // subsampling rate for depth map (higher, more accurate, but maybe slower)
    
    void init(GLProgram& prog);
    void update(GLProgram& prog);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

struct IBLRenderer : public BaseRenderer
{
    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    FramebufferPtr fb_depth_;
    IBLRenderParams param_;
    bool show_sphere_ = false;
    
    std::vector<GLuint> spec_HDRI_locations_;
    std::vector<GLuint> diff_HDRI_locations_;
    
    // sphere rendering
    glSphere ball_;
    
    IBLRenderer(){}
    IBLRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    virtual void init(std::string data_dir, FaceModelPtr model);
    void render(const Camera& camera,
                const FaceData& fd,
                bool draw_sphere = false);
    
#ifdef FACE_TOOLKIT
    virtual void render(const FaceResult& result);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
