#pragma once

#include <opencv2/opencv.hpp>

#include <gl_utility/framebuffer.h>
#include <shape_model/face_model.h>

#include "base_renderer.h"

struct LSRenderParams{
    
    bool use_pointlight = 0;
    bool enable_mask = 0;
    bool enable_cull_occlusion = 0;
    float cull_offset = 0.0;
    float light_rot = 0.0;
    glm::vec3 light_pos = glm::vec3(1.0,0.0,0.0);
    float diff_scale = 1.0;
    float spec_scale = 1.0;
    
    float mesomap_size = 6000.0;
    
    float alpha = 1.0;
    
    bool uv_view = 0;
    
    int env_id = 0;
    int env_size = 0;
    
    int location = 0;
    
    int sub_samp = 1; // subsampling rate for depth map (higher, more accurate, but maybe slower)
    
    void init(GLProgram& prog);
    void update(GLProgram& prog);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

struct LSRenderer : public BaseRenderer
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
        uv,
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
    
    LSRenderer(){}
    LSRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
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
