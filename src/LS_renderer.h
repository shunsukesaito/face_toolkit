#pragma once

#include <opencv2/opencv.hpp>

#include "EigenHelper.h"
#include "camera.h"
#include "gl_core.h"
#include "gl_mesh.h"
#include "framebuffer.h"
#include "face_model.h"

#ifdef WITH_IMGUI
#include "imgui.h"
#endif

struct LSRenderParams{
    
    bool use_pointlight = 0;
    bool enable_mask = 0;
    bool enable_cull_occlusion = 0;
    float cull_offset = 0.0;
    float light_rot = 0.0;
    glm::vec3 light_pos = glm::vec3(1.0,0.0,0.0);
    
    bool uv_view = 0;
    
    int env_id = 0;
    int env_size = 0;
    
    int location = 0;
    
    int sub_samp = 2; // subsampling rate for depth map (higher, more accurate, but maybe slower)
    
    void init(GLProgram& prog);
    void update(GLProgram& prog);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

struct LSRenderer
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

    std::unordered_map<std::string, GLProgram> programs_;
    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    FramebufferPtr fb_depth_;
    LSRenderParams param_;
    
    std::vector<GLuint> diff_env_locations_;
    std::vector<GLuint> spec_env1_locations_;
    std::vector<GLuint> spec_env2_locations_;
    
    void init(std::string data_dir, FaceModelPtr model);
    void render(const Camera& camera,
                const FaceData& fd);
    
#ifdef WITH_IMGUI
    inline void updateIMGUI(){ param_.updateIMGUI();}
#endif
    
};
