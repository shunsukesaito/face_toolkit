#pragma once

#include <opencv2/opencv.hpp>

#include "EigenHelper.h"
#include "face_gradient.h"
#include "camera.hpp"
#include "gl_core.hpp"
#include "gl_mesh.hpp"
#include "framebuffer.hpp"
#include "face_model.hpp"

#ifdef WITH_IMGUI
#include "imgui.h"
#endif

struct IBLRenderParams{
    int texture_mode = 0; // 0: none, 1: uv space, 2: image space
    int diffuse_mode = 1; // 0: SH, 1: HDRI
    bool enable_mask = 0;
    bool enable_cull_occlusion = 0;
    float cull_offset = 0.0;
    float light_rot = 0.0;
    bool uv_view = 0;
    
    int env_id = 0;
    int env_size = 0;
    
    int sub_samp = 4; // subsampling rate for depth map (higher, more accurate, but maybe slower)
    
    void init(GLProgram& prog);
    void update(GLProgram& prog);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

struct IBLRenderer
{
    std::unordered_map<std::string, GLProgram> programs_;
    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_depth_;
    IBLRenderParams param_;
    
    std::vector<GLuint> spec_HDRI_locations_;
    std::vector<GLuint> diff_HDRI_locations_;
    std::vector<Eigen::Matrix3Xf> SHCoeffs_;
    
    // sphere rendering
    glSphere ball_;
    
    void init(std::string data_dir, const FaceModel& model);
    void render(const Camera& camera,
                const FaceParams& fParam,
                const FaceModel& model,
                bool draw_sphere = false);
    
#ifdef WITH_IMGUI
    inline void updateIMGUI(){ param_.updateIMGUI();}
#endif
    
};
