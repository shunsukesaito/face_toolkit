#pragma once

#include <opencv2/opencv.hpp>

#include "EigenHelper.h"
#include "face_gradient.h"
#include "camera.h"
#include "gl_core.h"
#include "gl_mesh.h"
#include "framebuffer.h"
#include "face_model.h"

#ifdef WITH_IMGUI
#include "imgui.h"
#endif

struct F2FRenderParams{
    bool enable_tex = 0;
    bool enable_mask = 1;
    bool enable_seg = 0;
    float cull_offset = 0.0;
    
    // for preview
    bool tex_mode = 0;
    bool enable_inv_diffuse = 0;
    
    int location = 4;
    
    bool preview = false;
    
    void init(GLProgram& prog, bool _preview = false);
    void update(GLProgram& prog);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

struct F2FRenderer
{
    enum RT_NAMES
    {
        positions = 0,
        normals,
        colors,
        texCoords,
        diffuse,
        shading,
        vBarycentric,
        vIndices,
        count
    };
    
    std::unordered_map<std::string, GLProgram> programs_;
    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    F2FRenderParams param_;
    
    void init(std::string data_dir, FaceModelPtr model);
    void render(const Camera& camera, const FaceData& fd);
    void render(int w, int h, const Camera& camera, const FaceData& fd, std::vector<cv::Mat_<cv::Vec4f>>& out);
    
#ifdef WITH_IMGUI
    inline void updateIMGUI(){ param_.updateIMGUI();}
#endif
    
    static float computeJacobianColor(Eigen::VectorXf& Jtr,
                                      Eigen::MatrixXf& JtJ,
                                      const FaceData& fd,
                                      const std::vector<Eigen::Vector2f>& pV,
                                      const std::vector<Eigen::Matrix2Xf>& dpV,
                                      const std::vector<Eigen::Vector3f>& nV,
                                      const std::vector<Eigen::Matrix3Xf>& dnV,
                                      const Eigen::Matrix3Xf& sh,
                                      const std::vector< cv::Mat_<cv::Vec4f> >& renderTarget,
                                      const cv::Mat_<cv::Vec4f>& inputRGB,
                                      const cv::Mat_<cv::Vec3f>& dIx,
                                      const cv::Mat_<cv::Vec3f>& dIy,
                                      const DOF& dof,
                                      const float w);
};
