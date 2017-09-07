#pragma once

#include <opencv2/opencv.hpp>

#include "EigenHelper.h"
#include "face_gradient.h"
#include "camera.hpp"
#include "gl_core.hpp"
#include "gl_mesh.hpp"
#include "framebuffer.hpp"
#include "face_model.hpp"

struct F2FRenderParams{
    uint enable_tex = 0;
    uint enable_mask = 0;
    uint enable_seg = 0;
    float cull_offset = 0.0;
    
    void init(GLProgram& prog);
    void update(GLProgram& prog);
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
    
    GLProgram prog_;
    glMesh mesh_;
    FramebufferPtr fb_;
    F2FRenderParams param_;
    
    void init(std::string data_dir, Camera& camera, FaceModel& model);
    void render(int w, int h, Camera& camera, FaceParams& fParam, FaceModel& model, std::vector<cv::Mat_<cv::Vec4f>>& out);
    
    static void computeJacobianColor(Eigen::VectorXf& Jtr,
                                     Eigen::MatrixXf& JtJ,
                                     const Eigen::MatrixXf& w_al,
                                     const std::vector<cv::Mat_<cv::Vec3f>>& w_al_uv,
                                     const std::vector<Eigen::Vector2f>& pV,
                                     const std::vector<Eigen::MatrixX2f>& dpV,
                                     const std::vector<Eigen::Vector3f>& nV,
                                     const std::vector<Eigen::MatrixXf>& dnV,
                                     const Eigen::Vector3f* shCoeff,
                                     const std::vector< cv::Mat_<cv::Vec4f> >& renderTarget,
                                     const cv::Mat_<cv::Vec4f>& renderRGB,
                                     const cv::Mat_<cv::Vec4f>& inputRGB,
                                     const cv::Mat_<cv::Vec3f>& dIx,
                                     const cv::Mat_<cv::Vec3f>& dIy,
                                     const DOF& dof,
                                     const float w,
                                     bool tex_mode);
};




