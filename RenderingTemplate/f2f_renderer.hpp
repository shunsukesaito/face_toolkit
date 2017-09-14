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
    
    void init(std::string data_dir, Camera& camera, FaceModel& model);
    void render(Camera& camera, const FaceParams& fParam);
    void render(int w, int h, Camera& camera, const FaceParams& fParam, std::vector<cv::Mat_<cv::Vec4f>>& out);
    
    static float computeJacobianColor(Eigen::VectorXf& Jtr,
                                      Eigen::MatrixXf& JtJ,
                                      const Eigen::MatrixXf& w_al,
                                      const std::vector<Eigen::Vector2f>& pV,
                                      const std::vector<Eigen::MatrixX2f>& dpV,
                                      const std::vector<Eigen::Vector3f>& nV,
                                      const std::vector<Eigen::MatrixXf>& dnV,
                                      const Eigen::MatrixX3f& sh,
                                      const std::vector< cv::Mat_<cv::Vec4f> >& renderTarget,
                                      const cv::Mat_<cv::Vec4f>& renderRGB,
                                      const cv::Mat_<cv::Vec4f>& inputRGB,
                                      const cv::Mat_<cv::Vec3f>& dIx,
                                      const cv::Mat_<cv::Vec3f>& dIy,
                                      const DOF& dof,
                                      const float w);
};
