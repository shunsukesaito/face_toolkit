#pragma once

#include <opencv2/opencv.hpp>

#include <renderer/base_renderer.h>
#include <optimizer/face_gradient.h>
#include <gl_utility/framebuffer.h>
#include <shape_model/face_model.h>

struct FaceResult;

struct F2FRenderParams{
    bool enable_tex = 0;
    bool enable_mask = 1;
    bool enable_seg = 0;
    bool enable_cull = 1;
    float cull_offset = 0.0;
    float alpha = 1.0;
    
    bool tex_mode = 0;
    bool enable_inv_diffuse = 0;
    
    int location = 4;

    void init(GLProgram& prog);
    void update(GLProgram& prog);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

struct F2FRenderer : public BaseRenderer
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
    
    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_;
    FramebufferPtr fb_depth_;
    F2FRenderParams param_;
    
    F2FRenderer(){}
    F2FRenderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    virtual void init(std::string data_dir, std::string shader_dir, FaceModelPtr model);
    
    void updateSegment(const cv::Mat& seg);
    
    void render(const Camera& camera, const FaceData& fd);
    void render(int w, int h, const Camera& camera, const FaceData& fd, std::vector<cv::Mat_<cv::Vec4f>>& out);
    
#ifdef FACE_TOOLKIT
    virtual void render(const FaceResult& result, int cam_id, int frame_id);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
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
    
    static RendererHandle Create(std::string name, bool show = false);
};
