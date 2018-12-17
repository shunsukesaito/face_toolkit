//
//  mp2_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 10/21/18.
//  Copyright © 2018 Shunsuke Saito. All rights reserved.
//
#pragma once

#include "base_renderer.h"
#include <gl_utility/framebuffer.h>

struct MP2Renderer : public BaseRenderer
{
    glMesh mesh_;
    glPlane plane_;
    FramebufferPtr fb_plane_;
    
    cv::Mat_<uchar> mask_;
    
    std::vector<glm::vec3> pts_;
    
    std::vector<unsigned int> tri_;
    
    bool texture_update_ = true;
    float alpha_ = 1.0;
    
    int sub_samp_ = 1;
    
    MP2Renderer(){}
    MP2Renderer(std::string name, bool show) : BaseRenderer(name,show){}
    
    void init(std::string data_dir,
              std::string shader_dir,
              const Eigen::MatrixX3i& tri,
              const cv::Mat& img = cv::Mat());
    
    void set_vertices(const std::vector<unsigned int>& tri,
                      const std::vector<glm::vec3>& bpts);
    
    void render(const Camera& camera,
                const Eigen::VectorXf& pts,
                const std::vector<unsigned int>& tri,
                const std::vector<glm::vec3>& bpts,
                const cv::Mat& img = cv::Mat());
    
    void render(const Camera& camera,
                const Eigen::Matrix4f& RT,
                const Eigen::VectorXf& pts,
                const std::vector<unsigned int>& tri,
                const std::vector<glm::vec3>& bpts,
                const cv::Mat& img = cv::Mat());

    void render(const Camera& camera,
                const Eigen::Matrix4f& RT,
                const Eigen::VectorXf& pts,
                const cv::Mat& img = cv::Mat());
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, std::string shader_dir, FaceModelPtr fm);
    virtual void render(const FaceResult& result, int cam_id = 0, int frame_id = 0);
#endif
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
    
    static RendererHandle Create(std::string name, bool show = false);
};
