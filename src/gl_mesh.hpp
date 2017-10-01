//
//  gl_mesh.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef gl_mesh_hpp
#define gl_mesh_hpp

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gl_core.hpp"
#include "gl_utils.h"

// flags for attributes
static const int AT_POSITION = 0x0001;
static const int AT_NORMAL   = 0x0002;
static const int AT_COLOR    = 0x0004;
static const int AT_UV       = 0x0008;
static const int AT_TRI      = 0x0010;

struct glPlane
{
    std::vector<glm::vec2> uvs_;
    std::vector<glm::vec3> pts_;
    
    void init(GLProgram&, float z = 1.0);
};

struct glPoint2D
{
    std::vector<glm::vec2> pts_;
    std::vector<glm::vec4> clr_;
    
    void init(GLProgram& prog);
    
    void update(GLProgram& prog,
                int width,
                int height,
                const std::vector<Eigen::Vector2f>& pts,
                const Eigen::Vector4f& clr);
};

struct glSphere
{
    glSphere(){ generateSphere(10.0,24,48);}
    
    void generateSphere(float radius, unsigned int rings, unsigned int sectors);
    void init(GLProgram& prog, int flag);
    void update(GLProgram& prog, int flag);
    
    std::vector<glm::vec2> uvs_;
    std::vector<glm::vec3> nml_;
    std::vector<glm::vec3> pts_;
    std::vector<glm::vec4> clr_;
    
    std::vector<unsigned int> tri_;
};

struct glMesh
{
    std::vector<glm::vec2> uvs_;
    std::vector<glm::vec3> pts_;
    std::vector<glm::vec3> nml_;
    std::vector<glm::vec4> clr_;
    
    std::vector<unsigned int> tri_;
    
    void init(GLProgram& prog, int flag);
    void update(GLProgram& prog, int flag);
    
    void update_position(const Eigen::VectorXf& pts,
                         const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_position(const std::vector<Eigen::Vector3f>& pts,
                         const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_normal(const Eigen::MatrixX3f& nml,
                       const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_color(const Eigen::VectorXf& clr,
                      const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_color(const Eigen::Vector4f& clr,
                      const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_uv(const Eigen::MatrixX2f& uvs,
                   const Eigen::MatrixX3i& tri_uv,
                   const Eigen::MatrixX3i& tri_pts = Eigen::MatrixX3i());
    
    void update_tri(const Eigen::MatrixX3i& tri);
};

#endif /* gl_mesh_hpp */
