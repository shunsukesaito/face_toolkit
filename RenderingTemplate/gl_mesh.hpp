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
    
    void init(GLProgram&,
              int width,
              int height,
              const std::vector<Eigen::Vector2f>&,
              const Eigen::Vector4f&);
};

struct glMesh
{
    std::vector<glm::vec2> uvs_;
    std::vector<glm::vec3> pts_;
    std::vector<glm::vec3> nml_;
    std::vector<glm::vec4> clr_;
    
    std::vector<unsigned int> tri_;
    
    GLuint idx_vbo_;
    
    void init(GLProgram& prog,
              const Eigen::VectorXf& pts,
              const Eigen::MatrixX3f& nml,
              const Eigen::MatrixX2f& uv,
              const Eigen::MatrixX3i& tri_pts,
              const Eigen::MatrixX3i& tri_uv);
    
    void init(GLProgram& prog,
              const Eigen::VectorXf& pts,
              const Eigen::VectorXf& clr,
              const Eigen::MatrixX3f& nml,
              const Eigen::MatrixX2f& uv,
              const Eigen::MatrixX3i& tri_pts,
              const Eigen::MatrixX3i& tri_uv);
    
    void init(GLProgram& prog,
              const Eigen::VectorXf& pts,
              const Eigen::MatrixX3f& nml,
              const Eigen::MatrixX3i& tri_pts);
    
    void init(GLProgram& prog,
              const Eigen::VectorXf& pts);
    
    void init(GLProgram& prog,
              const Eigen::VectorXf& pts,
              const Eigen::Vector4f& clr);

    void init_with_idx(GLProgram& prog,
                       const Eigen::VectorXf& pts,
                       const Eigen::MatrixX3f& nml,
                       const Eigen::MatrixX2f& uv,
                       const Eigen::MatrixX3i& tri_pts,
                       const Eigen::MatrixX3i& tri_uv);
    
    void init_with_idx(GLProgram& prog,
                       const Eigen::VectorXf& pts,
                       const Eigen::MatrixX3f& nml,
                       const Eigen::MatrixX3i& tri_pts);
    
    void init_with_idx(GLProgram& prog,
                       const Eigen::VectorXf& pts,
                       const Eigen::VectorXf& clr,
                       const Eigen::MatrixX3f& nml,
                       const Eigen::MatrixX3i& tri_pts);
    
    void init_with_idx(GLProgram& program,
                       const Eigen::VectorXf& pts,
                       const Eigen::VectorXf& clr,
                       const Eigen::MatrixX3f& nml,
                       const Eigen::MatrixX2f& uvs,
                       const Eigen::MatrixX3i& tri_pts,
                       const Eigen::MatrixX3i& tri_uv);
    
    void update(GLProgram& prog,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml,
                const Eigen::MatrixX3i& tri_pts);
    
    void update(GLProgram& prog,
                const Eigen::VectorXf& pts,
                const Eigen::VectorXf& clr,
                const Eigen::MatrixX3f& nml,
                const Eigen::MatrixX3i& tri_pts);
    
    void update_with_idx(GLProgram& prog,
                         const Eigen::VectorXf& pts,
                         const Eigen::MatrixX3f& nml);
    
    void update_with_idx(GLProgram& prog,
                         const Eigen::VectorXf& pts,
                         const Eigen::VectorXf& clr,
                         const Eigen::MatrixX3f& nml);
};

#endif /* gl_mesh_hpp */
