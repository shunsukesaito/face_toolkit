//
//  gl_mesh.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gl_mesh.hpp"

void Mesh::init(GLProgram& program,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml,
                const Eigen::Matrix2f& uvs,
                const Eigen::MatrixX3i& tri_pts,
                const Eigen::MatrixX3i& tri_uv)
{
    assert(tri_pts.size() == tri_uv.size());
    
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_normal", DataType::VECTOR3, true);
    program.createAttribute("v_texcoord", DataType::VECTOR2, false);
    
    uvs_.resize(tri_pts.size());
    pts_.resize(tri_pts.size());
    nml_.resize(tri_pts.size());
    for(int i = 0; i < tri_pts.rows(); ++i)
    {
        const int& idx0 = tri_pts(i, 0);
        const int& idx1 = tri_pts(i, 1);
        const int& idx2 = tri_pts(i, 2);
        
        const int& idxuv0 = tri_uv(i, 0);
        const int& idxuv1 = tri_uv(i, 1);
        const int& idxuv2 = tri_uv(i, 2);
        
        pts_[i * 3 + 0] = glm::vec3(pts(idx0*3+0), pts(idx0*3+1), pts(idx0*3+2));
        pts_[i * 3 + 1] = glm::vec3(pts(idx1*3+0), pts(idx1*3+1), pts(idx1*3+2));
        pts_[i * 3 + 2] = glm::vec3(pts(idx2*3+0), pts(idx2*3+1), pts(idx2*3+2));
        
        nml_[i * 3 + 0] = glm::vec3(nml(idx0, 0), nml(idx0, 1), nml(idx0, 2));
        nml_[i * 3 + 1] = glm::vec3(nml(idx1, 0), nml(idx1, 1), nml(idx1, 2));
        nml_[i * 3 + 2] = glm::vec3(nml(idx2, 0), nml(idx2, 1), nml(idx2, 2));

        uvs_[i * 3 + 0] = glm::vec2(uvs(idxuv0, 0), uvs(idxuv0, 1));
        uvs_[i * 3 + 1] = glm::vec2(uvs(idxuv1, 0), uvs(idxuv1, 1));
        uvs_[i * 3 + 2] = glm::vec2(uvs(idxuv2, 0), uvs(idxuv2, 1));
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_normal", nml_);
    program.setAttributeData("v_texcoord", uvs_);
}

void Mesh::update(GLProgram& program,
                  const Eigen::VectorXf& pts,
                  const Eigen::MatrixX3f& nml,
                  const Eigen::MatrixX3i& tri)
{
    assert(pts_.size() == tri.size());
    
    int idx0, idx1, idx2;
    for(int i = 0; i < tri.rows(); ++i)
    {
        idx0 = tri(i, 0);
        idx1 = tri(i, 1);
        idx2 = tri(i, 2);
        
        pts_[i * 3 + 0] = glm::vec3(pts(idx0*3+0), pts(idx0*3+1), pts(idx0*3+2));
        pts_[i * 3 + 1] = glm::vec3(pts(idx1*3+0), pts(idx1*3+1), pts(idx1*3+2));
        pts_[i * 3 + 2] = glm::vec3(pts(idx2*3+0), pts(idx2*3+1), pts(idx2*3+2));
        
        nml_[i * 3 + 0] = glm::vec3(nml(idx0, 0), nml(idx0, 1), nml(idx0, 2));
        nml_[i * 3 + 1] = glm::vec3(nml(idx1, 0), nml(idx1, 1), nml(idx1, 2));
        nml_[i * 3 + 2] = glm::vec3(nml(idx2, 0), nml(idx2, 1), nml(idx2, 2));
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_normal", nml_);
}
