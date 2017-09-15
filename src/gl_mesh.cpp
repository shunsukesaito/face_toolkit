//
//  gl_mesh.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gl_mesh.hpp"

void glPlane::init(GLProgram &program, float z)
{
    pts_.clear();
    uvs_.clear();
    
    program.createAttribute("v_position", DataType::VECTOR3, false);
    program.createAttribute("v_texcoord", DataType::VECTOR2, false);
    
    pts_.push_back(glm::vec3(-1.,-1.,z));
    pts_.push_back(glm::vec3(-1.,1.,z));
    pts_.push_back(glm::vec3(1.,-1.,z));
    pts_.push_back(glm::vec3(1.,1.,z));
    pts_.push_back(glm::vec3(1.,-1.,z));
    pts_.push_back(glm::vec3(-1.,1.,z));
    
    uvs_.push_back(glm::vec2(0,0));
    uvs_.push_back(glm::vec2(0,1));
    uvs_.push_back(glm::vec2(1,0));
    uvs_.push_back(glm::vec2(1,1));
    uvs_.push_back(glm::vec2(1,0));
    uvs_.push_back(glm::vec2(0,1));
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_texcoord", uvs_);
}

void glPoint2D::init(GLProgram& program)
{
    pts_.clear();
    clr_.clear();

    program.createAttribute("v_p2d", DataType::VECTOR2, false);
    program.createAttribute("v_color", DataType::VECTOR4, false);
}

void glPoint2D::update(GLProgram& program,
                       int width,
                       int height,
                       const std::vector<Eigen::Vector2f>& pts,
                       const Eigen::Vector4f& clr)
{
    pts_.clear();
    clr_.clear();
    
    for(int i = 0; i < pts.size(); ++i)
    {
        pts_.push_back(glm::vec2(pts[i](0)/(float)width,pts[i](1)/(float)height));
        clr_.push_back(glm::vec4(clr(0),clr(1),clr(2),clr(3)));
    }
    
    program.setAttributeData("v_p2d", pts_);
    program.setAttributeData("v_color", clr_);
}


void glMesh::init(GLProgram& program,
                  const Eigen::VectorXf& pts,
                  const Eigen::MatrixX3f& nml,
                  const Eigen::MatrixX2f& uvs,
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

void glMesh::init(GLProgram& program,
                  const Eigen::VectorXf& pts,
                  const Eigen::VectorXf& clr,
                  const Eigen::MatrixX3f& nml,
                  const Eigen::MatrixX2f& uvs,
                  const Eigen::MatrixX3i& tri_pts,
                  const Eigen::MatrixX3i& tri_uv)
{
    assert(tri_pts.size() == tri_uv.size());
    
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_normal", DataType::VECTOR3, true);
    program.createAttribute("v_texcoord", DataType::VECTOR2, false);
    program.createAttribute("v_color", DataType::VECTOR4, true);
    
    uvs_.resize(tri_pts.size());
    clr_.resize(tri_pts.size());
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
        
        clr_[i * 3 + 0] = glm::vec4(clr(idx0*3+0), clr(idx0*3+1), clr(idx0*3+2), 1.0);
        clr_[i * 3 + 1] = glm::vec4(clr(idx1*3+0), clr(idx1*3+1), clr(idx1*3+2), 1.0);
        clr_[i * 3 + 2] = glm::vec4(clr(idx2*3+0), clr(idx2*3+1), clr(idx2*3+2), 1.0);

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
    program.setAttributeData("v_color", clr_);
}

void glMesh::init(GLProgram& program,
                  const Eigen::VectorXf& pts,
                  const Eigen::MatrixX3f& nml,
                  const Eigen::MatrixX3i& tri_pts)
{
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_normal", DataType::VECTOR3, true);
    
    pts_.resize(tri_pts.size());
    nml_.resize(tri_pts.size());
    for(int i = 0; i < tri_pts.rows(); ++i)
    {
        const int& idx0 = tri_pts(i, 0);
        const int& idx1 = tri_pts(i, 1);
        const int& idx2 = tri_pts(i, 2);
        
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

void glMesh::init(GLProgram& program,
                  const Eigen::VectorXf& pts)
{
    program.createAttribute("v_position", DataType::VECTOR3, true);
    
    pts_.resize(pts.size()/3);
    
    for(int i = 0; i < pts_.size()/3; ++i)
    {
        pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
    }
    
    program.setAttributeData("v_position", pts_);
}

void glMesh::init(GLProgram& program,
                  const Eigen::VectorXf& pts,
                  const Eigen::Vector4f& clr)
{
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_color", DataType::VECTOR4, false);
    
    pts_.resize(pts.size()/3);
    clr_.resize(pts.size()/3);
    
    for(int i = 0; i < pts_.size()/3; ++i)
    {
        pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
        clr_[i] = glm::vec4(clr(0),clr(1),clr(2),clr(3));
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_color", clr_);    
}

void glMesh::init(GLProgram& program,
                  const std::vector<Eigen::Vector3f>& pts)
{
    program.createAttribute("v_position", DataType::VECTOR3, true);
    
    pts_.resize(pts.size());
    
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts[i](0),pts[i](1),pts[i](2));
    }
    
    program.setAttributeData("v_position", pts_);
}

void glMesh::init(GLProgram& program,
                  const std::vector<Eigen::Vector3f>& pts,
                  const Eigen::Vector4f& clr)
{
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_color", DataType::VECTOR4, false);
    
    pts_.resize(pts.size());
    clr_.resize(pts.size());
    
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts[i](0),pts[i](1),pts[i](2));
        clr_[i] = glm::vec4(clr(0),clr(1),clr(2),clr(3));
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_color", clr_);
}


void glMesh::init_with_idx(GLProgram& program,
                           const Eigen::VectorXf& pts,
                           const Eigen::MatrixX3f& nml,
                           const Eigen::MatrixX2f& uvs,
                           const Eigen::MatrixX3i& tri_pts,
                           const Eigen::MatrixX3i& tri_uv)
{
    assert(tri_pts.size() == tri_uv.size());
    
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_normal", DataType::VECTOR3, true);
    program.createAttribute("v_texcoord", DataType::VECTOR2, false);
    
    uvs_.resize(pts.size()/3);
    pts_.resize(pts.size()/3);
    nml_.resize(pts.size()/3);
    tri_.resize(tri_pts.size());
    
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
        nml_[i] = glm::vec3(nml(i,0),nml(i,1),nml(i,2));
    }
    
    for(int i = 0; i < tri_pts.rows(); ++i)
    {
        const int& idx0 = tri_pts(i, 0);
        const int& idx1 = tri_pts(i, 1);
        const int& idx2 = tri_pts(i, 2);
        
        const int& idxuv0 = tri_uv(i, 0);
        const int& idxuv1 = tri_uv(i, 1);
        const int& idxuv2 = tri_uv(i, 2);
        
        uvs_[idx0] = glm::vec2(uvs(idxuv0, 0), uvs(idxuv0, 1));
        uvs_[idx1] = glm::vec2(uvs(idxuv1, 0), uvs(idxuv1, 1));
        uvs_[idx2] = glm::vec2(uvs(idxuv2, 0), uvs(idxuv2, 1));
        
        tri_[i*3+0] = idx0;
        tri_[i*3+1] = idx1;
        tri_[i*3+2] = idx2;
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_normal", nml_);
    program.setAttributeData("v_texcoord", uvs_);
    
    program.createElementIndex(tri_);
}

void glMesh::init_with_idx(GLProgram& program,
                           const Eigen::VectorXf& pts,
                           const Eigen::MatrixX3f& nml,
                           const Eigen::MatrixX3i& tri_pts)
{
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_normal", DataType::VECTOR3, true);
    
    pts_.resize(pts.size()/3);
    nml_.resize(pts.size()/3);
    tri_.resize(tri_pts.size());
    
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
        nml_[i] = glm::vec3(nml(i,0),nml(i,1),nml(i,2));
    }
    
    for(int i = 0; i < tri_pts.rows(); ++i)
    {
        const int& idx0 = tri_pts(i, 0);
        const int& idx1 = tri_pts(i, 1);
        const int& idx2 = tri_pts(i, 2);
        
        tri_[i*3+0] = idx0;
        tri_[i*3+1] = idx1;
        tri_[i*3+2] = idx2;
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_normal", nml_);
    
    program.createElementIndex(tri_);
}

void glMesh::init_with_idx(GLProgram& program,
                           const Eigen::VectorXf& pts,
                           const Eigen::VectorXf& clr,
                           const Eigen::MatrixX3f& nml,
                           const Eigen::MatrixX3i& tri_pts)
{
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_normal", DataType::VECTOR3, true);
    program.createAttribute("v_color", DataType::VECTOR4, true);
    
    pts_.resize(pts.size()/3);
    nml_.resize(pts.size()/3);
    clr_.resize(pts.size()/3);
    tri_.resize(tri_pts.size());
    
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
        nml_[i] = glm::vec3(nml(i,0),nml(i,1),nml(i,2));
        clr_[i] = glm::vec4(clr(i*3+0),clr(i*3+1),clr(i*3+2),1.0);
    }
    
    for(int i = 0; i < tri_pts.rows(); ++i)
    {
        const int& idx0 = tri_pts(i, 0);
        const int& idx1 = tri_pts(i, 1);
        const int& idx2 = tri_pts(i, 2);
        
        tri_[i*3+0] = idx0;
        tri_[i*3+1] = idx1;
        tri_[i*3+2] = idx2;
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_normal", nml_);
    program.setAttributeData("v_color", clr_);
    
    program.createElementIndex(tri_);
}

void glMesh::init_with_idx(GLProgram& program,
                           const Eigen::VectorXf& pts,
                           const Eigen::VectorXf& clr,
                           const Eigen::MatrixX3f& nml,
                           const Eigen::MatrixX2f& uvs,
                           const Eigen::MatrixX3i& tri_pts,
                           const Eigen::MatrixX3i& tri_uv)
{
    assert(tri_pts.size() == tri_uv.size());
    
    program.createAttribute("v_position", DataType::VECTOR3, true);
    program.createAttribute("v_normal", DataType::VECTOR3, true);
    program.createAttribute("v_texcoord", DataType::VECTOR2, false);
    program.createAttribute("v_color", DataType::VECTOR4, true);
    
    uvs_.resize(pts.size()/3);
    pts_.resize(pts.size()/3);
    nml_.resize(pts.size()/3);
    clr_.resize(pts.size()/3);
    tri_.resize(tri_pts.size());
    
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
        nml_[i] = glm::vec3(nml(i,0),nml(i,1),nml(i,2));
        clr_[i] = glm::vec4(clr(i*3+0),clr(i*3+1),clr(i*3+2),1.0);
    }
    
    for(int i = 0; i < tri_pts.rows(); ++i)
    {
        const int& idx0 = tri_pts(i, 0);
        const int& idx1 = tri_pts(i, 1);
        const int& idx2 = tri_pts(i, 2);
        
        const int& idxuv0 = tri_uv(i, 0);
        const int& idxuv1 = tri_uv(i, 1);
        const int& idxuv2 = tri_uv(i, 2);
        
        uvs_[idx0] = glm::vec2(uvs(idxuv0, 0), uvs(idxuv0, 1));
        uvs_[idx1] = glm::vec2(uvs(idxuv1, 0), uvs(idxuv1, 1));
        uvs_[idx2] = glm::vec2(uvs(idxuv2, 0), uvs(idxuv2, 1));
        
        tri_[i*3+0] = idx0;
        tri_[i*3+1] = idx1;
        tri_[i*3+2] = idx2;
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_normal", nml_);
    program.setAttributeData("v_texcoord", uvs_);
    program.setAttributeData("v_color", clr_);
    
    program.createElementIndex(tri_);
}

void glMesh::update(GLProgram& program,
                    const std::vector<Eigen::Vector3f>& pts)
{
    pts_.resize(pts.size());
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts[i](0),pts[i](1),pts[i](2));
    }
    
    program.setAttributeData("v_position", pts_);
}

void glMesh::update(GLProgram& program,
                    const std::vector<Eigen::Vector3f>& pts,
                    const Eigen::Vector4f& clr)
{
    pts_.resize(pts.size());
    clr_.resize(pts.size());
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts[i](0),pts[i](1),pts[i](2));
        clr_[i] = glm::vec4(clr(0),clr(1),clr(2),clr(3));
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_color", clr_);
}


void glMesh::update(GLProgram& program,
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

void glMesh::update(GLProgram& program,
                    const Eigen::VectorXf& pts,
                    const Eigen::VectorXf& clr,
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
        
        clr_[i * 3 + 0] = glm::vec4(clr(idx0*3+0), clr(idx0*3+1), clr(idx0*3+2), 1.0);
        clr_[i * 3 + 1] = glm::vec4(clr(idx1*3+0), clr(idx1*3+1), clr(idx1*3+2), 1.0);
        clr_[i * 3 + 2] = glm::vec4(clr(idx2*3+0), clr(idx2*3+1), clr(idx2*3+2), 1.0);

        nml_[i * 3 + 0] = glm::vec3(nml(idx0, 0), nml(idx0, 1), nml(idx0, 2));
        nml_[i * 3 + 1] = glm::vec3(nml(idx1, 0), nml(idx1, 1), nml(idx1, 2));
        nml_[i * 3 + 2] = glm::vec3(nml(idx2, 0), nml(idx2, 1), nml(idx2, 2));
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_color", clr_);
    program.setAttributeData("v_normal", nml_);
}

void glMesh::update_with_idx(GLProgram& program,
                             const Eigen::VectorXf& pts,
                             const Eigen::MatrixX3f& nml)
{
    assert(pts_.size() == pts.size()/3);
    assert(nml_.size() == nml.size()/3);
    
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
        nml_[i] = glm::vec3(nml(i,0),nml(i,1),nml(i,2));
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_normal", nml_);
}

void glMesh::update_with_idx(GLProgram& program,
                             const Eigen::VectorXf& pts,
                             const Eigen::VectorXf& clr,
                             const Eigen::MatrixX3f& nml)
{
    assert(pts_.size() == pts.size()/3);
    assert(nml_.size() == nml.size()/3);
    assert(clr_.size() == clr.size()/3);
    
    for(int i = 0; i < pts_.size(); ++i)
    {
        pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
        nml_[i] = glm::vec3(nml(i,0),nml(i,1),nml(i,2));
        clr_[i] = glm::vec4(clr(i*3+0),clr(i*3+1),clr(i*3+2),1.0);
    }
    
    program.setAttributeData("v_position", pts_);
    program.setAttributeData("v_normal", nml_);
    program.setAttributeData("v_color", clr_);
}
