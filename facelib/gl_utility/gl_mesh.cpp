//
//  gl_mesh.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gl_mesh.h"

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

void glPoint2D::update(GLProgram& program,
                       int width,
                       int height,
                       const std::vector<Eigen::Vector3f>& pts)
{
    pts_.clear();
    clr_.clear();
    
    for(int i = 0; i < pts.size(); ++i)
    {
        pts_.push_back(glm::vec2(pts[i](0)/(float)width,pts[i](1)/(float)height));
        clr_.push_back(glm::vec4(1.0-pts[i](2),pts[i](2),0.0,1.0));
    }
    
    program.setAttributeData("v_p2d", pts_);
    program.setAttributeData("v_color", clr_);
}

void glSphere::generateSphere(float radius, unsigned int rings, unsigned int sectors, bool with_idx)
{
    float const R = 1./(float)(rings-1);
    float const S = 1./(float)(sectors-1);
    int r, s;
    
    std::vector<glm::vec3> pts(rings * sectors);
    std::vector<glm::vec3> nml(rings * sectors);
    std::vector<glm::vec2> uvs(rings * sectors);
    std::vector<glm::vec4> clr(rings * sectors);
    std::vector<unsigned int> tri;
    
    std::vector<glm::vec3>::iterator v = pts.begin();
    std::vector<glm::vec3>::iterator n = nml.begin();
    std::vector<glm::vec2>::iterator t = uvs.begin();
    std::vector<glm::vec4>::iterator c = clr.begin();
    for(r = 0; r < rings; r++) for(s = 0; s < sectors; s++) {
        float const y = sin( -M_PI_2 + M_PI * r * R );
        float const x = cos(2.0*M_PI * s * S) * sin( M_PI * r * R );
        float const z = sin(2.0*M_PI * s * S) * sin( M_PI * r * R );
        
        (*t)[0] = s*S;
        (*t)[1] = r*R;
        *t++;
        
        (*v)[0] = x * radius;
        (*v)[1] = y * radius;
        (*v)[2] = z * radius;
        *v++;
        
        (*n)[0] = x;
        (*n)[1] = y;
        (*n)[2] = z;
        *n = glm::normalize(*n);
        *n++;
        
        (*c) = glm::vec4(1.0,1.0,1.0,1.0);
        *c++;
        
        int curRow = r * sectors;
        int nextRow = (r+1) * sectors;
        
        tri.push_back(curRow + s);
        tri.push_back(nextRow + s);
        tri.push_back(nextRow + (s+1));
        
        tri.push_back(curRow + s);
        tri.push_back(nextRow + (s+1));
        tri.push_back(curRow + (s+1));
    }
    
    if(with_idx){
        pts_ = pts;
        nml_ = nml;
        uvs_ = uvs;
        clr_ = clr;
        tri_ = tri;
    }
    else{
        pts_.clear();
        nml_.clear();
        uvs_.clear();
        clr_.clear();
        tri_.clear();
        for(int i : tri)
        {
            pts_.push_back(pts[i]);
            nml_.push_back(nml[i]);
            uvs_.push_back(uvs[i]);
            clr_.push_back(clr[i]);
        }
    }
}

void glSphere::init(GLProgram& program, int flag)
{
    if(flag & AT_POSITION)
        program.createAttribute("v_position", DataType::VECTOR3, true);
    if(flag & AT_NORMAL)
        program.createAttribute("v_normal", DataType::VECTOR3, true);
    if(flag & AT_COLOR)
        program.createAttribute("v_color", DataType::VECTOR4, true);
    if(flag & AT_UV)
        program.createAttribute("v_texcoord", DataType::VECTOR2, false);
    if(flag & AT_TRI)
        program.createElementIndex(tri_);
}

void glSphere::update(GLProgram& program, int flag)
{
    if(flag & AT_POSITION)
        program.setAttributeData("v_position", pts_);
    if(flag & AT_NORMAL)
        program.setAttributeData("v_normal", nml_);
    if(flag & AT_COLOR)
        program.setAttributeData("v_color", clr_);
    if(flag & AT_UV)
        program.setAttributeData("v_texcoord", uvs_);
    if(flag & AT_TRI)
        program.updateElementIndex(tri_);
}

void glMesh::init(GLProgram& program, int flag)
{
    if(flag & AT_POSITION)
        program.createAttribute("v_position", DataType::VECTOR3, true);
    if(flag & AT_NORMAL)
        program.createAttribute("v_normal", DataType::VECTOR3, true);
    if(flag & AT_COLOR)
        program.createAttribute("v_color", DataType::VECTOR4, true);
    if(flag & AT_UV)
        program.createAttribute("v_texcoord", DataType::VECTOR2, false);
    if(flag & AT_TAN){
        program.createAttribute("v_tangent", DataType::VECTOR3, true);
        program.createAttribute("v_bitangent", DataType::VECTOR3, true);
    }
    if(flag & AT_TRI)
        program.createElementIndex(tri_);
}

void glMesh::update(GLProgram& program, int flag)
{
    if(flag & AT_POSITION)
        program.setAttributeData("v_position", pts_);
    if(flag & AT_NORMAL)
        program.setAttributeData("v_normal", nml_);
    if(flag & AT_COLOR)
        program.setAttributeData("v_color", clr_);
    if(flag & AT_UV)
        program.setAttributeData("v_texcoord", uvs_);
    if(flag & AT_TAN){
        program.setAttributeData("v_tangent", tan_);
        program.setAttributeData("v_bitangent", btan_);
    }
    if(flag & AT_TRI)
        program.updateElementIndex(tri_);
}

void glMesh::update_position(const Eigen::VectorXf& pts,
                             const Eigen::MatrixX3i& tri)
{
    if(tri.size() != 0){
        pts_.resize(tri.size());
        for(int i = 0; i < tri.rows(); ++i)
        {
            const int& idx0 = tri(i, 0);
            const int& idx1 = tri(i, 1);
            const int& idx2 = tri(i, 2);
            
            pts_[i * 3 + 0] = glm::vec3(pts(idx0*3+0), pts(idx0*3+1), pts(idx0*3+2));
            pts_[i * 3 + 1] = glm::vec3(pts(idx1*3+0), pts(idx1*3+1), pts(idx1*3+2));
            pts_[i * 3 + 2] = glm::vec3(pts(idx2*3+0), pts(idx2*3+1), pts(idx2*3+2));
        }
    }
    else{
        pts_.resize(pts.size()/3);
        for(int i = 0; i < pts_.size(); ++i)
        {
            pts_[i] = glm::vec3(pts(i*3+0),pts(i*3+1),pts(i*3+2));
        }
    }
}

void glMesh::update_position(const std::vector<Eigen::Vector3f>& pts,
                             const Eigen::MatrixX3i& tri)
{
    if(tri.size() != 0){
        pts_.resize(tri.size());
        for(int i = 0; i < tri.rows(); ++i)
        {
            const int& idx0 = tri(i, 0);
            const int& idx1 = tri(i, 1);
            const int& idx2 = tri(i, 2);
            
            pts_[i * 3 + 0] = glm::vec3(pts[idx0](0), pts[idx0](1), pts[idx0](2));
            pts_[i * 3 + 1] = glm::vec3(pts[idx1](0), pts[idx1](1), pts[idx1](2));
            pts_[i * 3 + 2] = glm::vec3(pts[idx2](0), pts[idx2](1), pts[idx2](2));
        }
    }
    else{
        pts_.resize(pts.size());
        for(int i = 0; i < pts_.size(); ++i)
        {
            pts_[i] = glm::vec3(pts[i](0),pts[i](1),pts[i](2));
        }
    }
}

void glMesh::update_normal(const Eigen::MatrixX3f& nml,
                           const Eigen::MatrixX3i& tri)
{
    if(tri.size() != 0){
        nml_.resize(tri.size());
        for(int i = 0; i < tri.rows(); ++i)
        {
            const int& idx0 = tri(i, 0);
            const int& idx1 = tri(i, 1);
            const int& idx2 = tri(i, 2);
            
            nml_[i * 3 + 0] = glm::vec3(nml(idx0, 0), nml(idx0, 1), nml(idx0, 2));
            nml_[i * 3 + 1] = glm::vec3(nml(idx1, 0), nml(idx1, 1), nml(idx1, 2));
            nml_[i * 3 + 2] = glm::vec3(nml(idx2, 0), nml(idx2, 1), nml(idx2, 2));
        }
    }
    else{
        nml_.resize(nml.rows());
        for(int i = 0; i < nml_.size(); ++i)
        {
            nml_[i] = glm::vec3(nml(i, 0),nml(i, 1),nml(i, 2));
        }
    }
}

void glMesh::update_color(const Eigen::VectorXf& clr,
                          const Eigen::MatrixX3i& tri)
{
    if(tri.size() != 0){
        clr_.resize(tri.size());
        for(int i = 0; i < tri.rows(); ++i)
        {
            const int& idx0 = tri(i, 0);
            const int& idx1 = tri(i, 1);
            const int& idx2 = tri(i, 2);
            
            clr_[i * 3 + 0] = glm::vec4(clr(idx0*3+0), clr(idx0*3+1), clr(idx0*3+2), 1.0);
            clr_[i * 3 + 1] = glm::vec4(clr(idx1*3+0), clr(idx1*3+1), clr(idx1*3+2), 1.0);
            clr_[i * 3 + 2] = glm::vec4(clr(idx2*3+0), clr(idx2*3+1), clr(idx2*3+2), 1.0);
        }
    }
    else{
        clr_.resize(clr.size()/3);
        for(int i = 0; i < clr_.size(); ++i)
        {
            clr_[i] = glm::vec4(clr(i*3+0),clr(i*3+1),clr(i*3+2), 1.0);
        }
    }
}

void glMesh::update_color(const Eigen::Vector4f& clr,
                          const Eigen::MatrixX3i& tri)
{
    if(tri.size() != 0){
        clr_.resize(tri.size());
        for(int i = 0; i < tri.rows(); ++i)
        {
            const int& idx0 = tri(i, 0);
            const int& idx1 = tri(i, 1);
            const int& idx2 = tri(i, 2);
            
            clr_[i * 3 + 0] = glm::vec4(clr(0), clr(1), clr(2), clr(3));
            clr_[i * 3 + 1] = glm::vec4(clr(0), clr(1), clr(2), clr(3));
            clr_[i * 3 + 2] = glm::vec4(clr(0), clr(1), clr(2), clr(3));
        }
    }
    else{
        clr_.resize(pts_.size());
        for(int i = 0; i < pts_.size(); ++i)
        {
            clr_[i] = glm::vec4(clr(0), clr(1), clr(2), clr(3));
        }
    }
}

void glMesh::update_tangent(const Eigen::MatrixX3f& tan,
                            const Eigen::MatrixX3f& btan,
                            const Eigen::MatrixX3i& tri)
{
    assert(tan.size() == btan.size());
    
    if(tri.size() != 0){
        tan_.resize(tri.size());
        btan_.resize(tri.size());
        for(int i = 0; i < tri.rows(); ++i)
        {
            const int& idx0 = tri(i, 0);
            const int& idx1 = tri(i, 1);
            const int& idx2 = tri(i, 2);
            
            tan_[i * 3 + 0] = glm::vec3(tan(idx0,0), tan(idx0,1), tan(idx0,2));
            tan_[i * 3 + 1] = glm::vec3(tan(idx1,0), tan(idx1,1), tan(idx1,2));
            tan_[i * 3 + 2] = glm::vec3(tan(idx2,0), tan(idx2,1), tan(idx2,2));

            btan_[i * 3 + 0] = glm::vec3(btan(idx0,0), btan(idx0,1), btan(idx0,2));
            btan_[i * 3 + 1] = glm::vec3(btan(idx1,0), btan(idx1,1), btan(idx1,2));
            btan_[i * 3 + 2] = glm::vec3(btan(idx2,0), btan(idx2,1), btan(idx2,2));
        }
    }
    else{
        tan_.resize(tan.rows());
        btan_.resize(btan.rows());
        for(int i = 0; i < tan_.size(); ++i)
        {
            tan_[i] = glm::vec3(tan(i,0), tan(i,1), tan(i,2));
            btan_[i] = glm::vec3(btan(i,0), btan(i,1), btan(i,2));
        }
    }
}

void glMesh::update_uv(const Eigen::MatrixX2f& uvs,
                       const Eigen::MatrixX3i& tri_uv,
                       const Eigen::MatrixX3i& tri_pts)
{
    if(tri_pts.size() != 0){
        assert(tri_uv.size() == tri_pts.size());
        assert(pts_.size() != 0);
        uvs_.resize(pts_.size());
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
        }
    }
    else{
        uvs_.resize(tri_uv.size());
        for(int i = 0; i < tri_uv.rows(); ++i)
        {
            const int& idx0 = tri_uv(i, 0);
            const int& idx1 = tri_uv(i, 1);
            const int& idx2 = tri_uv(i, 2);
            
            uvs_[i * 3 + 0] = glm::vec2(uvs(idx0, 0), uvs(idx0, 1));
            uvs_[i * 3 + 1] = glm::vec2(uvs(idx1, 0), uvs(idx1, 1));
            uvs_[i * 3 + 2] = glm::vec2(uvs(idx2, 0), uvs(idx2, 1));
        }
    }
}

void glMesh::update_tri(const Eigen::MatrixX3i& tri)
{
    tri_.resize(tri.size());
    
    for(int i = 0; i < tri.rows(); ++i)
    {
        const int& idx0 = tri(i, 0);
        const int& idx1 = tri(i, 1);
        const int& idx2 = tri(i, 2);
        
        tri_[i*3+0] = idx0;
        tri_[i*3+1] = idx1;
        tri_[i*3+2] = idx2;
    }
}
