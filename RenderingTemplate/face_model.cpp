//
//  face_model.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include <iostream>
#include <vector>

#include "face_model.hpp"
#include "obj_loader.hpp"

static void computeEdgeBasis(std::vector<std::array<Eigen::Matrix3Xf, 2>>& id_edge,
                             std::vector<std::array<Eigen::Matrix3Xf, 2>>& ex_edge,
                             const Eigen::MatrixXf& w_id,
                             const Eigen::MatrixXf& w_ex,
                             const Eigen::MatrixX3i& tri,
                             int ID,
                             int EX)
{
    id_edge.resize(tri.rows());
    ex_edge.resize(tri.rows());
    
    for (int i = 0; i < tri.rows(); ++i)
    {
        if (ID != 0){
            id_edge[i][0] = w_id.block(3 * tri(i,1), 0, 3, ID) - w_id.block(3 * tri(i,0), 0, 3, ID);
            id_edge[i][1] = w_id.block(3 * tri(i,2), 0, 3, ID) - w_id.block(3 * tri(i,0), 0, 3, ID);
        }
        
        if (EX != 0){
            ex_edge[i][0] = w_ex.block(3 * tri(i,1), 0, 3, EX) - w_ex.block(3 * tri(i,0), 0, 3, EX);
            ex_edge[i][1] = w_ex.block(3 * tri(i,2), 0, 3, EX) - w_ex.block(3 * tri(i,0), 0, 3, EX);
        }
    }
}

void FaceParams::updateIdentity(const FaceModel& model)
{
    assert(model.w_id_.cols() == idCoeff.size());
    d_id_ = model.w_id_ * idCoeff;
    neu_ = model.mu_id_ + d_id_;
}

void FaceParams::updateExpression(const FaceModel& model)
{
    assert(model.w_ex_.cols() == exCoeff.size());

    d_ex_ = model.w_ex_ * exCoeff;
    pts_ = neu_ + d_ex_;
    
    calcNormal(nml_, pts_, model.tri_pts_);
}

void FaceParams::updateColor(const FaceModel& model)
{
    assert(model.w_cl_.cols() == alCoeff.size());

    clr_ = model.mu_cl_ + model.w_cl_ * alCoeff;
}

void FaceParams::updateAll(const FaceModel &model)
{
    updateColor(model);
    updateIdentity(model);
    updateExpression(model);
}

void FaceParams::updateShape(const FaceModel& model)
{
    updateIdentity(model);
    updateExpression(model);
}

Eigen::Vector3f FaceParams::computeV(int i, const FaceModel& model) const
{
    assert(model.w_id_.cols() == idCoeff.size());
    assert(model.w_ex_.cols() == exCoeff.size());
    
    Eigen::Vector3f p = model.mu_id_.b3(i) + model.w_id_.block(i*3, 0, 3, idCoeff.size()) * idCoeff;
    p += model.mu_ex_.b3(i) + model.w_ex_.block(i*3, 0, 3, exCoeff.size()) * exCoeff;
    return p;
}

void FaceParams::init(const FaceModel& model)
{
    idCoeff = Eigen::VectorXf::Zero(model.sigma_id_.size());
    exCoeff = Eigen::VectorXf::Zero(model.sigma_ex_.size());
    alCoeff = Eigen::VectorXf::Zero(model.sigma_cl_.size());
    SH = Eigen::Matrix3Xf::Zero(3,9);
    SH.col(0).setOnes();
    
    updateColor(model);
    updateIdentity(model);
    updateExpression(model);
}

void FaceModel::saveBinaryModel(std::string file)
{
    FILE* fp;
    fp = fopen(file.c_str(), "wb");

    long tmp;
    tmp = w_id_.rows();
    fwrite(&tmp, sizeof(long), 1, fp);
    tmp = w_id_.cols();
    fwrite(&tmp, sizeof(long), 1, fp);
    fwrite(&mu_id_.data()[0], sizeof(float), mu_id_.size(), fp);
    fwrite(&sigma_id_.data()[0], sizeof(float), sigma_id_.size(), fp);
    fwrite(&w_id_.data()[0], sizeof(float), w_id_.size(), fp);
    
    tmp = w_ex_.rows();
    fwrite(&tmp, sizeof(long), 1, fp);
    tmp = w_ex_.cols();
    fwrite(&tmp, sizeof(long), 1, fp);
    fwrite(&mu_ex_.data()[0], sizeof(float), mu_ex_.size(), fp);
    fwrite(&sigma_ex_.data()[0], sizeof(float), sigma_ex_.size(), fp);
    fwrite(&w_ex_.data()[0], sizeof(float), w_ex_.size(), fp);
    
    tmp = w_cl_.rows();
    fwrite(&tmp, sizeof(long), 1, fp);
    tmp = w_cl_.cols();
    fwrite(&tmp, sizeof(long), 1, fp);
    fwrite(&mu_cl_.data()[0], sizeof(float), mu_cl_.size(), fp);
    fwrite(&sigma_cl_.data()[0], sizeof(float), sigma_cl_.size(), fp);
    fwrite(&w_cl_.data()[0], sizeof(float), w_cl_.size(), fp);
    
    tmp = uvs_.rows();
    fwrite(&tmp, sizeof(long), 1, fp);
    fwrite(&uvs_.data()[0], sizeof(float), uvs_.size(), fp);
    
    assert(tri_pts_.size() == tri_uv_.size());
    tmp = tri_pts_.rows();
    fwrite(&tmp, sizeof(long), 1, fp);
    fwrite(&tri_pts_.data()[0], sizeof(int), tri_pts_.size(), fp);
    fwrite(&tri_uv_.data()[0], sizeof(int), tri_uv_.size(), fp);
    
    tmp = sym_list_.rows();
    fwrite(&tmp, sizeof(long), 1, fp);
    fwrite(&sym_list_.data()[0], sizeof(int), sym_list_.size(), fp);
    
    fclose(fp);
}

void FaceModel::loadBinaryModel(std::string file)
{
    FILE* fp;
    fp = fopen(file.c_str(), "rb");
    
    long r,c;
    fread(&r, sizeof(long), 1, fp);
    fread(&c, sizeof(long), 1, fp);
    
    mu_id_.resize(r);
    sigma_id_.resize(c);
    w_id_.resize(r,c);
    fread(&mu_id_.data()[0], sizeof(float), mu_id_.size(), fp);
    fread(&sigma_id_.data()[0], sizeof(float), sigma_id_.size(), fp);
    fread(&w_id_.data()[0], sizeof(float), w_id_.size(), fp);

    fread(&r, sizeof(long), 1, fp);
    fread(&c, sizeof(long), 1, fp);
    mu_ex_.resize(r);
    sigma_ex_.resize(c);
    w_ex_.resize(r,c);
    fread(&mu_ex_.data()[0], sizeof(float), mu_ex_.size(), fp);
    fread(&sigma_ex_.data()[0], sizeof(float), sigma_ex_.size(), fp);
    fread(&w_ex_.data()[0], sizeof(float), w_ex_.size(), fp);

    fread(&r, sizeof(long), 1, fp);
    fread(&c, sizeof(long), 1, fp);
    mu_cl_.resize(r);
    sigma_cl_.resize(c);
    w_cl_.resize(r,c);
    fread(&mu_cl_.data()[0], sizeof(float), mu_cl_.size(), fp);
    fread(&sigma_cl_.data()[0], sizeof(float), sigma_cl_.size(), fp);
    fread(&w_cl_.data()[0], sizeof(float), w_cl_.size(), fp);
    
    fread(&r, sizeof(long), 1, fp);
    uvs_.resize(r,2);
    fread(&uvs_.data()[0], sizeof(float), uvs_.size(), fp);
    
    fread(&r, sizeof(long), 1, fp);
    tri_uv_.resize(r, 3);
    tri_pts_.resize(r, 3);
    fread(&tri_pts_.data()[0], sizeof(int), tri_pts_.size(), fp);
    fread(&tri_uv_.data()[0], sizeof(int), tri_uv_.size(), fp);
    
    fread(&r, sizeof(long), 1, fp);
    sym_list_.resize(r, 2);
    fread(&sym_list_.data()[0], sizeof(int), sym_list_.size(), fp);
    
    fclose(fp);
    
    computeEdgeBasis(id_edge_, ex_edge_, w_id_, w_ex_, tri_pts_, (int)sigma_id_.size(), (int)sigma_ex_.size());
}

void FaceModel::loadOldBinaryModel(std::string modelfile, std::string meshfile)
{
    FILE* fp;
    fp = fopen(modelfile.c_str(), "rb");
    
    if ( fp == NULL )
    {   std::cout << modelfile << std::endl;
        return;
    }
    
    int property[8];
    // 0: #vertices, 1:#triangles, 2: #identities, 3: #expression, 4: #symlist, 5: #symlist_tri, 6: #keypoints, 7: #segbin_tri
    fread(&property[0], sizeof(int), 8, fp);
    
    // load triangle list
    std::vector<int> tmp_tri(property[1]*3);
    fread(&tmp_tri[0], sizeof(int), property[1]*3, fp);
    
    // load segmentation bin
    std::vector<int> tmp_segbin(property[0] * 4);
    std::vector<int> tmp_segbintri(property[7] * 4);
    if (property[7] != 0){
        fread(&tmp_segbin[0], sizeof(int), property[0] * 4, fp);
        fread(&tmp_segbintri[0], sizeof(int), property[7] * 4, fp);
    }
    
    // load symmetry vertices list
    std::vector<int> sym_list(property[4]*2);
    if (property[4] != 0){
        fread(&sym_list[0], sizeof(int), property[4] * 2, fp);
    }
    sym_list_.resize(property[4],2);
    for(int i = 0; i < property[4]; ++i)
    {
        sym_list_(i, 0) = sym_list[i*2+0];
        sym_list_(i, 1) = sym_list[i*2+1];
    }
    
    // load symmetry triangles list
    std::vector<int> tmp_sym_tri(property[5]*2);
    if (property[5] != 0){
        fread(&tmp_sym_tri[0], sizeof(int), property[5] * 2, fp);
    }
    
    // load key points
    std::vector<int> key_points(property[6]);
    fread(&key_points[0], sizeof(int), property[6], fp);
    
    std::vector<int> trimIndex(property[0]);
    fread(&trimIndex[0], sizeof(int), property[0], fp);
    
    std::vector<int> map_tri(property[0]*2,-1);
    int x = 0;
    
    for(int i = 0; i < trimIndex.size(); ++i)
    {
        map_tri[trimIndex[i]] = x++;
    }
    assert(x == property[0]);
    
    // load mean for shape, expression, and albedo
    mu_id_.resize(property[0]*3);
    mu_ex_.resize(property[0]*3);
    mu_cl_.resize(property[0]*3);
    
    fread(&mu_id_.data()[0], sizeof(float), mu_id_.size(), fp);
    fread(&mu_ex_.data()[0], sizeof(float), mu_ex_.size(), fp);
    fread(&mu_cl_.data()[0], sizeof(float), mu_cl_.size(), fp);
    
    // load standard deviation for shape, expression, and albedo
    sigma_id_.resize(property[2]);
    sigma_ex_.resize(property[3]);
    sigma_cl_.resize(property[2]);
    
    fread(&sigma_id_.data()[0], sizeof(float), sigma_id_.size(), fp);
    fread(&sigma_ex_.data()[0], sizeof(float), sigma_ex_.size(), fp);
    fread(&sigma_cl_.data()[0], sizeof(float), sigma_cl_.size(), fp);
    
    // load pca delta for shape, expression, and albedo
    w_id_.resize(property[0]*3,property[2]);
    w_ex_.resize(property[0]*3,property[3]);
    w_cl_.resize(property[0]*3, property[2]);
    
    fread(&w_id_.data()[0], sizeof(float), w_id_.size(), fp);
    fread(&w_ex_.data()[0], sizeof(float), w_ex_.size(), fp);
    fread(&w_cl_.data()[0], sizeof(float), w_cl_.size(), fp);
    
    // transforming shapeCoreTensor
    float scale = 0.1;
    Eigen::Matrix3f R = Eigen::Quaternion<float>(0,1,0,0).toRotationMatrix();
    for (int i = 0; i < property[0]; ++i)
    {
        mu_id_.b3(i) = scale*R*mu_id_.b3(i);
        mu_ex_.b3(i) = scale*R*mu_ex_.b3(i);
        w_id_.block(3 * i, 0, 3, property[2]) = scale*R*w_id_.block(3 * i, 0, 3, property[2]);
        w_ex_.block(3 * i, 0, 3, property[3]) = scale*R*w_ex_.block(3 * i, 0, 3, property[3]);
    }
    
    // normalizing albedo basis + rescale to [0,1]
    mu_cl_ *= 1.f / 255.f;
    w_cl_ *= 1.f / 255.f;
    
    fclose(fp);
    
    Eigen::VectorXf pts;
    Eigen::MatrixX3f nml;
    loadObjFile(meshfile, pts, nml, uvs_, tri_pts_, tri_uv_);
    
    computeEdgeBasis(id_edge_, ex_edge_, w_id_, w_ex_, tri_pts_, (int)sigma_id_.size(), (int)sigma_ex_.size());
}
