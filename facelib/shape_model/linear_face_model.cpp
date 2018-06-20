//
//  face_model.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include <fstream>
#include <iostream>
#include <vector>

#include <utility/obj_loader.h>

#include "face_model.h"

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

void LinearFaceModel::updateIdentity(FaceData& data)
{
    if(sigma_id_.size() == 0){
        data.neu_ = mu_id_;
        return;
    }

    assert(w_id_.cols() == data.idCoeff.size());
    data.d_id_ = w_id_ * data.idCoeff;
    data.neu_ = mu_id_ + data.d_id_;
}

void LinearFaceModel::updateExpression(FaceData& data)
{
    if(sigma_ex_.size() == 0){
        data.pts_ = data.neu_;
        calcNormal(data.nml_, data.pts_, tri_pts_);
        return;
    }
    assert(w_ex_.cols() == data.exCoeff.size());

    data.d_ex_ = w_ex_ * data.exCoeff;
    data.pts_ = data.neu_ + data.d_ex_;

    calcNormal(data.nml_, data.pts_, tri_pts_);
}

void LinearFaceModel::updateColor(FaceData& data)
{
    if(sigma_cl_.size() == 0){
        data.clr_ = Eigen::VectorXf::Ones(mu_id_.size());
        return;
    }
    assert(w_cl_.cols() == data.alCoeff.size());

    data.clr_ = mu_cl_ + w_cl_ * data.alCoeff;
}

Eigen::Vector3f LinearFaceModel::computeV(int vidx, const FaceData& data) const
{
    assert(w_id_.cols() == data.idCoeff.size());
    assert(w_ex_.cols() == data.exCoeff.size());
    
    Eigen::Vector3f p = mu_id_.b3(vidx) + w_id_.block(vidx*3, 0, 3, data.idCoeff.size()) * data.idCoeff;
    p += w_ex_.block(vidx*3, 0, 3, data.exCoeff.size()) * data.exCoeff;
    return p;
}

Eigen::Ref<const Eigen::MatrixXf> LinearFaceModel::dID(int vidx, int size, const FaceData& data) const
{
    assert(vidx < mu_id_.size()/3 && vidx >= 0);
    
    return w_id_.block(vidx*3, 0, 3, size);
}

Eigen::Ref<const Eigen::MatrixXf> LinearFaceModel::dEX(int vidx, int size, const FaceData& data) const
{
    assert(vidx < mu_id_.size()/3 && vidx >= 0);
    
    return w_ex_.block(vidx*3, 0, 3, size);
}
Eigen::Ref<const Eigen::MatrixXf> LinearFaceModel::dCL(int vidx, int size, const FaceData& data) const
{
    assert(vidx < mu_id_.size()/3 && vidx >= 0);

    return w_cl_.block(vidx*3, 0, 3, size);
}

const Eigen::Matrix3Xf& LinearFaceModel::dIDEdge(int fidx, int eidx, const FaceData& data) const
{
    assert(fidx < id_edge_.size() && fidx >= 0);
    assert(eidx == 0 || eidx == 1);
    
    return id_edge_[fidx][eidx];
}

const Eigen::Matrix3Xf& LinearFaceModel::dEXEdge(int fidx, int eidx, const FaceData& data) const
{
    assert(fidx < ex_edge_.size() && fidx >= 0);
    assert(eidx == 0 || eidx == 1);
    
    return ex_edge_[fidx][eidx];
}

void LinearFaceModel::dSym(int symidx, int axis, int nid, int nex, Eigen::Vector3f& v, Eigen::MatrixXf& dv, const FaceData& data) const
{
    assert(symidx < sym_list_.rows() && symidx >= 0);
    assert(axis == 0 || axis == 1 || axis == 2);
    int idx1 = sym_list_(symidx,0);
    int idx2 = sym_list_(symidx,1);
    
    const Eigen::Vector3f& p1id = data.d_id_.b3(idx1);
    const Eigen::Vector3f& p2id = data.d_id_.b3(idx2);
    const Eigen::Vector3f& p1ex = data.d_ex_.b3(idx1);
    const Eigen::Vector3f& p2ex = data.d_ex_.b3(idx2);
    
    if(dv.rows() != 3 || dv.cols() != nid + nex)
        dv.resize(3, nid+nex);
    
    for(int i = 0; i < 3; ++i)
    {
        if(i == axis){
            v[i] = p1id[i] + p2id[i];
            dv.block(i,0,1,nid) = w_id_.block(idx1*3+i, 0, 1, nid) + w_id_.block(idx2*3+i, 0, 1, nid);
            if(nex != 0){
                v[i] += p1ex[i] + p2ex[i];
                dv.block(i,nid,1,nex) += w_ex_.block(idx1*3+i, 0, 1, nex) + w_ex_.block(idx2*3+i, 0, 1, nex);
            }
        }
        else{
            v[i] = p1id[i] - p2id[i];
            dv.block(i,0,1,nid) = w_id_.block(idx1*3+i, 0, 1, nid) - w_id_.block(idx2*3+i, 0, 1, nid);
            if(nex != 0){
                v[i] += p1ex[i] - p2ex[i];
                dv.block(i,nid,1,nex) += w_ex_.block(idx1*3+i, 0, 1, nex) - w_ex_.block(idx2*3+i, 0, 1, nex);
            }
        }
    }
}

void LinearFaceModel::saveBinaryModel(const std::string& file)
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

void LinearFaceModel::loadBinaryModel(const std::string& file)
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

void LinearFaceModel::loadOldBinaryModel(const std::string& modelfile, const std::string& meshfile)
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
    float scale = 1.e-4;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();//Eigen::Quaternion<float>(0,1,0,0).toRotationMatrix();
    for (int i = 0; i < property[0]; ++i)
    {
        mu_id_.b3(i) = scale*R*mu_id_.b3(i);
        mu_ex_.b3(i) = scale*R*mu_ex_.b3(i);
        w_id_.block(3 * i, 0, 3, property[2]) = scale*R*w_id_.block(3 * i, 0, 3, property[2]);
        w_ex_.block(3 * i, 0, 3, property[3]) = scale*R*w_ex_.block(3 * i, 0, 3, property[3]);
    }
    
//    // normalizing albedo basis + rescale to [0,1]
//    w_id_ = w_id_*sigma_id_.asDiagonal();
//    sigma_id_.setConstant(1.0f);
//    
//    w_ex_ = 0.02f*w_ex_*sigma_ex_.asDiagonal();
//    sigma_ex_.setConstant(1.0f);
//    
//    // normalizing albedo basis + rescale to [0,1]
//    mu_cl_ *= 1.f / 255.f;
//    w_cl_ = 1.f / 255.f * w_cl_*sigma_cl_.asDiagonal();
//    sigma_cl_.setOnes();

    // normalizing albedo basis + rescale to [0,1]
    mu_cl_ *= 1.f / 255.f;
    w_cl_ *= 1.f / 255.f;
    
    fclose(fp);
    
    Eigen::VectorXf pts;
    Eigen::MatrixX3f nml;
    loadObjFile(meshfile, pts, nml, uvs_, tri_pts_, tri_uv_);
    
    computeEdgeBasis(id_edge_, ex_edge_, w_id_, w_ex_, tri_pts_, (int)sigma_id_.size(), (int)sigma_ex_.size());
}

void LinearFaceModel::loadMMBinaryModel(const std::string& modelfile)
{
    std::string binary_model_name = modelfile;
    
    FILE* fp;
    fp = fopen(binary_model_name.c_str(), "rb");
    
    if ( fp == NULL )
    {   std::cout << binary_model_name << std::endl;
        return;
    }
    
    int property[8];
    // 0: #vertices, 1:#triangles, 2: #identities, 3: #expression, 4: #symlist, 5: #symlist_tri, 6: #keypoints, 7: #segbin_tri
    fread(&property[0], sizeof(int), 8, fp);
    
    // load triangle list
    std::vector<int> tmp_tri(property[1]*3);
    fread(&tmp_tri[0], sizeof(int), property[1]*3, fp);
    
    // load segmentation bin
    std::vector<int> tmp_segbin(property[0]*4);
    fread(&tmp_segbin[0],sizeof(int), property[0]*4, fp);
    std::vector<int> tmp_segbintri(property[7]*4);
    fread(&tmp_segbintri[0], sizeof(int), property[7]*4, fp);
    
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
    
    // note: somehow the original triangle is fliped
    std::vector<int> tri_v;
    for(int i = 0; i < tmp_tri.size()/3; ++i)
    {
        if(map_tri[tmp_tri[i*3+0]] != -1 && map_tri[tmp_tri[i*3+1]] != -1 && map_tri[tmp_tri[i*3+2]] != -1){
            tri_v.push_back(map_tri[tmp_tri[i*3+2]]);
            tri_v.push_back(map_tri[tmp_tri[i*3+1]]);
            tri_v.push_back(map_tri[tmp_tri[i*3+0]]);
        }
    }
    
    tri_pts_.resize(tri_v.size()/3, 3);
    tri_uv_.resize(tri_v.size()/3, 3);
    for (size_t f = 0; f < tri_v.size()/3; f++)
    {
        tri_pts_(f,0) = tri_v[f*3+0];
        tri_pts_(f,1) = tri_v[f*3+1];
        tri_pts_(f,2) = tri_v[f*3+2];

        tri_uv_(f,0) = tri_pts_(f,0);
        tri_uv_(f,1) = tri_pts_(f,1);
        tri_uv_(f,2) = tri_pts_(f,2);
    }
    
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
    w_cl_.resize(property[0]*3,property[2]);
    
    fread(&w_id_.data()[0], sizeof(float), w_id_.size(), fp);
    fread(&w_ex_.data()[0], sizeof(float), w_ex_.size(), fp);
    fread(&w_cl_.data()[0], sizeof(float), w_cl_.size(), fp);
    
    uvs_.resize(property[0],2);
    std::vector<float> uvs(property[0]*2);
    fread(&uvs[0], sizeof(float), uvs_.size(), fp);
    for(int i = 0; i < uvs_.rows(); ++i)
    {
        uvs_(i,0) = uvs[i*2+0];
        uvs_(i,1) = uvs[i*2+1];
    }
    
    // transforming shapeCoreTensor
    float scale = 1.e-4;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();//Eigen::Quaternion<float>(0,1,0,0).toRotationMatrix();
    for (int i = 0; i < property[0]; ++i)
    {
        mu_id_.b3(i) = scale*R*mu_id_.b3(i);
        mu_ex_.b3(i) = scale*R*mu_ex_.b3(i);
        w_id_.block(3 * i, 0, 3, property[2]) = scale*R*w_id_.block(3 * i, 0, 3, property[2]);
        w_ex_.block(3 * i, 0, 3, property[3]) = scale*R*w_ex_.block(3 * i, 0, 3, property[3]);
    }
    
    // normalizing albedo basis + rescale to [0,1]
    w_id_ = w_id_*sigma_id_.asDiagonal();
    sigma_id_.setConstant(1.0f);

    w_ex_ = 0.02f*w_ex_*sigma_ex_.asDiagonal();
    sigma_ex_.setConstant(1.0f);

    // normalizing albedo basis + rescale to [0,1]
    mu_cl_ *= 1.f / 255.f;
    w_cl_ = 1.f / 255.f * w_cl_*sigma_cl_.asDiagonal();
    sigma_cl_.setOnes();
    
    fclose(fp);
    
    computeEdgeBasis(id_edge_, ex_edge_, w_id_, w_ex_, tri_pts_, (int)sigma_id_.size(), (int)sigma_ex_.size());
}

void LinearFaceModel::loadModelFromObj(const std::string& obj_dir, int id_size, int ex_size)
{
    Eigen::MatrixX3f nml;
    loadObjFile(obj_dir + "/neutral.obj", mu_id_, nml, uvs_, tri_pts_, tri_uv_);
    
    Eigen::MatrixXf w_id(mu_id_.size(),id_size);
    Eigen::MatrixXf w_ex(mu_id_.size(),ex_size);
    
    Eigen::VectorXf pts;
    Eigen::MatrixX2f uvs;
    Eigen::MatrixX3i tri_pts;
    Eigen::MatrixX3i tri_uv;
    for(int i = 0; i < id_size; ++i)
    {
        char tmp[256];
        sprintf(tmp, "/id%03d.obj", i);
        loadObjFile(obj_dir + tmp, pts, nml, uvs, tri_pts, tri_uv);
        w_id.col(i) = pts - mu_id_;
    }
    
    for(int i = 0; i < ex_size; ++i)
    {
        char tmp[256];
        sprintf(tmp, "/ex%03d.obj", i);
        loadObjFile(obj_dir + tmp, pts, nml, uvs, tri_pts, tri_uv);
        w_ex.col(i) = pts - mu_id_;
    }
    
    w_id_ = w_id;
    w_ex_ = w_ex;
    sigma_ex_.resize(ex_size);
    sigma_ex_.setOnes();
    sigma_id_.resize(id_size);
    sigma_id_.setOnes();
}

FaceModelPtr LinearFaceModel::LoadModel(const std::string& file, const std::string& fm_type)
{
    auto model = new LinearFaceModel();
    
    model->fm_type_ = fm_type;
    model->loadBinaryModel(file);
        
    std::cout << "Face Model Info:" << std::endl;
    std::cout << "#Vert: " << model->mu_id_.size()/3 << " #Tri: " << model->tri_pts_.rows() << std::endl;

    return FaceModelPtr(model);
}


