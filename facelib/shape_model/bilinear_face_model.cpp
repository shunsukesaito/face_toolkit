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
#include <utility/fw_utils.h>

#include "face_model.h"

// FIXME: current hack get crashed in multithreading env
static Eigen::MatrixXf g_did, g_dex;

void BiLinearFaceModel::updateIdentity(FaceData& data)
{
    if(sigma_id_.size() == 0) return;
    
    assert(Cshape_[0].cols() == data.idCoeff.size());
    
    data.neu_ = mu_id_ + Cshape_[0] * data.idCoeff;
    data.w_ex_ = w_mu_ex_;
    for(int i = 0; i < n_exp(); ++i)
    {
        data.w_ex_.col(i) += Cshape_[i+1] * data.idCoeff;
    }
}

void BiLinearFaceModel::updateExpression(FaceData& data)
{
    if(sigma_ex_.size() == 0) return;
    assert(data.w_ex_.cols() == data.exCoeff.size());
    
    data.d_ex_ = data.w_ex_ * data.exCoeff;
    data.pts_ = data.neu_ + data.d_ex_;
    
    if(data.opt_id_only_){
        if(data.w_id_.rows() != mu_id_.size() || data.w_id_.cols() != n_id())
            data.w_id_.resize(mu_id_.size(), n_id());
     
        data.w_id_.setZero();
        for(int i = 0; i < n_exp(); ++i)
            data.w_id_ += data.exCoeff[i]*Cshape_[i+1];
    }
    
    calcNormal(data.nml_, data.pts_, tri_pts_);
}

Eigen::Ref<const Eigen::MatrixXf> BiLinearFaceModel::dID(int vidx, int size, const FaceData& data) const
{
    if (vidx > mu_id_.size()/3 || vidx < 0){
        std::cerr << "BilinearFaceModel::dID() - vidx is invalid. " << vidx
        << " #v: " << mu_id_.size()/3 << "\n";
        throw std::runtime_error("BilinearFaceModel::dID() - invalid vidx");
    }
    if ( size < 0 || (data.opt_id_only_ && size > data.w_id_.cols())){
        std::cerr << "BilinearFaceModel::dID() - size is invalid. " << size
        << " Wid = [" << data.w_id_.rows() << "," << data.w_id_.cols() << "]\n";
        throw std::runtime_error("BilinearFaceModel::dID() - invalid size");
    }
    
    // faster computation if optimizing for identity only
    if(data.opt_id_only_)
        return data.w_id_.block(vidx*3, 0, 3, size);
    else{
        g_did = Eigen::MatrixXf::Zero(3,size);
        for(int i = 0; i < n_exp(); ++i)
        {
            g_did += data.exCoeff[i]*Cshape_[i+1].block(vidx*3,0,3,size);
        }
        return g_did;
    }
}

Eigen::Ref<const Eigen::MatrixXf> BiLinearFaceModel::dEX(int vidx, int size, const FaceData& data) const
{
    if (vidx > mu_id_.size()/3 || vidx < 0){
        std::cerr << "BilinearFaceModel::dEX() - vidx is invalid. " << vidx
        << " #v: " << mu_id_.size()/3 << "\n";
        throw std::runtime_error("BilinearFaceModel::dEX() - invalid vidx");
    }
    if (size < 0 || (data.opt_ex_only_ && size > data.w_ex_.cols())){
        std::cerr << "BilinearFaceModel::dEX() - size is invalid. " << size
        << " Wex = [" << data.w_ex_.rows() << "," << data.w_ex_.cols() << "]\n";
        throw std::runtime_error("BilinearFaceModel::dEX() - invalid size");
    }
    
    // faster computation if optimzing for expression only
    if(data.opt_ex_only_){
        g_dex = w_mu_ex_.block(vidx*3, 0, 3, size) + data.w_ex_.block(vidx*3, 0, 3, size);
        return g_dex;
    }
    else{
        // without static, the returned value could be destroyed in the middle
        g_dex = w_mu_ex_.block(vidx*3, 0, 3, size);
        for(int i = 0; i < size; ++i)
        {
            g_dex.block(0,i,3,1) += Cshape_[i+1].block(vidx*3,0,3,n_id())*data.idCoeff;
        }

        return g_dex;
    }
}

Eigen::Vector3f BiLinearFaceModel::computeV(int vidx, const FaceData& data) const
{
    Eigen::Vector3f p = mu_id_.b3(vidx) + w_mu_ex_.block(vidx*3, 0, 3, n_exp())*data.exCoeff;
    
    if(data.opt_id_only_)
        p += data.w_id_.block(vidx*3,0,3,n_id())*data.idCoeff;
    else if(data.opt_ex_only_)
        p += data.w_ex_.block(vidx*3,0,3,n_exp())*data.exCoeff;
    else{
        p += Cshape_[0].block(vidx*3,0,3,n_id())*data.idCoeff;
        for(int i = 0; i < n_exp(); ++i)
        {
            p += data.exCoeff[i]*Cshape_[i+1].block(vidx*3,0,3,n_id())*data.idCoeff;
        }
    }

    return p;
}

void BiLinearFaceModel::saveBinaryModel(const std::string& file)
{
    FILE* fp;
    fp = fopen(file.c_str(), "wb");
    
    const int n_exp = Cshape_.size();
    const int n_id = Cshape_[0].cols();
    const int n_v = Cshape_[0].rows();
    
    fwrite(&n_exp, sizeof(int), 1, fp);
    fwrite(&n_v, sizeof(int), 1, fp);
    fwrite(&n_id, sizeof(int), 1, fp);
    for(int i = 0; i < Cshape_.size(); ++i)
    {
        fwrite(&Cshape_[i].data()[0], sizeof(float), Cshape_[i].size(), fp);
    }
    
    assert(mu_id_.size() == n_v);
    assert(sigma_id_.size() == n_id);
    assert(w_mu_ex_.rows() == n_v);
    assert(w_mu_ex_.cols() == n_exp-1);
    assert(sigma_ex_.size() == n_exp-1);
    
    fwrite(&mu_id_.data()[0], sizeof(float), mu_id_.size(), fp);
    fwrite(&sigma_id_.data()[0], sizeof(float), sigma_id_.size(), fp);

    fwrite(&w_mu_ex_.data()[0], sizeof(float), w_mu_ex_.size(), fp);
    fwrite(&sigma_ex_.data()[0], sizeof(float), sigma_ex_.size(), fp);
    
    int tmp = uvs_.rows();
    fwrite(&tmp, sizeof(int), 1, fp);
    fwrite(&uvs_.data()[0], sizeof(float), uvs_.size(), fp);
    
    assert(tri_pts_.size() == tri_uv_.size());
    tmp = tri_pts_.rows();
    fwrite(&tmp, sizeof(int), 1, fp);
    fwrite(&tri_pts_.data()[0], sizeof(int), tri_pts_.size(), fp);
    fwrite(&tri_uv_.data()[0], sizeof(int), tri_uv_.size(), fp);
    
    fclose(fp);
}

void BiLinearFaceModel::loadBinaryModel(const std::string& file)
{
    FILE* fp;
    fp = fopen(file.c_str(), "rb");
    
    int n_exp, n_id, n_v;
    fread(&n_exp, sizeof(int), 1, fp);
    fread(&n_v, sizeof(int), 1, fp);
    fread(&n_id, sizeof(int), 1, fp);
    Cshape_.resize(n_exp);
    for(int i = 0; i < Cshape_.size(); ++i)
    {
        Cshape_[i].resize(n_v, n_id);
        fread(&Cshape_[i].data()[0], sizeof(float), Cshape_[i].size(), fp);
    }
    
    mu_id_.resize(n_v);
    fread(&mu_id_.data()[0], sizeof(float), mu_id_.size(), fp);
    sigma_id_.resize(n_id);
    fread(&sigma_id_.data()[0], sizeof(float), sigma_id_.size(), fp);
    
    w_mu_ex_.resize(n_v, n_exp-1);
    fread(&w_mu_ex_.data()[0], sizeof(float), w_mu_ex_.size(), fp);
    sigma_ex_.resize(n_exp-1);
    fread(&sigma_ex_.data()[0], sizeof(float), sigma_ex_.size(), fp);
    
    int tmp;
    fread(&tmp, sizeof(int), 1, fp); uvs_.resize(tmp,2);
    fread(&uvs_.data()[0], sizeof(float), uvs_.size(), fp);
    
    fread(&tmp, sizeof(int), 1, fp); tri_pts_.resize(tmp,3); tri_uv_.resize(tmp,3);
    fread(&tri_pts_.data()[0], sizeof(int), tri_pts_.size(), fp);
    fread(&tri_uv_.data()[0], sizeof(int), tri_uv_.size(), fp);
    
    std::ofstream fout("tri.txt");
    fout << tri_pts_;
    fout.close();
    
    fclose(fp);
}

void BiLinearFaceModel::computeModelFW(const std::string& mesh_dir,
                                       const std::string& topo_mesh,
                                       float scale)
{
    // load all meshes
    std::vector<Eigen::MatrixXf> shapes(150);
    for(int i = 1; i <= 150; ++i)
    {
        char filename[256];
        snprintf(filename, 256, "%s/FaceWarehouse/Tester_%d/Blendshape/shape.bs", mesh_dir.c_str(), i);
        std::cout << filename << std::endl;
        loadBlendshapeFW(filename, shapes[i-1], 10.0);
    }
    
    computeCoreTensor(shapes, Cshape_, mu_id_, w_mu_ex_, sigma_id_, sigma_ex_, 80, -1);
    //sigma_ex_ = Eigen::VectorXf::Ones(n_exp());
    
    Eigen::VectorXf pts;
    Eigen::MatrixX3f nml;
    loadObjFile(topo_mesh, pts, nml, uvs_, tri_pts_, tri_uv_);    
}

FaceModelPtr BiLinearFaceModel::LoadModel(const std::string& file, const std::string& fm_type)
{
    auto model = new BiLinearFaceModel();
    
    model->fm_type_ = fm_type;
    model->loadBinaryModel(file);
    
    std::cout << "Face Model Info:" << std::endl;
    std::cout << "#Vert: " << model->mu_id_.size()/3 << " #Tri: " << model->tri_pts_.rows() << std::endl;

    return FaceModelPtr(model);
}
