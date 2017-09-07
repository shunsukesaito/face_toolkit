//
//  face_model.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef face_model_hpp
#define face_model_hpp

#include <array>

#include "EigenHelper.h"

struct FaceParams
{
    Eigen::VectorXf idCoeff;
    Eigen::VectorXf exCoeff;
    Eigen::VectorXf alCoeff;
    
    Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Vector3f> SH = std::vector<Eigen::Vector3f>(9,Eigen::Vector3f::Zero());
};

struct FaceModel
{
    // identity basis
    Eigen::VectorXf mu_id_;
    Eigen::VectorXf sigma_id_;
    Eigen::MatrixXf w_id_;
    
    // expression basis
    Eigen::VectorXf mu_ex_;
    Eigen::VectorXf sigma_ex_;
    Eigen::MatrixXf w_ex_;
    
    // albedo basis
    Eigen::VectorXf mu_cl_;
    Eigen::VectorXf sigma_cl_;
    Eigen::MatrixXf w_cl_;
    
    // texture coordinates
    Eigen::MatrixX2f uvs_;
    
    // triangle list
    Eigen::MatrixX3i tri_pts_;
    Eigen::MatrixX3i tri_uv_;

    // current shape
    Eigen::VectorXf pts_;
    // current normal
    Eigen::MatrixX3f nml_;
    // current neutral
    Eigen::VectorXf neu_;
    // current color
    Eigen::VectorXf clr_;
    
    // expression delta
    Eigen::VectorXf d_ex_;
    // identity delta
    Eigen::VectorXf d_id_;
    
    Eigen::MatrixX2i sym_list_;
    
    void updateIdentity(const Eigen::VectorXf& val);
    void updateExpression(const Eigen::VectorXf& val);
    void updateColor(const Eigen::VectorXf& val);
    
    void Reset();
    
    void saveBinaryModel(std::string file);
    void loadBinaryModel(std::string file);
    
    void loadOldBinaryModel(std::string modelfile, std::string meshfile);
};

#endif /* face_model_hpp */
