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

#ifdef WITH_IMGUI
#include "imgui.h"
#endif

struct FaceModel;

struct FaceParams
{
    Eigen::VectorXf idCoeff;
    Eigen::VectorXf exCoeff;
    Eigen::VectorXf alCoeff;
    
    Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
    Eigen::Matrix3Xf SH = Eigen::Matrix3Xf::Zero(3,9);
    
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
    
    void updateIdentity(const FaceModel& model);
    void updateExpression(const FaceModel& model);
    void updateColor(const FaceModel& model);
    
    void updateAll(const FaceModel& model);
    void updateShape(const FaceModel& model);
    
    Eigen::Vector3f computeV(int i, const FaceModel& model) const;
    
    void init(const FaceModel& model);
    
#ifdef WITH_IMGUI
    bool updateIMGUI();
#endif
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
    
    Eigen::MatrixX2i sym_list_;
    
    std::vector<std::array<Eigen::Matrix3Xf, 2>> id_edge_;
    std::vector<std::array<Eigen::Matrix3Xf, 2>> ex_edge_;
    
    void saveBinaryModel(std::string file);
    void loadBinaryModel(std::string file);
    
    void loadOldBinaryModel(std::string modelfile, std::string meshfile);
};

#endif /* face_model_hpp */
