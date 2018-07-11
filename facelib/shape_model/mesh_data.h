//
//  mesh_data.h
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <Eigen/Dense>

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

struct MeshData{
    Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
    Eigen::Matrix3Xf SH = Eigen::Matrix3Xf::Zero(3,9);
    
    Eigen::VectorXf pts_;  // current shape
    Eigen::MatrixX3f nml_; // current normal
    Eigen::VectorXf clr_;  // current color
    
    Eigen::MatrixX2f uvs_;
    
    // triangle list
    Eigen::MatrixX3i tri_pts_;
    Eigen::MatrixX3i tri_uv_;
    
    // for texture maps
    std::vector<unsigned int> maps_;
    
    void saveObj(const std::string& filename, bool no_uv = false);
    void loadObj(const std::string& filename);

#ifdef WITH_IMGUI
    void updateIMGUI();
#endif

};
