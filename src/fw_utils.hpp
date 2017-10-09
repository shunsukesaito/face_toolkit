//
//  fw_utils.hpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 10/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef fw_utils_hpp
#define fw_utils_hpp
#include <fstream>
#include <iostream>
#include <vector>

#include <Eigen/Core>

void loadBlendshapeFW( const std::string& filename,
                      Eigen::MatrixXf& shape,
                      float scale);

void writeCoreTensor(const std::string& out_model,
                     const std::string& mesh_dir,
                     const std::string& topo_mesh,
                     float scale = 1.0);

void computeCoreTensor(const std::vector<Eigen::MatrixXf>& in,
                       std::vector<Eigen::MatrixXf>& out,
                       Eigen::VectorXf& mean,
                       Eigen::VectorXf& stddev,
                       int tar_id);

#endif /* fw_utils_hpp */
