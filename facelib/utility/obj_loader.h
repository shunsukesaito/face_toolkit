//
//  obj_loader.h
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 8/25/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <iostream>
#include <fstream>

#include <tiny_obj_loader.h>

#include "EigenHelper.h"

void writeObj(const std::string& filename,
              Eigen::VectorXf& pts,
              Eigen::MatrixX3f& nml,
              Eigen::MatrixX2f& uvs,
              Eigen::MatrixX3i& tri_pts,
              Eigen::MatrixX3i& tri_uv);

void loadObjFile(const std::string& filename,
                 Eigen::VectorXf& pts,
                 Eigen::MatrixX3f& nml,
                 Eigen::MatrixX2f& uvs,
                 Eigen::MatrixX3i& tri_pts,
                 Eigen::MatrixX3i& tri_uv);
