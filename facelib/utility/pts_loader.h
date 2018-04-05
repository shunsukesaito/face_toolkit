//
//  pts_loader.h
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 8/25/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Core>

std::vector<Eigen::Vector3f> load_pts(std::string file);
bool write_pts(std::string file, const std::vector<Eigen::Vector3f>& pts);
