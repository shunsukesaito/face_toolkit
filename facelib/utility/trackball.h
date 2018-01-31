//
//  trackball.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 8/26/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

void trackball(const double w,
               const double h,
               const double speed_factor,
               const double down_mouse_x,
               const double down_mouse_y,
               const double mouse_x,
               const double mouse_y,
               Eigen::Quaternion<float>& quat);
