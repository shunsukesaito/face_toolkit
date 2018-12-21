//
//  hogFeature.hpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 10/29/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <utility>
#include <cassert>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

extern "C" {
#include "hog.h" // From the VLFeat C library
}

void GetHogFeature(Eigen::MatrixXf& feature,
                   const std::vector<Eigen::Vector2f>& pixels,
                   const cv::Mat_<uchar>& image,
                   int cell_size = 16,
                   VlHogVariant vlhog_variant = VlHogVariant::VlHogVariantUoctti,
                   int num_cells = 3,
                   int num_bins = 4);

