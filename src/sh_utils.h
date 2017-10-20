//
//  sh_utils.hpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 10/4/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef sh_utils_hpp
#define sh_utils_hpp

#define _USE_MATH_DEFINES
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <glm/glm.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tinyexr.h"

int CreateSphericalHarmonics(int M, int L, TinyExrImage &dest);

void RotateSHCoefficients(const Eigen::Matrix3Xf &src, Eigen::Matrix3Xf &tar,float x, float y, float z);

bool ReadSHCoefficients(std::string filepath, int order, Eigen::Matrix3Xf& SHCoeff);

void ReconstructSHfromSHImage(const int order, Eigen::Matrix3Xf& SHCoeff, const TinyExrImage* SHBasis, TinyExrImage& result);

void PanoramaSphericalHarmonicsBlurFromSHImage(const int order, const TinyExrImage* SH, TinyExrImage& source, TinyExrImage& result);

#endif /* sh_utils_hpp */
