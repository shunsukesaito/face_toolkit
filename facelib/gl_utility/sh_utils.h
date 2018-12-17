/*
 MIT License
 
 Copyright (c) 2018 Shunsuke Saito
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
#pragma once

#define _USE_MATH_DEFINES
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <glm/glm.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tinyexr.h>

int CreateSphericalHarmonics(int M, int L, TinyExrImage &dest);

void RotateSHCoefficients(const Eigen::Matrix3Xf &src, Eigen::Matrix3Xf &tar,float x, float y, float z);

bool ReadSHCoefficients(std::string filepath, int order, Eigen::Matrix3Xf& SHCoeff);

void ReconstructSHfromSHImage(const int order, Eigen::Matrix3Xf& SHCoeff, const TinyExrImage* SHBasis, TinyExrImage& result);

void PanoramaSphericalHarmonicsBlurFromSHImage(const int order, const TinyExrImage* SH, TinyExrImage& source, TinyExrImage& result);
