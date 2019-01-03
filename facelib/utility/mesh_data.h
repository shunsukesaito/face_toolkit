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

#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

struct MeshData{
    Eigen::Matrix4f RT_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix3Xf SH_ = Eigen::Matrix3Xf::Zero(3,9);
    
    Eigen::VectorXf pts_;  // current shape
    Eigen::MatrixX3f nml_; // current normal
    Eigen::VectorXf clr_;  // current color
    
    Eigen::MatrixX2f uvs_;
    
    // triangle list
    Eigen::MatrixX3i tri_pts_;
    Eigen::MatrixX3i tri_nml_;
    Eigen::MatrixX3i tri_uv_;
    
    // for texture maps
    std::unordered_map<std::string, unsigned> maps_;
    //std::vector<unsigned> maps_;
    
    Eigen::Matrix4f& RT() { return RT_; }
    Eigen::Matrix3Xf& SH() { return SH_; }
    
    Eigen::VectorXf& pts() { return pts_; }
    Eigen::MatrixX3f& nml() { return nml_; }
    Eigen::VectorXf& clr() { return clr_; }
    
    std::unordered_map<std::string, unsigned>& maps() { return maps_; }
    Eigen::MatrixX2f& uvs() { return uvs_; }
    Eigen::MatrixX3i& tripts() { return tri_pts_; }
    Eigen::MatrixX3i& trinml() { return tri_nml_; }
    Eigen::MatrixX3i& triuv() { return tri_uv_; }

    const Eigen::Matrix4f& RT() const { return RT_; }
    const Eigen::Matrix3Xf& SH() const { return SH_; }
    
    const Eigen::VectorXf& pts() const { return pts_; }
    const Eigen::MatrixX3f& nml() const { return nml_; }
    const Eigen::VectorXf& clr() const { return clr_; }

    const std::unordered_map<std::string, unsigned>& maps() const { return maps_; }
    const Eigen::MatrixX2f& uvs() const { return uvs_; }
    const Eigen::MatrixX3i& tripts() const { return tri_pts_; }
    const Eigen::MatrixX3i& trinml() const { return tri_nml_; }
    const Eigen::MatrixX3i& triuv() const { return tri_uv_; }
    
    void saveObj(const std::string& filename, bool no_uv = false) const;

    void loadObj(const std::string& filename);
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif

};
