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

#include <fstream>
#include <gl_utility/camera.h>
#include <utility/str_utils.h>
#include <utility/capture_data.h>

#include <shape_model/face_model.h>
#include "constraints.h"

struct FaceResult
{
    bool processed_ = false;
    
    // assume single camera for now
    MFMVCaptureData cap_data = MFMVCaptureData(1,1);
    std::vector<Camera> cameras = std::vector<Camera>(1);
    std::vector<FaceData> fd = std::vector<FaceData>(1);
    
    std::vector<P2P2DC> c_p2p;
    std::vector<P2L2DC> c_p2l;
    
    // assuming single frame and single camera
    inline void loadFromTXT(std::string filename){
        std::ifstream infile(filename);
        if(!infile.is_open()){
            std::cout << "Warning: failed parsing face data from " << filename << std::endl;
            return;
        }
        std::string line;
        
        std::vector<float> tmp;
        // identity
        std::getline(infile, line);
        tmp = string2arrayf(line);
        fd[0].idCoeff.segment(0,tmp.size()) = Eigen::Map<Eigen::VectorXf>(&tmp[0],tmp.size());
        // expression
        std::getline(infile, line);
        tmp = string2arrayf(line);
        fd[0].exCoeff.segment(0,tmp.size()) = Eigen::Map<Eigen::VectorXf>(&tmp[0],tmp.size());
        // albedo
        std::getline(infile, line);
        tmp = string2arrayf(line);
        fd[0].alCoeff.segment(0,tmp.size()) = Eigen::Map<Eigen::VectorXf>(&tmp[0],tmp.size());
        // face rotation
        std::getline(infile, line);
        tmp = string2arrayf(line);
        fd[0].RT.block(0,0,3,3) = Eigen::Map<Eigen::Matrix3f>(&tmp[0]);
        // face translation
        std::getline(infile, line);
        tmp = string2arrayf(line);
        fd[0].RT.block(0,3,3,1) = Eigen::Map<Eigen::Vector3f>(&tmp[0]);
        // spherical hamonics
        std::getline(infile, line);
        tmp = string2arrayf(line);
        fd[0].SH = Eigen::Map<Eigen::MatrixXf>(&tmp[0],3,9);
        // camera rotation
        std::getline(infile, line);
        tmp = string2arrayf(line);
        cameras[0].extrinsic_.block(0,0,3,3) = Eigen::Map<Eigen::Matrix3f>(&tmp[0]);
        // camera translation
        std::getline(infile, line);
        tmp = string2arrayf(line);
        cameras[0].extrinsic_.block(0,3,3,1) = Eigen::Map<Eigen::Vector3f>(&tmp[0]);
        // camera intrinsic
        std::getline(infile, line);
        tmp = string2arrayf(line);
        cameras[0].intrinsic_.block(0,0,3,3) = Eigen::Map<Eigen::Matrix3f>(&tmp[0]);
    }
    
    inline void loadFromVec(const std::vector<std::pair<std::string, int>>& dof, const std::vector<float>& val){
        std::vector<float> val_ = val;
        float* p = &val_[0];
        int total_dof = 0;
        for(auto itr = dof.begin(); itr != dof.end(); ++itr)
        {
            std::string label = itr->first;
            int dim = itr->second;
            if(dim <= 0)
                continue;
            
            if(label.find("id") != std::string::npos){
                fd[0].idCoeff.segment(0,dim) = Eigen::Map<Eigen::VectorXf>(p,dim);
                p += dim;
            }
            if(label.find("ex") != std::string::npos){
                fd[0].exCoeff.segment(0,dim) = Eigen::Map<Eigen::VectorXf>(p,dim);
                p += dim;
            }
            if(label.find("al") != std::string::npos){
                fd[0].alCoeff.segment(0,dim) = Eigen::Map<Eigen::VectorXf>(p,dim);
                p += dim;
            }
            if(label.find("rf") != std::string::npos){
                assert(dim == 9);
                fd[0].RT.block(0,0,3,3) = Eigen::Map<Eigen::Matrix3f>(p);
                p += dim;
            }
            if(label.find("tf") != std::string::npos){
                assert(dim == 3);
                fd[0].RT.block(0,3,3,1) = Eigen::Map<Eigen::Vector3f>(p);
                p += dim;
            }
            if(label.find("sh") != std::string::npos){
                assert(dim == 27);
                fd[0].SH = Eigen::Map<Eigen::MatrixXf>(p,3,9);
                p += dim;
            }
            if(label.find("rc") != std::string::npos){
                assert(dim == 9);
                cameras[0].extrinsic_.block(0,0,3,3) = Eigen::Map<Eigen::Matrix3f>(p);
                p += dim;
            }
            if(label.find("tc") != std::string::npos){
                assert(dim == 3);
                cameras[0].extrinsic_.block(0,3,3,1) = Eigen::Map<Eigen::Vector3f>(p);
                p += dim;
            }
            if(label.find("k") != std::string::npos){
                assert(dim == 9);
                cameras[0].intrinsic_.block(0,0,3,3) = Eigen::Map<Eigen::Matrix3f>(p);
                p += dim;
            }
            total_dof += dim;
            if(total_dof > val.size())
                break;
        }
        if(total_dof != val.size()){
            std::cout << "Warning: dof and value dimension doesn't match." << std::endl;
        }
    }
    
    inline void saveToTXT(std::string filename){
        std::ofstream fout(filename);
        if(!fout.is_open()){
            std::cout << "Warning: failed writing result to " << filename << std::endl;
            return;
        }
        
        fout << fd[0].idCoeff.transpose() << std::endl;
        fout << fd[0].exCoeff.transpose() << std::endl;
        fout << fd[0].alCoeff.transpose() << std::endl;
        Eigen::Matrix3f Rf = fd[0].RT.block(0,0,3,3);
        Eigen::Map<Eigen::RowVectorXf> Rfmap(Rf.data(), Rf.size());
        fout << Rfmap << std::endl;
        fout << fd[0].RT.block(0,3,3,1).transpose() << std::endl;
        Eigen::Map<Eigen::RowVectorXf> SHmap(fd[0].SH.data(), fd[0].SH.size());
        fout << SHmap << std::endl;
        Eigen::Matrix3f Rc = cameras[0].extrinsic_.block(0,0,3,3);
        Eigen::Map<Eigen::RowVectorXf> Rcmap(Rc.data(), Rc.size());
        fout << Rcmap << std::endl;
        fout << cameras[0].extrinsic_.block(0,3,3,1).transpose() << std::endl;
        Eigen::Matrix3f K = cameras[0].intrinsic_.block(0,0,3,3);
        Eigen::Map<Eigen::RowVectorXf> Kmap(K.data(), K.size());
        fout << Kmap << std::endl;
    }
};

inline std::vector<Eigen::Vector3f> getP3DFromP2PC(const Eigen::VectorXf& pts, const std::vector<P2P2DC>& c_p2p)
{
    std::vector<Eigen::Vector3f> out;
    
    for(auto&& c : c_p2p)
    {
        out.push_back(pts.b3(c.v_idx));
    }
    
    return out;
}

inline std::vector<Eigen::Vector3f> getP3DFromP2LC(const Eigen::VectorXf& pts, const std::vector<P2L2DC>& c_p2l)
{
    std::vector<Eigen::Vector3f> out;
    
    for(auto&& c : c_p2l)
    {
        out.push_back(pts.b3(c.v_idx));
    }
    
    return out;
}
