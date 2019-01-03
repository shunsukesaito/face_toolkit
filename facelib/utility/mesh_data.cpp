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
#include "mesh_data.h"

#include <fstream>
#include <iostream>
#include <vector>

#include "obj_loader.h"

void MeshData::saveObj(const std::string& filename, bool no_uv) const
{
    std::ofstream fout(filename);
    if(fout.is_open()){
        for(int i = 0; i < pts_.size()/3; ++i)
        {
            fout << "v " << pts_(i*3+0) << " " << pts_(i*3+1) << " " << pts_(i*3+2);
            if(clr_.size() == pts_.size())
                fout << " " << clr_(i*3+0) << " " << clr_(i*3+1) << " " << clr_(i*3+2);
            fout << std::endl;
            if(nml_.size() == pts_.size())
                fout << "vn " << nml_(i,0) << " " << nml_(i,1) << " " << nml_(i,2) << std::endl;
        }
        const Eigen::MatrixX2f& uvs = this->uvs();
        const Eigen::MatrixX3i& tri_pts = this->tripts();
        const Eigen::MatrixX3i& tri_nml = this->trinml();
        const Eigen::MatrixX3i& tri_uv = this->triuv();
        
        if(no_uv){
            for(int i = 0; i < tri_pts.rows(); ++i)
            {
                fout << "f " << tri_pts(i,0)+1;
                fout << " " << tri_pts(i,1)+1;
                fout << " " << tri_pts(i,2)+1 << std::endl;
            }
        }
        else{
            for(int i = 0; i < uvs.rows(); ++i)
            {
                fout << "vt " << uvs(i,0) << " " << uvs(i,1) << std::endl;
            }
            for(int i = 0; i < tri_pts.rows(); ++i)
            {
                fout << "f " << tri_pts(i,0)+1 << "/" << tri_uv(i,0)+1 << "/" << tri_nml(i,0)+1;
                fout << " " << tri_pts(i,1)+1 << "/" << tri_uv(i,1)+1 << "/" << tri_nml(i,1)+1;
                fout << " " << tri_pts(i,2)+1 << "/" << tri_uv(i,2)+1 << "/" << tri_nml(i,2)+1 << std::endl;
            }
        }
        fout.close();
    }
    else{
        std::cout << "Error: cannot open the file :" << filename << std::endl;
    }
}

void MeshData::loadObj(const std::string &filename)
{
    loadObjFile(filename, pts_, nml_, uvs_, tri_pts_, tri_nml_, tri_uv_);
    
    // TODO: make it easier to switch between per-tri and per vert attribute managements
    // for now, orignal normal values are ignored
    tri_nml_ = tri_pts_;
    calcNormal(nml_, pts_, tri_nml_);
}

#ifdef WITH_IMGUI
void MeshData::updateIMGUI()
{
    Eigen::Vector3f euler = Eigen::matToEulerAngle(RT_.block<3,3>(0,0));
    if (ImGui::CollapsingHeader("Mesh Parameters")){
        if (ImGui::Button("Reset")){
            RT_ = Eigen::Matrix4f::Identity();
            SH_.setZero();
            SH_.col(0).setOnes();
        }
        ImGui::InputFloat3("Rot", &euler(0), -1, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputFloat3("Tr", &RT_(0,3));
        if (ImGui::TreeNode("SH")){
            for(int i = 0; i < SH_.cols(); ++i)
                ImGui::InputFloat3(("sh" + std::to_string(i)).c_str(), &SH_.col(i)[0]);
            ImGui::TreePop();
        }
    }
}
#endif
