//
//  mesh_data.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include <fstream>
#include <iostream>
#include <vector>

#include <utility/obj_loader.h>

#include "mesh_data.h"

// constants
#include <gflags/gflags.h>

void MeshData::saveObj(const std::string& filename, bool no_uv)
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
        Eigen::MatrixX2f& uvs = uvs_;
        Eigen::MatrixX3i& tri_pts = tri_pts_;
        Eigen::MatrixX3i& tri_uv = tri_uv_;
        
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
                fout << "f " << tri_pts(i,0)+1 << "/" << tri_uv(i,0)+1 << "/" << tri_pts(i,0)+1;
                fout << " " << tri_pts(i,1)+1 << "/" << tri_uv(i,1)+1 << "/" << tri_pts(i,1)+1;
                fout << " " << tri_pts(i,2)+1 << "/" << tri_uv(i,2)+1 << "/" << tri_pts(i,2)+1 << std::endl;
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
    loadObjFile(filename, pts_, nml_, uvs_, tri_pts_, tri_uv_);
}

#ifdef WITH_IMGUI
void MeshData::updateIMGUI()
{
    Eigen::Vector3f euler = Eigen::matToEulerAngle(RT.block<3,3>(0,0));
    if (ImGui::CollapsingHeader("Mesh Parameters")){
        if (ImGui::Button("Reset")){
            RT = Eigen::Matrix4f::Identity();
            SH.setZero();
            SH.col(0).setOnes();
        }
        ImGui::InputFloat3("Rot", &euler(0), -1, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputFloat3("Tr", &RT(0,3));
        if (ImGui::TreeNode("SH")){
            for(int i = 0; i < SH.cols(); ++i)
                ImGui::InputFloat3(("sh" + std::to_string(i)).c_str(), &SH.col(i)[0]);
            ImGui::TreePop();
        }
    }
}
#endif
