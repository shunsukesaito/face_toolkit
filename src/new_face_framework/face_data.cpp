//
//  face_model.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include <fstream>
#include <iostream>
#include <vector>

#include "face_model.hpp"
#include "obj_loader.hpp"

void FaceData::updateIdentity()
{
    model_->updateIdentity(*this);
}

void FaceData::updateExpression()
{
    model_->updateExpression(*this);
}

void FaceData::updateColor()
{
    model_->updateColor(*this);
}

void FaceData::updateAll()
{
    model_->updateColor(*this);
    model_->updateIdentity(*this);
    model_->updateExpression(*this);
}

void FaceData::updateShape()
{
    model_->updateIdentity(*this);
    model_->updateExpression(*this);
}

Eigen::Vector3f FaceData::computeV(int i) const
{
    return model_->computeV(i, *this);
}

Eigen::Ref<const Eigen::MatrixXf> FaceData::dID(int vidx, int size) const
{
    return model_->dID(vidx, size, *this);
}

Eigen::Ref<const Eigen::MatrixXf> FaceData::dEX(int vidx, int size) const
{
    return model_->dEX(vidx, size, *this);
}

Eigen::Ref<const Eigen::MatrixXf> FaceData::dCL(int vidx, int size) const
{
    return model_->dCL(vidx, size, *this);
}

const Eigen::Matrix3Xf& FaceData::dIDEdge(int fidx, int eidx) const
{
    return model_->dIDEdge(fidx, eidx, *this);
}

const Eigen::Matrix3Xf& FaceData::dEXEdge(int fidx, int eidx) const
{
    return model_->dEXEdge(fidx, eidx, *this);
}

void FaceData::dSym(int symidx, int axis, int nid, int nex, Eigen::Vector3f& v, Eigen::MatrixXf& dv) const
{
    model_->dSym(symidx, axis, nid, nex, v, dv, *this);
}

void FaceData::init()
{
    idCoeff = Eigen::VectorXf::Zero(model_->n_id());
    exCoeff = Eigen::VectorXf::Zero(model_->n_exp());
    alCoeff = Eigen::VectorXf::Zero(model_->n_clr());

    SH = Eigen::Matrix3Xf::Zero(3,9);
    SH.col(0).setOnes();
    RT.setIdentity();
    
    updateAll();
}

void FaceData::saveObj(const std::string& filename)
{
    std::ofstream fout(filename);
    if(fout.is_open()){
        for(int i = 0; i < pts_.size()/3; ++i)
        {
            fout << "v " << pts_(i*3+0) << " " << pts_(i*3+1) << " " << pts_(i*3+2);
            fout << " " << clr_(i*3+0) << " " << clr_(i*3+1) << " " << clr_(i*3+2) << std::endl;
            fout << "vn " << nml_(i,0) << " " << nml_(i,1) << " " << nml_(i,2) << std::endl;
        }
        Eigen::MatrixX2f& uvs = model_->uvs_;
        Eigen::MatrixX3i& tri_pts = model_->tri_pts_;
        Eigen::MatrixX3i& tri_uv = model_->tri_uv_;
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
        fout.close();
    }
    else{
        std::cout << "Error: cannot open the file :" << filename << std::endl;
    }
}

#ifdef WITH_IMGUI
void FaceData::updateIMGUI()
{
    Eigen::Vector3f euler = Eigen::matToEulerAngle(RT.block<3,3>(0,0));
    if (ImGui::CollapsingHeader("Face Parameters")){
        if (ImGui::TreeNode("ID")){
            for(int i = 0; i < idCoeff.size(); ++i)
                ImGui::SliderFloat(("id" + std::to_string(i)).c_str(), &idCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("EX")){
            for(int i = 0; i < exCoeff.size(); ++i)
                ImGui::SliderFloat(("ex" + std::to_string(i)).c_str(), &exCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("CL")){
            for(int i = 0; i < alCoeff.size(); ++i)
                ImGui::SliderFloat(("al" + std::to_string(i)).c_str(), &alCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
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

