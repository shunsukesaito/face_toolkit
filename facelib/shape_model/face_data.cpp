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
#include <fstream>
#include <iostream>
#include <vector>

#include <utility/obj_loader.h>

#include "face_model.h"

// constants
#include <gflags/gflags.h>
DEFINE_uint32(fw_cont_size, 15, "number of contour vertices on FW");

std::vector<int> sampleUniformContour(int n_sample, std::vector<int>& cont_indices, const Eigen::VectorXf& pts, const Eigen::Matrix4f& KRT)
{
    if(n_sample < 1) return std::vector<int>();
    
    std::vector<Eigen::Vector2f> p2d;
    Eigen::Vector4f v;
    for(int i = 0; i < cont_indices.size(); ++i)
    {
        int idx = cont_indices[i];
        v << pts.b3(idx), 1.0;
        v = KRT * v;
        if(fabs(v[2]) > 1e-8)
            v /= v[2];
        p2d.push_back(v.segment(0,2));
    }
    
    assert(n_sample%2 == 0); // the landmark points are sampled evenly on the right and left side separetely
    assert(p2d.size()%2 == 1);
    const int half_index = n_sample / 2;
    const int half_vertices_list = (p2d.size() - 1) / 2;
    float total_len = 0.0;
    
    // sample the first half of contour
    Eigen::Vector2f prev_p = p2d[0];
    std::vector<float> piece_lengthes;
    for(int i = 1; i < half_vertices_list+1; ++i)
    {
        float len = (p2d[i]-prev_p).norm();
        piece_lengthes.push_back(len);
        total_len += len;
        prev_p = p2d[i];
    }
    
    float step = total_len / (float)half_index;
    std::vector<int> sampled_contour_indices;
    sampled_contour_indices.push_back(cont_indices[0]);
    float cur_len = piece_lengthes[0];
    for(int i = 1; i <= piece_lengthes.size(); ++i)
    {
        if(cur_len > step){
            float ratio = (cur_len-step)/piece_lengthes[i-1];
            if(ratio > 0.5)
                sampled_contour_indices.push_back(cont_indices[i-1]);
            else
                sampled_contour_indices.push_back(cont_indices[i]);
            cur_len = cur_len - step;
            i--;
            continue;
        }
        cur_len += piece_lengthes[i];
    }
    if(sampled_contour_indices.size() != half_index + 1)
        sampled_contour_indices.push_back(cont_indices[half_vertices_list]);
    
    // sampled points on the second half of the contour
    total_len = 0.0;
    
    prev_p = p2d[half_vertices_list];
    piece_lengthes.clear();
    for(int i = half_vertices_list+1; i < p2d.size(); ++i)
    {
        float len = (p2d[i]-prev_p).norm();
        piece_lengthes.push_back(len);
        total_len += len;
        prev_p = p2d[i];
    }
    
    step = total_len / (float)half_index;
    
    cur_len = piece_lengthes[0];
    for(int i = 1; i <= piece_lengthes.size(); ++i)
    {
        if(cur_len > step){
            float ratio = (cur_len-step)/piece_lengthes[i-1];
            if(ratio > 0.5)
                sampled_contour_indices.push_back(cont_indices[half_vertices_list+i-1]);
            else
                sampled_contour_indices.push_back(cont_indices[half_vertices_list+i]);

            if(sampled_contour_indices.size() == n_sample) break;
            cur_len = cur_len - step;
            i--;
            continue;
        }
        cur_len += piece_lengthes[i];
    }
    if(sampled_contour_indices.size() != n_sample+1)
        sampled_contour_indices.push_back(cont_indices[p2d.size()-1]);

    assert(sampled_contour_indices.size() == n_sample+1);

    return sampled_contour_indices;
}

void FaceData::updateParams(const FaceParams& fp)
{
    this->idCoeff = fp.idCoeff;
    this->exCoeff = fp.exCoeff;
    this->alCoeff = fp.alCoeff;

    this->RT_ = fp.RT;
    this->SH_ = fp.SH;
}

void FaceData::updateIdentity()
{
    assert(model_ != NULL);
    model_->updateIdentity(*this);
    idCoeff_prev = idCoeff;
}

void FaceData::updateExpression()
{
    assert(model_ != NULL);
    model_->updateExpression(*this);
    exCoeff_prev = exCoeff;
}

void FaceData::updateColor()
{
    assert(model_ != NULL);
    model_->updateColor(*this);
    alCoeff_prev = alCoeff;
}

void FaceData::updateAll()
{
    assert(model_ != NULL);
    if(idCoeff.size() != idCoeff_prev.size() || !idCoeff.isApprox(idCoeff_prev)){
        updateIdentity();
        updateExpression();
    }
    else if(exCoeff.size() != exCoeff_prev.size() || !exCoeff.isApprox(exCoeff_prev))
        updateExpression();
    if(alCoeff.size() != alCoeff_prev.size() || !alCoeff.isApprox(alCoeff_prev))
        updateColor();
}

void FaceData::updateShape()
{
    assert(model_ != NULL);
    if(idCoeff.size() != idCoeff_prev.size() || !idCoeff.isApprox(idCoeff_prev)){
        updateIdentity();
        updateExpression();
    }
    else if(exCoeff.size() != exCoeff_prev.size() || !exCoeff.isApprox(exCoeff_prev))
        updateExpression();
}

Eigen::Vector3f FaceData::computeV(int vidx) const
{
    assert(model_ != NULL);
    return model_->computeV(vidx, *this);
}

Eigen::Ref<const Eigen::MatrixXf> FaceData::dID(int vidx, int size) const
{
    assert(model_ != NULL);
    return model_->dID(vidx, size, *this);
}

Eigen::Ref<const Eigen::MatrixXf> FaceData::dEX(int vidx, int size) const
{
    assert(model_ != NULL);
    return model_->dEX(vidx, size, *this);
}

Eigen::Ref<const Eigen::MatrixXf> FaceData::dCL(int vidx, int size) const
{
    assert(model_ != NULL);
    return model_->dCL(vidx, size, *this);
}

const Eigen::Matrix3Xf& FaceData::dIDEdge(int fidx, int eidx) const
{
    assert(model_ != NULL);
    return model_->dIDEdge(fidx, eidx, *this);
}

const Eigen::Matrix3Xf& FaceData::dEXEdge(int fidx, int eidx) const
{
    assert(model_ != NULL);
    return model_->dEXEdge(fidx, eidx, *this);
}

void FaceData::dSym(int symidx, int axis, int nid, int nex, Eigen::Vector3f& v, Eigen::MatrixXf& dv) const
{
    assert(model_ != NULL);
    model_->dSym(symidx, axis, nid, nex, v, dv, *this);
}

void FaceData::setFaceModel(FaceModelPtr model)
{
    model_ = model;
    
    maps_ = model_->maps_;
    uvs_ = model_->uvs_;
    tri_pts_ = model_->tri_pts_;
    tri_uv_ = model_->tri_uv_;
    
    init();
}

void FaceData::init()
{
    assert(model_ != NULL);
    idCoeff = Eigen::VectorXf::Zero(model_->n_id());
    exCoeff = Eigen::VectorXf::Zero(model_->n_exp());
    alCoeff = Eigen::VectorXf::Zero(model_->n_clr());

    SH_ = Eigen::Matrix3Xf::Zero(3,9);
    SH_.col(0).setOnes();
    RT_.setIdentity();
    
    updateAll();
}

void FaceData::updateContour(const Eigen::Matrix4f& K, const Eigen::Matrix4f& RTc)
{
    if(model_->fm_type_.find("bv") != std::string::npos)
        updateContourBV(K, RTc);
    else if(model_->fm_type_.find("fw") != std::string::npos)
        updateContourFW(K, RTc);
    else if(model_->fm_type_.find("pin") != std::string::npos)
        updateContourPIN(K, RTc);
}

void FaceData::updateContourBV(const Eigen::Matrix4f& K, const Eigen::Matrix4f& RTc)
{
    const auto& cont_lists = model_->cont_candi_;
    
	Eigen::Matrix4f KRT = K*RTc*RT();
    
    Eigen::Vector4f v;
    cont_idx_.clear();
    for(int i = 0; i < cont_lists.size(); ++i)
    {
		int idx = cont_lists[i][0];
        v << pts_.b3(idx), 1.0;
		v = KRT*v;

        float px = v[0]/v[2];
        for(int j = 1; j < cont_lists[i].size(); ++j)
        {
			v << pts_.b3(cont_lists[i][j]), 1.0;
			v = KRT*v;
            v /= v[2];
            if(i < 8){
                if(v[0] < px){
					px = v[0];
                    idx = cont_lists[i][j];
                }
            }
            else{
				if (v[0] > px){
					px = v[0];
                    idx = cont_lists[i][j];
                }
            }
        }
        cont_idx_.push_back(idx);
    }
}

void FaceData::updateContourPIN(const Eigen::Matrix4f& K, const Eigen::Matrix4f& RTc)
{
    const auto& cont_lists = model_->cont_candi_;
    
    Eigen::Matrix4f extRT = RTc*RT();
    Eigen::Matrix4f KRT = K*RTc*RT();
    
    cont_idx_.clear();
    Eigen::Vector4f n, v;
    for(int i = 0; i < cont_lists.size(); ++i)
    {
        int min_id = -1;
        float prev_dot = 0.0;
        float dot = 0.0;
        
        std::vector<int> cont_cand;
        for(int j = 0; j < cont_lists[i].size(); ++j)
        {
            int idx = cont_lists[i][j];
            n << nml_.row(idx).transpose(), 0.0;
            v << pts_.b3(idx), 1.0;
            
            n = extRT * n;
            v = extRT * v;
            
            dot = v.segment(0,3).normalized().dot(n.segment(0,3));
            if(j != 0 && prev_dot*dot < 0){
                if(fabs(dot) < fabs(prev_dot)){
                    cont_cand.push_back(cont_lists[i][j]);
                }
                else{
                    cont_cand.push_back(cont_lists[i][j-1]);
                }
            }
            prev_dot = dot;
        }
        if(cont_cand.size() == 0){
            if(prev_dot < 0){
                min_id = cont_lists[i][0];
            }
            else{
                min_id = cont_lists[i][cont_lists[i].size()-1];
            }
        }
        else {
            min_id = cont_cand[0]; // Note: so far, it works.
        }
        
        assert(min_id != -1);
        cont_idx_.push_back(min_id);
    }
}

void FaceData::updateContourFW(const Eigen::Matrix4f& K, const Eigen::Matrix4f& RTc)
{
    const auto& cont_lists = model_->cont_candi_;
    
    Eigen::Matrix4f extRT = RTc*RT();
	Eigen::Matrix4f KRT = K*RTc*RT();
    
    std::vector<int> contour_indices;
    Eigen::Vector4f n, v;
    for(int i = 0; i < cont_lists.size(); ++i)
    {
        int min_id = -1;
        float prev_dot = 0.0;
        float dot = 0.0;

        std::vector<int> cont_cand;
        for(int j = 0; j < cont_lists[i].size(); ++j)
        {
            int idx = cont_lists[i][j];
            n << nml_.row(idx).transpose(), 0.0;
            v << pts_.b3(idx), 1.0;

            n = extRT * n;
            v = extRT * v;
          
            dot = v.segment(0,3).normalized().dot(n.segment(0,3));
            if(j != 0 && prev_dot*dot < 0){
                if(fabs(dot) < fabs(prev_dot)){
                    cont_cand.push_back(cont_lists[i][j]);
                }
                else{
                    cont_cand.push_back(cont_lists[i][j-1]);
                }
            }
            prev_dot = dot;
        }
        if(cont_cand.size() == 0){
            if(prev_dot < 0){
                min_id = cont_lists[i][0];
            }
            else{
                min_id = cont_lists[i][cont_lists[i].size()-1];
            }
        }
        else {
            min_id = cont_cand[0]; // Note: so far, it works.
        }

        assert(min_id != -1);
        contour_indices.push_back(min_id);
    }
    
    cont_idx_ = sampleUniformContour(FLAGS_fw_cont_size-1, contour_indices, pts_, KRT);
}

#ifdef WITH_IMGUI
void FaceData::updateIMGUI()
{
    Eigen::Vector3f euler = Eigen::matToEulerAngle(RT_.block<3,3>(0,0));
    if (ImGui::CollapsingHeader("Face Parameters")){
        if (ImGui::Button("Reset")){
            idCoeff.setZero();
            exCoeff.setZero();
            alCoeff.setZero();
            RT_ = Eigen::Matrix4f::Identity();
            SH_.setZero();
            SH_.col(0).setOnes();
        }
        if(idCoeff.size() != 0)
        if (ImGui::TreeNode("ID")){
            for(int i = 0; i < idCoeff.size(); ++i)
                ImGui::SliderFloat(("id" + std::to_string(i)).c_str(), &idCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
        }
        if(exCoeff.size() != 0)
        if (ImGui::TreeNode("EX")){
            for(int i = 0; i < exCoeff.size(); ++i)
                ImGui::SliderFloat(("ex" + std::to_string(i)).c_str(), &exCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
        }
        if(alCoeff.size() != 0)
        if (ImGui::TreeNode("CL")){
            for(int i = 0; i < alCoeff.size(); ++i)
                ImGui::SliderFloat(("al" + std::to_string(i)).c_str(), &alCoeff[i], -2.0, 2.0);
            ImGui::TreePop();
        }
//        ImGui::InputFloat("Scale", &scale);
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

void BaseFaceModel::loadContourList(std::string file)
{
    std::ifstream fin;
    fin.open(file,std::ifstream::in);
    
    cont_candi_.clear();
    if(fin.is_open()){
        std::string line;
        
        int index;
        while(getline(fin, line))
        {
            std::vector<int> points;
            size_t current = 0, found;
            while((found = line.find_first_of(" ", current)) != std::string::npos){
                index = atoi(std::string(line,current,found - current).c_str());
                points.push_back(index);
                current = found+1;
            }
            index = atoi(std::string(line,current,line.size()-current).c_str());
            points.push_back(index);
            cont_candi_.push_back(points);
        }
        fin.close();
    }
    else{
        std::cout << "Error Contour List cannot open... " << file << std::endl;
    }
}

void BaseFaceModel::loadMeanFromObj(const std::string &filename)
{
    Eigen::MatrixX3f nml;
    Eigen::MatrixX3i trinml;
    loadObjFile(filename, mu_id_, nml, uvs_, tri_pts_, trinml, tri_uv_);
}


