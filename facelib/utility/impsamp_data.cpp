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

#include "imsamp_data.h"
#include "timer.h"

void ImSampData::computeThickness(int nSample)
{
    // create sampler
    Sampler sampler((unsigned)sqrt(nSample));
    
    // build bvh tree
    BVHTree bvh;
    bvh.build(dynamic_cast<MeshData*>(this));
    
    int nVert = pts_.size()/3;
    int nTri = tri_pts_.size()/3;
    
    cxxtimer::Timer timer;
    timer.start();
    
    size_t steps_completed = 0;
    
    // compute center of triangle + face normal
    std::vector<Eigen::Vector3f> ptsF(nTri);
    std::vector<Eigen::Vector3f> nmlF(nTri);
    std::vector<float> areaF(nTri);
#pragma omp parallel for
    for(int i = 0; i < nTri; ++i)
    {
        const Eigen::Vector3f& p0 = pts_.b3(tri_pts_(i,0));
        const Eigen::Vector3f& p1 = pts_.b3(tri_pts_(i,1));
        const Eigen::Vector3f& p2 = pts_.b3(tri_pts_(i,2));
        Eigen::Vector3f p = (p0 + p1 + p2)/3.0;
        ptsF[i] = p;
        
        nmlF[i] = (p1-p0).cross(p2-p0);
        areaF[i] = 0.5*nmlF[i].norm();
        nmlF[i].normalize();
    }
    
    Eigen::VectorXf thicknessF(nTri);
    thicknessF.setConstant(1.e10f);
#pragma omp parallel for
    for(int i = 0; i < nTri; ++i)
    {
        const Eigen::Vector3f& p = ptsF[i];
        const Eigen::Vector3f& n = nmlF[i];
        
        float cur_thick = thicknessF[i];
        for(int j = 0; j < nSample; ++j)
        {
            const Sample& smpl = sampler.samples_[j];
            float dot = n.dot(smpl.dir_);
            
            if(fabs(dot) <= 0.4f)
                continue;
            
            Ray ray(p, smpl.dir_, dot > 0);
            
            if(bvh.interTest(ray))
            {
                if(cur_thick > ray.t_)
                    cur_thick = ray.t_;
            }
        }
        thicknessF[i] = cur_thick;
        
#pragma omp atomic
        ++steps_completed;
        
        if(steps_completed % 1000 == 0){
#pragma omp critical
            std::cout << steps_completed << "/" << nTri << " completed..." << std::endl;
        }
    }
    
    thickness_.resize(nVert);
    thickness_.setZero();
    Eigen::VectorXf areaSum(nVert);
    areaSum.setZero();
    for(int i = 0; i < nTri; ++i)
    {
        thickness_[tri_pts_(i,0)] += areaF[i]*thicknessF[i];
        thickness_[tri_pts_(i,1)] += areaF[i]*thicknessF[i];
        thickness_[tri_pts_(i,2)] += areaF[i]*thicknessF[i];

        areaSum[tri_pts_(i,0)] += areaF[i];
        areaSum[tri_pts_(i,1)] += areaF[i];
        areaSum[tri_pts_(i,2)] += areaF[i];
    }
    
    for(int i = 0; i < nVert; ++i)
    {
        thickness_[i] /= areaSum[i];
    }
    timer.stop();
    std::cout << "thickness - process time[s]: " << timer.count<std::chrono::seconds>() << std::endl;
}

void ImSampData::writeThickness(std::string file)
{
    std::ofstream fout(file);
    if(fout.is_open())
    {
        fout << thickness_;
        
        fout.close();
    }
    else
    {
        std::cerr << "ImSampData::writeThickness - cannot open " << file << std::endl;
    }
}

#ifdef WITH_IMGUI
void ImSampData::updateIMGUI()
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
