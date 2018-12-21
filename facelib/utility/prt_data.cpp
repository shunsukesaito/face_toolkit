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

#include "prt_data.h"

void PRTData::diffuseDirectPRT(int band, Sampler &sampler, BVHTree &bvh, bool shadow)
{
    int nVert = pts_.size()/3;
    int band2 = band*band;
    int nSample = sampler.samples_.size();
    
    float weight = 4.0f*(float)M_PI/(float)nSample;
    
#pragma omp parallel for
    for(int i = 0; i < nVert; ++i)
    {
        const Eigen::Vector3f& n = nml_.row(i).transpose();
        int negSize = 0;
        int noSize = 0; // non-occluded size
        for(int j = 0; j < nSample; ++j)
        {
            const Sample& smpl = sampler.samples_[j];
            float dot = n.dot(smpl.dir_);
            
            if(dot <= 0.0f)
            {
                negSize++;
                continue;
            }
            
            if(shadow)
            {
                Ray ray(pts_.b3(i), smpl.dir_);
//                ray.o_ += 2.e-3f * n; // to make sure it won't intersect itself
                if(bvh.interTest(ray))
                    continue;
            }
            
            noSize++;
            
            for(int k = 0; k < band2; ++k)
            {
                prt_(i,k) += smpl.shValue_[k] * dot;
            }
        }
        
        for(int k = 0; k < band2; ++k)
        {
            prt_(i,k) *= weight;
        }
        //std::cout << prt_.row(i) << std::endl;
        //std::cout << "Point: " << i << " total: " << nSample << " neg: " << negSize << " noHit: " << noSize << std::endl;
    }
}

void PRTData::diffuseInterRefPRT(int band, Sampler &sampler, BVHTree &bvh, int bounce)
{
    
}

void PRTData::computePRT(int band, int nSample, bool shadow, int bounce)
{
    // create sampler
    Sampler sampler((unsigned)sqrt(nSample));
    sampler.computeSH(band);
    
    // build bvh tree
    BVHTree bvh;
    bvh.build(dynamic_cast<MeshData*>(this));
    
    prt_.resize(pts_.size()/3, band*band);
    prt_.setZero();
    
    diffuseDirectPRT(band, sampler, bvh, shadow);

    if(bounce > 0){
        diffuseInterRefPRT(band, sampler, bvh, bounce);
    }
}

#ifdef WITH_IMGUI
void PRTData::updateIMGUI()
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
