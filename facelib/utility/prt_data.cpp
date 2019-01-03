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

#include "prt_data.h"
#include "timer.h"

void PRTData::diffuseDirectPRT(int band, Sampler &sampler, BVHTree &bvh, bool shadow)
{
    int nVert = pts_.size()/3;
    int band2 = band*band;
    int nSample = sampler.samples_.size();
    
    float weight = 4.0f*(float)M_PI/(float)nSample;
    
    cxxtimer::Timer timer;
    timer.start();
#pragma omp parallel for
    for(int i = 0; i < nVert; ++i)
    {
        const Eigen::RowVector3f& n = nml_.row(i);
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
        //std::cout << prt_.row(i) << std::endl;
        //std::cout << "Point: " << i << " total: " << nSample << " neg: " << negSize << " noHit: " << noSize << std::endl;
    }
    
    prt_ *= weight;
    
    timer.stop();
    std::cout << "diffusePRT - process time: " << timer.count<std::chrono::seconds>() << std::endl;
}

void PRTData::diffuseInterRefPRT(int band, Sampler &sampler, BVHTree &bvh, int bounce)
{
    int nVert = pts_.size()/3;
    int band2 = band*band;
    int nSample = sampler.samples_.size();
    
    float weight = 4.0f*(float)M_PI/(float)nSample;
    
    intrRef_.resize(bounce, Eigen::MatrixXf::Zero(prt_.rows(), prt_.cols()));
    Eigen::MatrixXf curPrt;
    
    Eigen::Vector3f pTmp;
    float w;
    
    for(int b = 0; b < bounce; ++b)
    {
        if(b == 0)
            curPrt = prt_;
        else
            curPrt = intrRef_[b-1];
        
        cxxtimer::Timer timer;
        timer.start();
        
#pragma omp parallel for
        for(int i = 0; i < nVert; ++i)
        {
            const Eigen::RowVector3f& n = nml_.row(i);
            const Eigen::Vector3f& p = pts_.b3(i);
            int negSize = 0;
            int hitSize = 0; // non-occluded size
            for(int j = 0; j < nSample; ++j)
            {
                const Sample& smpl = sampler.samples_[j];
                float dot = n.dot(smpl.dir_);
                
                if(dot <= 0.0f)
                {
                    negSize++;
                    continue;
                }
                
                Ray ray(p, smpl.dir_);
                if(!bvh.interTest(ray))
                    continue;
            
                hitSize++;
                
                Eigen::RowVectorXf shTmp;
                shTmp.setZero();
                
                w = 1.0f - (ray.u_ + ray.v_);
                shTmp = ray.u_ * curPrt.row(tri_pts_(ray.idx_,0))
                      + ray.v_ * curPrt.row(tri_pts_(ray.idx_,1))
                      + w * curPrt.row(tri_pts_(ray.idx_,2));
                
                for(int k = 0; k < band2; ++k)
                {
                    intrRef_[b](i,k) += smpl.shValue_[k] * dot * shTmp[k];
                }
            }
            //std::cout << "Point: " << i << " total: " << nSample << " neg: " << negSize << " noHit: " << noSize << std::endl;
        }
        
        intrRef_[b] *= weight;
        timer.stop();
        std::cout << "interreflect " << b << " - process time: " << timer.count<std::chrono::seconds>() << std::endl;
    }
}

void PRTData::diffusePRT(int band, Sampler &sampler, BVHTree &bvh, bool shadow, int bounce)
{
    int nVert = pts_.size()/3;
    int band2 = band*band;
    int nSample = sampler.samples_.size();
    
    float weight = 4.0f*(float)M_PI/(float)nSample;
    
    cxxtimer::Timer timer;
    timer.start();
    // store interection info for interreflection
    std::vector<std::vector<Ray>> hitRays(nVert);
    std::vector<std::vector<float>> hitDots(nVert);
    std::vector<std::vector<int>> hitSmpIdxs(nVert);
    
#pragma omp parallel for
    for(int i = 0; i < nVert; ++i)
    {
        const Eigen::RowVector3f& n = nml_.row(i);
        int negSize = 0;
        int noSize = 0; // non-occluded size
        std::vector<Ray> rays;
        std::vector<float> dots;
        std::vector<int> smpIdxs;
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

                if(bvh.interTest(ray))
                {
                    rays.push_back(ray);
                    dots.push_back(dot);
                    smpIdxs.push_back(j);
                    continue;
                }
            }
            
            noSize++;
            
            for(int k = 0; k < band2; ++k)
            {
                prt_(i,k) += smpl.shValue_[k] * dot;
            }
        }
        hitRays[i] = rays;
        hitDots[i] = dots;
        hitSmpIdxs[i] = smpIdxs;

        //std::cout << "Point: " << i << " total: " << nSample << " neg: " << negSize << " noHit: " << noSize << std::endl;
    }
    
    prt_ *= weight;
    
    timer.stop();
    std::cout << "diffusePRT - process time[s]: " << timer.count<std::chrono::seconds>() << std::endl;
    
    intrRef_.resize(bounce, Eigen::MatrixXf::Zero(prt_.rows(), prt_.cols()));
    Eigen::MatrixXf curPrt;
    for(int b = 0; b < bounce; ++b)
    {
        if(b == 0)
            curPrt = prt_;
        else
            curPrt = intrRef_[b-1];
        
        cxxtimer::Timer timer;
        timer.start();
        
#pragma omp parallel for
        for(int i = 0; i < nVert; ++i)
        {
            const std::vector<Ray>& rays = hitRays[i];
            for(int j = 0; j < rays.size(); ++j)
            {
                const Ray& ray = rays[j];
                const Sample& smpl = sampler.samples_[hitSmpIdxs[i][j]];
                
                Eigen::RowVectorXf shTmp;
                shTmp.setZero();
                
                float w = 1.0f - (ray.u_ + ray.v_);
                shTmp = ray.u_ * curPrt.row(tri_pts_(ray.idx_,0))
                + ray.v_ * curPrt.row(tri_pts_(ray.idx_,1))
                + w * curPrt.row(tri_pts_(ray.idx_,2));
                
                for(int k = 0; k < band2; ++k)
                {
                    intrRef_[b](i,k) += smpl.shValue_[k] * hitDots[i][j] * shTmp[k];
                }
            }
        }
        
        intrRef_[b] *= weight;
        timer.stop();
        std::cout << "interreflect " << b << " - process time[ms]: " << timer.count<std::chrono::milliseconds>() << std::endl;
    }
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
    
    diffusePRT(band, sampler, bvh, shadow, bounce);
    
    totPrt_.resize(bounce+1);
    totPrt_[0] = prt_;
    for(int i = 0; i < bounce; ++i)
    {
        totPrt_[i+1] = totPrt_[i] + intrRef_[i];
    }
}

void PRTData::writePRT(std::string file)
{
    std::ofstream fout(file);
    if(fout.is_open())
    {
        fout << prt_;
        
        fout.close();
    }
    else
    {
        std::cerr << "PRTData::writePRT - cannot open " << file << std::endl;
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
