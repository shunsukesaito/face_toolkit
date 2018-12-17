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

#include <utility/obj_loader.h>
#include <utility/exr_loader.h>
#include <gl_utility/gl_core.h>

#include "face_model.h"

void LinearFaceModel::loadLightStageData(const std::string& data_dir)
{
    // 0: obj, 1: displacement, 2: diff_albedo, 3: spec_albedo, 4: diff_normal, 5: spec_normal
    std::string files[6];
    std::ifstream fin(data_dir + "list.txt");
    if(fin.is_open()){
        for(int i = 0; i < 6; ++i)
        {
            fin >> files[i];
            files[i] = data_dir + files[i];
        }
    }
    
    loadMeanFromObj(files[0]);
    maps_.resize(5);
    for(int i = 0; i < 5; ++i)
    {
        cv::Mat_<cv::Vec4f> tmp;
        loadEXRToCV(files[i+1], tmp);
        cv::resize(tmp,tmp,cv::Size(3000,3000));
        maps_[i] = GLTexture::CreateTexture(tmp);
    }
}

void LinearFaceModel::loadDeepLSData(const std::string& data_dir)
{
    // 0: obj, 1: displacement, 2: diff_albedo(png), 3: spec_albedo(png), 
    std::string files[4];
    std::ifstream fin(data_dir + "list.txt");
    if(fin.is_open()){
        for(int i = 0; i < 4; ++i)
        {
            fin >> files[i];
            files[i] = data_dir + files[i];
        }
    }
    
    loadMeanFromObj(files[0]);
    maps_.resize(3);
    
    cv::Mat_<cv::Vec4f> disp;
    loadEXRToCV(files[1], disp);
    maps_[0] = GLTexture::CreateTexture(disp);
    cv::Mat_<cv::Vec3b> diff = cv::imread(files[2]);
    cv::flip(diff,diff,0);
    maps_[1] = GLTexture::CreateTexture(diff);
    cv::Mat_<cv::Vec3b> spec = cv::imread(files[3]);
    cv::flip(spec,spec,0);
    maps_[2] = GLTexture::CreateTexture(spec);
}

FaceModelPtr LinearFaceModel::LoadLSData(const std::string &data_dir, bool deep)
{
    auto model = new LinearFaceModel();
    
    if(deep){
        model->fm_type_ = "deepls";
        model->loadDeepLSData(data_dir);
    }
    else{
        model->fm_type_ = "ls";
        model->loadLightStageData(data_dir);
    }

    std::cout << "Face Model Info:" << std::endl;
    std::cout << "#Vert: " << model->mu_id_.size()/3 << " #Tri: " << model->tri_pts_.rows() << std::endl;

    return FaceModelPtr(model);
}
