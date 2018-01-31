//
//  ls_face_model.cpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 1/30/18.
//  Copyright © 2018 Shunsuke Saito. All rights reserved.
//

#include <utility/obj_loader.h>
#include <utility/exr_loader.h>
#include <gl_utility/gl_core.h>

#include "face_model.h"

void LinearFaceModel::loadLightStageData(const std::string& data_dir)
{
    // 0: obj, 1: diff_albedo, 2: diff_normal, 3: spec_albedo, 4: spec_normal, 5: displacement
    std::string files[6];
    std::ifstream fin(data_dir + "list.txt");
    if(fin.is_open()){
        for(int i = 0; i < 6; ++i)
        {
            fin >> files[i];
            files[i] = data_dir + files[i];
        }
    }
    
    Eigen::MatrixX3f nml;
    loadObjFile(files[0], mu_id_, nml, uvs_, tri_pts_, tri_uv_);
    maps_.resize(5);
    for(int i = 0; i < 5; ++i)
    {
        cv::Mat_<cv::Vec4f> tmp;
        loadEXRToCV(files[i+1], tmp);
        cv::resize(tmp,tmp,cv::Size(3000,3000));
        maps_[i] = GLTexture::CreateTexture(tmp);
    }
}
