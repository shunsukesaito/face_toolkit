//
//  DEM_adaptor.hpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 10/12/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#pragma once

#include <deque>

#include "train_util.h"

namespace cao
{
    struct DEMAdaptor
    {
        cv::Mat_<float> Vset_; // set of concatenated row vector of rotation and expression
        cv::Mat_<float> M_; // eigen vector matrix
        cv::Mat_<float> mean_; // mean of Vset
        
        std::deque<Data> data_;
        
        int L_ = 5; // number of sampling frames
        float thresh_ = 0.05; // threshold for frame sampling
        
        void Init();
        void Reset();
        
        bool AdaptDEM(Data& data);
        bool AddVector(const Data& data);
        
        void updatePCA();
        float getPCARcnErr(const cv::Mat_<float>& V);
        
        void identityAdaptation(cv::Mat_<float>& id);
        float focalAdaptation(const cv::Mat_<float>& id);
        
        void updateShapeVectors(const cv::Mat_<float>& id, float fl);
    };
};
