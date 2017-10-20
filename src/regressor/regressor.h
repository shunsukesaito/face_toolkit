//
//  regressor.hpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 9/19/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#ifndef regressor_hpp
#define regressor_hpp

#include<vector>
#include<utility>
#include<string>

#include<opencv2/opencv.hpp>

#include "fern.h"
#include "train_util.h"

namespace cao{

struct Regressor
{
    int K_ = 300;
    int P_ = 400;
    float kappa_ = 1.0;
    int stage_num_ = -1;
    
    std::vector<Fern> ferns_;
    std::vector<std::pair<int,cv::Vec2f>> points_; // where we are sampling pixel. triangle_id * baricentric(u,v)
    cv::Mat_<float> means_; // mean and SD for the shape vectors
    cv::Mat_<float> SDs_;

    Regressor(){}
    Regressor(int K, int P, int kappa, int F, int beta);
    ~Regressor(){}
    
    void setStage(int stage){stage_num_ = stage;}
    void setMeanStddev(const cv::Mat_<float>& mean, const cv::Mat_<float>& SDs){means_ = mean; SDs_ = SDs;}
    void train(std::vector<Data>& data,
               const cv::Mat_<cv::Vec2f>& mean_p2d,
               const cv::Mat_<int>& tri);

    void apply(cv::Mat_<float>& X,
               const cv::Mat_<cv::Vec2f>& p2d,
               const cv::Mat_<uchar>& img,
               const cv::Mat_<int>& tri,
               cv::Rect rect = cv::Rect());
    
    void apply(cv::Mat_<float>& X,
               const cv::Mat_<cv::Vec2f>& p2d,
               const cv::Mat_<uchar>& img,
               const cv::Mat_<int>& tri,
               const cv::Mat_<float>& pmap,
               const cv::Rect &rect);
    
    void normResidual(cv::Mat_<float>& res);
    void unnormResidual(cv::Mat_<float>& res);
    
    void write(cv::FileStorage &fs)const;
    void read(const cv::FileNode &fn);
};

void write(cv::FileStorage& fs, const std::string&, const Regressor& r);
void read(const cv::FileNode& node, Regressor& r, const Regressor&);
    
}

#endif /* regressor_hpp */
