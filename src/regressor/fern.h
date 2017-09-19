//
//  fern.hpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 9/19/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#ifndef fern_hpp
#define fern_hpp

#include<vector>
#include<utility>
#include<string>

#include<opencv2/opencv.hpp>

namespace cao{

struct Fern
{
    int F_ = 5;
    int beta_ = 250;
    
    std::vector<float> thresholds_;
    std::vector<std::pair<int, int>> shape_index_;
    cv::Mat_<float> res_;
    
    Fern(){}
    Fern(int F, int beta) : F_(F), beta_(beta){}
    ~Fern(){}
    
    void train(cv::Mat_<float>& residual, cv::Mat_<float>& pixel_values, cv::Mat_<float>& pixels_cov);
    void apply(cv::Mat_<float>& x, const cv::Mat_<float>& features);
    void apply(cv::Mat_<float>& x, const cv::Mat_<float>& features,const cv::Mat_<float> prob_val);
    
    void write(cv::FileStorage &fs)const;
    void read(const cv::FileNode &fn);
};

void write(cv::FileStorage& fs, const std::string&, const Fern& f);
void read(const cv::FileNode& node, Fern& f, const Fern&);
    
}

#endif /* fern_hpp */
