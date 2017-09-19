//
//  fern.cpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 9/19/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#include "fern.h"

namespace cao{
    
static float Covariance(float *x, float *y, const int size)
{
    float a = 0, b = 0, c = 0;
    for(int i = 0; i < size; ++i)
    {
        a += x[i];
        b += y[i];
        c += x[i]*y[i];
    }
    
    return c/(float)size - (a/(float)size)*(b/(float)size);
}

void Fern::apply(cv::Mat_<float>& x, const cv::Mat_<float>& features)
{
    int output_index = 0;
    for(int i = 0; i < F_; ++i)
    {
        const std::pair<int, int>& feature = shape_index_[i];
        const float& p1 = features(feature.first);
        const float& p2 = features(feature.second);
        
        output_index |= (p1 - p2 > thresholds_[i]) << i;
    }
    
    x += res_.row(output_index);
}

void Fern::apply(cv::Mat_<float>& x, const cv::Mat_<float>& features,const cv::Mat_<float> prob_val)
{
    int output_index = 0;
    
    bool is_occluded = false;
    const float thresh = 0.8;
    for(int i = 0; i < F_; ++i)
    {
        std::pair<int, int> feature = shape_index_[i];
        float p1 = features(feature.first);
        float p2 = features(feature.second);
        
        if(prob_val(feature.first)>thresh || prob_val(feature.second)>thresh){
            is_occluded = true;
            return;
        }
        
        output_index |= (p1 - p2 > thresholds_[i]) << i;
    }
    
    x += res_.row(output_index);
}

void Fern::train(cv::Mat_<float>& res, cv::Mat_<float>& pixel_values, cv::Mat_<float>& pixels_cov)
{
    shape_index_.assign(F_, std::pair<int,int>());
    thresholds_.assign(F_, 0);
    
    std::cout << "      Correlation-based Sampling..." << std::endl;
    for(int i = 0; i < F_; ++i)
    {
        cv::Mat_<float> projection(res.cols,1);
        cv::theRNG().fill(projection, cv::RNG::UNIFORM, cv::Scalar(-1.0), cv::Scalar(1.0));
        cv::Mat_<float> Y_proj(res.rows,1);
        static_cast<cv::Mat_<float>>(res*projection).copyTo(Y_proj);
        cv::Mat_<float> Y_pixel_cov(pixel_values.size(),1);
        for(int j = 0; j < pixel_values.rows; ++j)
        {
            Y_pixel_cov(j) = Covariance(Y_proj.ptr<float>(0), pixel_values.ptr<float>(j), (int)Y_proj.total());
        }
        
        float max_corr = -1.0;
        for(int j = 0; j < pixel_values.rows; ++j)
        {
            for(int k = 0; k < pixel_values.rows; ++k)
            {
                float temp = (pixels_cov(j,j) + pixels_cov(k,k) - 2.f*pixels_cov(j,k));
                if(fabs(temp) < 1.0e-10) continue;
                float corr = (Y_pixel_cov(j) - Y_pixel_cov(k)) /
                sqrt(temp);
                if(fabs(corr) > max_corr){
                    max_corr = corr;
                    shape_index_[i].first = j;
                    shape_index_[i].second = k;
                }
            }
        }
        assert(max_corr != -1.0);
        
        float thresh_max = -1.0e5;
        float thresh_min = 1.0e5;
//        float mean = 0.0; // mean
//        float sqmean = 0.0; //squared mean
        for(int j = 0; j < pixel_values.cols; ++j)// for all training set
        {
            float value = pixel_values(shape_index_[i].first,j) - pixel_values(shape_index_[i].second,j);
            thresh_max = std::max(thresh_max,value);
            thresh_min = std::min(thresh_min,value);
//            mean += value;
//            sqmean += value*value;
        }
        assert(thresh_max != -1.0e5 && thresh_min != 1.0e5);
//        mean /= (float)pixel_values.cols;
//        sqmean /= (float)pixel_values.cols;
//        float SD = sqrt(sqmean - mean*mean);
        
        // NOTE: I have tested different sampling scheme. empirically working, but can be improved.
        //thresholds_[i] = mean + cv::theRNG().uniform(-SD*0.2, SD*0.2);
        thresholds_[i] = (thresh_max+thresh_min)/2.0+cv::theRNG().uniform(-(thresh_max-thresh_min)*0.1,(thresh_max-thresh_min)*0.1);

        //std::cout << "      " << thresholds_[i] << " " << thresh_max << " " << thresh_min << " " << mean/pixel_values.cols << std::endl;
    }
    assert(F_ == shape_index_.size());
    
    std::cout << "      Storing Residual Vectors in Each Bin..." << std::endl;
    int bin_size = 1 << F_;
    
    res_ = cv::Mat_<float>::zeros(bin_size, res_.cols);
    std::vector<int> each_res_count(bin_size,0);
    for(int i = 0; i < res.rows; ++i) // for all training set
    {
        int mask = 0;
        for(int j = 0; j < F_; ++j)
        {
            float value = pixel_values(shape_index_[j].first,i)-pixel_values(shape_index_[j].second,i);
            mask |= (value > thresholds_[j]) << j;
        }
        res_.row(mask) += res.row(i);
        each_res_count[mask]++;
    }
    
    std::cout << "      ";
    for(int i = 0; i < bin_size; ++i)
    {
        res.row(i) *= 1.0/(each_res_count[i]+beta_);
        std::cout << each_res_count[i] << " ";
    }
    std::cout << std::endl;
}

void Fern::write(cv::FileStorage &fs)const
{
    fs << "{";
    fs << "thresholds" << thresholds_;
    fs << "features_index";
    fs << "[";
    for (auto it = shape_index_.begin(); it != shape_index_.end(); ++it)
        fs << "{" << "first" << it->first << "second" << it->second << "}";
    fs << "]";
    fs << "regressor" << res_;
    fs << "}";
}

void Fern::read(const cv::FileNode &fn)
{
    thresholds_.clear();
    shape_index_.clear();
    fn["thresholds"] >> thresholds_;
    cv::FileNode features_index_node = fn["features_index"];
    for (auto it = features_index_node.begin(); it != features_index_node.end(); ++it)
    {
        std::pair<int, int> feature_index;
        (*it)["first"] >> feature_index.first;
        (*it)["second"] >> feature_index.second;
        shape_index_.push_back(feature_index);
    }
    fn["outputs"] >> res_;
    
    F_ = (int)shape_index_.size();
}

    void write(cv::FileStorage& fs, const std::string&, const Fern &f)
{
    f.write(fs);
}

void read(const cv::FileNode& node, Fern &f, const Fern&)
{
    if (node.empty())
        throw std::runtime_error("Model file is corrupt!");
    else
        f.read(node);
}

}
