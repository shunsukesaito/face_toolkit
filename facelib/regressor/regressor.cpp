//
//  regressor.cpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 9/19/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#include "regressor.h"
#include <iostream>

namespace cao{

static void calcTriangleCenters(cv::Mat_<cv::Vec2f>& c,
                               const cv::Mat_<cv::Vec2f>& x, //n * 2
                               const cv::Mat_<int>& tri) // n_tri * 3
{
    c = cv::Mat_<cv::Vec2f>::zeros(tri.rows, 1);
    for(int i = 0; i < tri.rows; ++i)
    {
        c(i, 0) = (x(tri(i,0))+x(tri(i,1))+x(tri(i,2)))/3.0;
    }
}

static void calcBaryCentric(cv::Vec2f& vw,
                            const cv::Vec2f& p,
                            const cv::Vec2f& a,
                            const cv::Vec2f& b,
                            const cv::Vec2f& c)
{
    cv::Vec2f v0 = b - a, v1 = c - a, v2 = p - a;
    
    if(v0(0)*v1(1)-v0(1)*v1(0) == 0){
        vw = cv::Vec2f(std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest());
        return;
    }
    
    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom = d00 * d11 - d01 * d01;
    
    vw(0) = (d11 * d20 - d01 * d21) / denom;
    vw(1) = (d00 * d21 - d01 * d20) / denom;
}


Regressor::Regressor(int K, int P, int kappa, int F, int beta)
: K_(K), P_(P), kappa_(kappa)
{
    ferns_ = std::vector<Fern>(K_, Fern(F, beta));
}

void Regressor::train(std::vector<Data>& data,
                      const cv::Mat_<cv::Vec2f>& mean_p2d,
                      const cv::Mat_<int>& tri)
{
    assert(data.size() != 0);
    
    const int n_sample = (int)data.size();
    const int len_x = data[0].gt_x.cols;
    const int n_p2d = data[0].gt_p2d.rows;
    
    // compute residual
    std::cout << "  Computing Residual..." << std::endl;
    
    float sum_x_res = 0.0;
    float sum_p_res = 0.0;
    
    cv::Mat_<float> res = cv::Mat_<float>::zeros(n_sample, len_x);
    for(int i = 0; i < n_sample; ++i)
    {
        res.row(i) = data[i].gt_x - data[i].cur_x;
        sum_x_res += cv::norm(res.row(i));
        for(int j = 0; j < n_p2d; ++j)
        {
            sum_p_res += cv::norm(data[i].gt_p2d(j)-data[i].cur_p2d(j));
        }
    }
    
    std::cout << "  Shape Vector Residual: " << sum_x_res/(float)n_sample;
    std::cout << " 2Dlandmark norm: " << sum_p_res/(float)(n_sample*n_p2d) << std::endl;
    
    normResidual(res); // project residuals to nomarlized space where mean is 0 and SD is 1.
    
    std::cout << "  Computing Shape Index Features..." << std::endl;
    // determine sampling pixels using baricentric coordinates

    cv::Mat_<cv::Vec2f> c_tri;
    calcTriangleCenters(c_tri, mean_p2d, tri);
    int index = 0;
    cv::Vec2f offset;
    for(int i = 0; i < P_; ++i)
    {
        index = cv::theRNG().uniform(0, n_p2d);

        while(1){
            offset(0) = cv::theRNG().uniform(-kappa_, kappa_);
            offset(1) = cv::theRNG().uniform(-kappa_, kappa_);
            cv::Vec2f p = mean_p2d(index) + offset;
            
            cv::Vec2f bc;
            int tri_index = -1;
            for(int j = 0; j < tri.rows; ++j)
            {
                calcBaryCentric(bc, p, mean_p2d(tri(j,0)), mean_p2d(tri(j,1)), mean_p2d(tri(j,2)));
                if(bc(0) >= 0 && bc(1) >= 0 && bc(0)+bc(1) <= 1.0){
                    tri_index = j;
                    break;
                }
            }
            if(tri_index == -1){
                continue;
            }
            else{
                points_.push_back(std::make_pair(tri_index, bc));
                break;
            }
        }
    }
    
    // for debug of meanshape
    cv::Mat_<cv::Vec3b> debug_img(300,300,cv::Vec3b(255,255,255));
 
    for(int i = 0; i < tri.rows; ++i)
    {
        cv::Point p1 = cv::Vec2i(50.0*mean_p2d(tri(i,0)))+cv::Vec2i(150.0,150.0);
        cv::Point p2 = cv::Vec2i(50.0*mean_p2d(tri(i,1)))+cv::Vec2i(150.0,150.0);
        cv::Point p3 = cv::Vec2i(50.0*mean_p2d(tri(i,2)))+cv::Vec2i(150.0,150.0);
        cv::line(debug_img, p1, p2, cv::Scalar(0,0,0));
        cv::line(debug_img, p1, p3, cv::Scalar(0,0,0));
        cv::line(debug_img, p2, p3, cv::Scalar(0,0,0));
    }
    
    for(int i = 0; i < mean_p2d.rows; ++i)
    {
        cv::Point p = cv::Vec2i(50.0*mean_p2d(i))+cv::Vec2i(150.0,150.0);
        cv::circle(debug_img, p, 1, cv::Scalar(0,0,0),-1);
    }

    std::vector<cv::Point> sampled_pts;
    for(auto&& pt : points_)
    {
        const cv::Vec2f& p0 = mean_p2d(tri(pt.first,0));
        const cv::Vec2f& p1 = mean_p2d(tri(pt.first,1));
        const cv::Vec2f& p2 = mean_p2d(tri(pt.first,2));
                                                                                        
        cv::Vec2f p_bc = p1+pt.second[0]*(p1-p0)+pt.second[1]*(p2-p0);
        cv::Point p(p_bc(0)*50+150,p_bc(1)*50+150);
        sampled_pts.push_back(p);
        cv::circle(debug_img, p, 1, cv::Scalar(0,255,0),-1);
    }
    
    std::cout << "  Sampling Pixel Intensities..." << std::endl;
    // sample pixel intensities
    cv::Mat_<float> pixels_val(P_, n_sample);
    for(int i = 0; i < n_sample; ++i)
    {
        for(int j = 0; j < P_; ++j)
        {
            const auto& pt = points_[j];
            
            const cv::Vec2f& p0 = data[i].cur_p2d(tri(pt.first,0));
            const cv::Vec2f& p1 = data[i].cur_p2d(tri(pt.first,1));
            const cv::Vec2f& p2 = data[i].cur_p2d(tri(pt.first,2));
            
            cv::Vec2f p = p1+pt.second[0]*(p1-p0)+pt.second[1]*(p2-p0);
            
            cv::Point pixel_pos(p(0)-data[i].rect.x,p(1)-data[i].rect.y);
            if(pixel_pos.inside(cv::Rect(0,0,data[i].rect.width,data[i].rect.height))){
                // non-face area will be filled with black
#ifdef ENABLE_OH
                if(mask[i](pixel_pos) > 200)
                    pixels_val(j,i) = (float)data[i].img(pixel_pos);
                else
                    pixels_val(j,i) = 0.f;
#else
                pixels_val(j,i) = (float)data[i].img(pixel_pos);
#endif
            }
            else{
                pixels_val(j,i) = 0.f;
            }
        }
    }
    
    // train fern
    cv::Mat_<float> pixels_cov, means;
    cv::calcCovarMatrix(pixels_val, pixels_cov, means,
                        cv::COVAR_NORMAL | cv::COVAR_SCALE | cv::COVAR_COLS, CV_32F);
    
    std::cout << "  Starting fern Training..." << std::endl;
    cv::Mat_<float> dx = cv::Mat_<float>::zeros(res.size());
    for(int i = 0; i < K_; ++i)
    {
        ferns_[i].train(res, pixels_val, pixels_cov);

        float total_res_norm = 0.0;
        
        for(int j = 0; j < n_sample; ++j)
        {
            cv::Mat_<float> r = cv::Mat_<float>::zeros(1, res.cols);
            ferns_[i].apply(r, pixels_val(cv::Range::all(),cv::Range(j,j+1)));
            res.row(j) -= r;
            dx.row(j) += r;

            total_res_norm += cv::norm(res.row(j));
        }

        std::cout << "  fern Layer " << i << "th Stage: Average Residual Norm " << total_res_norm/n_sample << std::endl;
        
        // for debug
        {
            cv::Mat_<cv::Vec3b> debug_cpy = debug_img.clone();
            const std::vector<std::pair<int, int>>& shape_index = ferns_[i].shape_index_;
            
            for(auto&& si : shape_index)
                cv::line(debug_cpy, sampled_pts[si.first], sampled_pts[si.second], cv::Scalar(0,0,255));
            
            char stage[256];
            sprintf(stage, "Stage %d",stage_num_);

            cv::putText(debug_cpy, stage, cv::Point(10,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));

            cv::imshow("Shape Index Feature", debug_cpy);
            cv::waitKey(1);
        }
    }
    
    std::cout << "  Applying Trained Regressor..." << std::endl;
    unnormResidual(dx);
    
    for(int i = 0; i < n_sample; ++i)
    {
        data[i].cur_x += dx.row(i);
    }
}

void Regressor::apply(cv::Mat_<float>& X,
                      const cv::Mat_<cv::Vec2f>& p2d,
                      const cv::Mat_<uchar>& img,
                      const cv::Mat_<int>& tri,
                      cv::Rect rect)
{
    // sample pixel values
    cv::Mat_<float> pixels_val(1,P_);
    cv::Point pixel_pos;
    
    if(rect.area() == 0)
        rect = cv::Rect(0,0,img.cols,img.rows);

    for(int i = 0; i < P_; ++i)
    {
        const auto& pt = points_[i];
        const cv::Vec2f& p0 = p2d(tri(pt.first,0));
        const cv::Vec2f& p1 = p2d(tri(pt.first,1));
        const cv::Vec2f& p2 = p2d(tri(pt.first,2));

        cv::Vec2f p = p1+pt.second[0]*(p1-p0)+pt.second[1]*(p2-p0);
        
        pixel_pos.x = (p(0)-rect.x);
        pixel_pos.y = (p(1)-rect.y);
        if(pixel_pos.inside(cv::Rect(0,0,rect.width,rect.height))){
            pixels_val(0,i) = img(pixel_pos);
        }
        else{
            pixels_val(0,i) = 0.f;
        }
    }

    for(int i = 0; i < K_; ++i)
    {
        ferns_[i].apply(X, pixels_val);
    }
    
    unnormResidual(X);
}

void Regressor::apply(cv::Mat_<float>& X,
                      const cv::Mat_<cv::Vec2f>& p2d,
                      const cv::Mat_<uchar>& img,
                      const cv::Mat_<int>& tri,
                      const cv::Mat_<float>& pmap,
                      const cv::Rect &rect)
{
    // sample pixel values
    cv::Mat_<float> pixels_val(1,P_);
    cv::Mat_<float> prob_val(1,P_);

    for(int i = 0; i < P_; ++i)
    {
        const auto& pt = points_[i];
        const cv::Vec2f& p0 = p2d(tri(pt.first,0));
        const cv::Vec2f& p1 = p2d(tri(pt.first,1));
        const cv::Vec2f& p2 = p2d(tri(pt.first,2));
        
        cv::Vec2f p = p1+pt.second[0]*(p1-p0)+pt.second[1]*(p2-p0);
        
        cv::Point pixel_pos(p(0),p(1));
        if(pixel_pos.inside(cv::Rect(0,0,img.cols,img.rows))){
            pixels_val(0,i) = img(pixel_pos);
            if(pixel_pos.inside(rect)){
                prob_val(0,i) = pmap(p(1)-rect.y,p(0)-rect.x);
            }
            else{
                prob_val(0,i) = 1.0f;
            }
        }
        else{
            pixels_val(0,i) = 0.f;
            prob_val(0,i) = 1.0f;
        }
    }
    
    for(int i = 0; i < K_; ++i)
    {
        ferns_[i].apply(X, pixels_val);
    }
    unnormResidual(X);
}

void Regressor::normResidual(cv::Mat_<float>& res)
{
    if(stage_num_ == 0){
        means_ = cv::Mat_<float>::zeros(1, res.cols);
        SDs_ = cv::Mat_<float>::zeros(1, res.cols);
        
        cv::meanStdDev(res.t(), means_, SDs_);
        for(int i = 0; i < SDs_.total(); ++i)
            if(SDs_(i) == 0) SDs_(i) = 1.0;
    }
    for(int i = 0; i < res.rows; ++i)
        cv::divide(res.row(i)-means_, SDs_, res.row(i));
}

void Regressor::unnormResidual(cv::Mat_<float>& res)
{
    for(int i = 0; i < res.rows; ++i)
    {
        // TODO check if it works
        cv::multiply(res.row(i), SDs_, res.row(i));
        res.row(i) += means_;
    }
}


void Regressor::write(cv::FileStorage &fs)const
{
    assert(means_.size() == SDs_.size());
    
    fs << "{";
    fs << "pixels";
    fs << "[";
    for (auto it = points_.begin(); it != points_.end(); ++it)
        fs << "{" << "first" << it->first << "second1" << it->second(0) << "second2" << it->second(1) << "}";
    fs << "]";
    fs << "ferns" << "[";
    for (auto it = ferns_.begin(); it != ferns_.end(); ++it)
        fs << *it;
    fs << "]";
    fs << "means" << means_;
    fs << "SDs" << SDs_;
    fs << "}";
}


void Regressor::read(const cv::FileNode &fn)
{
    points_.clear();
    ferns_.clear();
    cv::FileNode pixels_node = fn["pixels"];
    for (auto it = pixels_node.begin(); it != pixels_node.end(); ++it)
    {
        std::pair<int, cv::Vec2f> pixel;
        (*it)["first"] >> pixel.first;
        (*it)["second1"] >> pixel.second(0);
        (*it)["second2"] >> pixel.second(1);
        points_.push_back(pixel);
    }
    cv::FileNode ferns_node = fn["ferns"];
    for (auto it = ferns_node.begin(); it != ferns_node.end(); ++it)
    {
        Fern f;
        *it >> f;
        ferns_.push_back(f);
    }
    
    fn["means"] >> means_;
    fn["SDs"] >> SDs_;
    
    K_ = ferns_.size();
    P_ = points_.size();
}

void write(cv::FileStorage& fs, const std::string&, const Regressor& r)
{
    r.write(fs);
}

void read(const cv::FileNode& node, Regressor& r, const Regressor&)
{
    if (node.empty())
        throw std::runtime_error("Model file is corrupt!");
    else
        r.read(node);
}
    
}
