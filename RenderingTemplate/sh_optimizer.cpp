//
//  sh_optimizer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/11/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "sh_optimizer.hpp"
#include "minitrace.h"

void evaluateSHLeastSquare(Eigen::VectorXf& Crgb, Eigen::MatrixXf& CCrgb, const cv::Mat_<cv::Vec4f>& input, const cv::Mat_<cv::Vec4f>& albedo, const cv::Mat_<cv::Vec4f>& normal, unsigned int sampling_rate)
{
    MTR_SCOPE("SH", "evaluateSHLeastSquare");
    assert(input.size() == albedo.size());
    
    float w = 0.2f;
    const float c1 = 0.429043f, c2 = 0.511664f, c3 = 0.743125f, c4 = 0.886227f, c5 = 0.247708f;
    for (int x = 0; x < input.cols; x += sampling_rate)
    {
        for (int y = 0; y < input.rows; y += sampling_rate)
        {
            // if I is green (0,1,0), discard
            if (albedo(y, x)[3] != 0.0)
            {
                const auto& c = input(y, x);
                const auto& a = albedo(y, x);
                const auto& n = normal(y, x);
                
                Eigen::VectorXf C(9);
                C <<
                c4,
                2.0f*c2*n[1],
                2.0f*c2*n[2],
                2.0f*c2*n[0],
                2.0f*c1*n[0] * n[1],
                2.0f*c1*n[1] * n[2],
                c3*n[2] * n[2] - c5,
                2.0f*c1*n[0] * n[2],
                c1*(n[0] * n[0] - n[1] * n[1]);
                
                if (Crgb.size() == 27){
                    Crgb.block(0, 0, 9, 1) += c[0] * a[0] * C;
                    Crgb.block(9, 0, 9, 1) += c[1] * a[1] * C;
                    Crgb.block(18, 0, 9, 1) += c[2] * a[2] * C;
                    
                    CCrgb.block(0, 0, 9, 9) += a[0] * a[0] * C*C.transpose();
                    CCrgb.block(9, 9, 9, 9) += a[1] * a[1] * C*C.transpose();
                    CCrgb.block(18, 18, 9, 9) += a[2] * a[2] * C*C.transpose();
                }
                else if (Crgb.size() == 9){
                    Crgb += c[0] * a[0] * C;
                    Crgb += c[1] * a[1] * C;
                    Crgb += c[2] * a[2] * C;
                    
                    CCrgb += a[0] * a[0] * C*C.transpose();
                    CCrgb += a[1] * a[1] * C*C.transpose();
                    CCrgb += a[2] * a[2] * C*C.transpose();
                }
                else{
                    std::cout << "Error: wrong matrix size" << std::endl;
                    abort();
                }
                
                C <<
                c4,
                2.0f*c2*n[1],
                -2.0f*c2*n[2],
                2.0f*c2*n[0],
                2.0f*c1*n[0] * n[1],
                -2.0f*c1*n[1] * n[2],
                c3*n[2] * n[2] - c5,
                -2.0f*c1*n[0] * n[2],
                c1*(n[0] * n[0] - n[1] * n[1]);
                
                if (Crgb.size() == 27){
                    Crgb.block(0, 0, 9, 1) += w * c[0] * a[0] * C;
                    Crgb.block(9, 0, 9, 1) += w * c[1] * a[1] * C;
                    Crgb.block(18, 0, 9, 1) += w * c[2] * a[2] * C;
                    
                    CCrgb.block(0, 0, 9, 9) += w * a[0] * a[0] * C*C.transpose();
                    CCrgb.block(9, 9, 9, 9) += w * a[1] * a[1] * C*C.transpose();
                    CCrgb.block(18, 18, 9, 9) += w * a[2] * a[2] * C*C.transpose();
                }
                else if (Crgb.size() == 9){
                    Crgb += w * c[0] * a[0] * C;
                    Crgb += w * c[1] * a[1] * C;
                    Crgb += w * c[2] * a[2] * C;
                    
                    CCrgb += w * a[0] * a[0] * C*C.transpose();
                    CCrgb += w * a[1] * a[1] * C*C.transpose();
                    CCrgb += w * a[2] * a[2] * C*C.transpose();
                }
                else{
                    std::cout << "Error: wrong matrix size" << std::endl;
                    abort();
                }
            }
        }
    }
}

void estimateSH(Eigen::MatrixX3f& SHCoeffs, const cv::Mat_<cv::Vec4f>& input, const cv::Mat_<cv::Vec4f>& albedo, const cv::Mat_<cv::Vec4f>& normal, int SH)
{
    MTR_SCOPE("SH", "estimateSH");
    Eigen::VectorXf X(SH);
    Eigen::VectorXf Crgb(SH); Crgb.setZero();
    Eigen::MatrixXf CCrgb(SH, SH); CCrgb.setZero();
    
    evaluateSHLeastSquare(Crgb, CCrgb, input, albedo, normal);
    
    X = CCrgb.ldlt().solve(Crgb);
    
    if (SH == 27){
        for (int i = 0; i < 9; ++i)
        {
            SHCoeffs(i,0) = X(9 * 0 + i);
            SHCoeffs(i,1) = X(9 * 1 + i);
            SHCoeffs(i,2) = X(9 * 2 + i);
        }
    }
    else if (SH == 9){
        for (int i = 0; i < 9; ++i)
        {
            SHCoeffs(i,0) = X(i);
            SHCoeffs(i,1) = X(i);
            SHCoeffs(i,2) = X(i);
        }
    }
    else{
        std::cout << "Error: wrong SH size" << std::endl;
        abort();
    }
}
