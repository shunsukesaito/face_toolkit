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
#include "regularizer.h"

float computeJacobianPCAReg(Eigen::Ref<Eigen::VectorXf> Jtr,
                            Eigen::Ref<Eigen::MatrixXf> JtJ,
                            const Eigen::VectorXf& X,
                            const Eigen::VectorXf& sigma,
                            unsigned int start,
                            unsigned int size,
                            const float& w)
{
    assert(sigma.size() >= size);
    assert(X.size() == Jtr.size());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    assert(X.size() >= start + size);
    
    float error = 0.0;
    float wsq_sigma = 0.0;
    for (int i = 0; i < size; ++i)
    {
        wsq_sigma = w / sigma[i] / sigma[i];
        JtJ(start + i, start + i) += wsq_sigma;
        Jtr[start + i] += wsq_sigma * X[start + i];
        
        error += wsq_sigma * X[start + i] * X[start + i];
    }
    
    return error;
}

float computeJacobianL1Reg(Eigen::Ref<Eigen::VectorXf> Jtr,
                           Eigen::Ref<Eigen::MatrixXf> JtJ,
                           const Eigen::VectorXf& X,
                           unsigned int start,
                           unsigned int size,
                           const float& w)
{
    assert(X.size() == Jtr.size());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    assert(X.size() >= start + size);
    
    float error = 0.0;
    for(int i = 0; i < size; ++i)
    {
        JtJ(start + i, start + i) += w * 0.25f / (fabs(X[start + i]) + EPSILON);
        Jtr[start + i] += (X[start + i] >= 0) ? 0.5f * w : -0.5f * w;
        
        error += w * (fabs(X[start + i]) + EPSILON);
    }
    
    return error;
}

float computeJacobianL2Reg(Eigen::Ref<Eigen::VectorXf> Jtr,
                           Eigen::Ref<Eigen::MatrixXf> JtJ,
                           const Eigen::VectorXf& X,
                           unsigned int start,
                           unsigned int size,
                           const float& w)
{
    assert(X.size() == Jtr.size());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    assert(X.size() >= start + size);
    
    float error = 0.0;
    for(int i = 0; i < size; ++i)
    {
        JtJ(start + i, start + i) += w;
        Jtr[start + i] += w * X[start + i];
        
        error += w * X[start + i] * X[start + i];
    }
    return error;
}

float computeJacobianLMixReg(Eigen::Ref<Eigen::VectorXf> Jtr,
                             Eigen::Ref<Eigen::MatrixXf> JtJ,
                             const Eigen::VectorXf& X,
                             float l,
                             float u,
                             float lambda,
                             unsigned int start,
                             unsigned int size,
                             const float& w)
{
    assert(X.size() == Jtr.size());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    assert(X.size() >= start + size);
    
    if(w <= 0.0 || size <= 0) return 0.f;
    if(l < 0){
        std::cerr << "computeJacobianLMixReg() - l should be above 0" << std::endl;
        return 0.f;
    }
    float a,b,c,d;
    float error = 0.0;
    for(int i = 0; i < size; ++i)
    {
        const float& x = X[start + i];
        a = 0.5f / l;
        b = a * l * l - l;
        c = 1.f-2.f * lambda * u;
        d = u + b - lambda * u * u - c * u;
        if(x < 0){
            JtJ(start + i, start + i) += w * lambda;
            Jtr[start + i] += w * lambda * x;
            
            error += w * lambda * x * x;
        }
        else if(x < l){
            JtJ(start + i, start + i) += w * a;
            Jtr[start + i] += w * a * x;
            
            error += w * a * x * x;
        }
        else if(x > u){
            JtJ(start + i, start + i) += w * lambda;
            Jtr[start + i] += w * (lambda * x + 0.5 * c);
            
            error += w * (lambda * x * x + c * x + d);
        }
        else{
            JtJ(start + i, start + i) += w * 0.25f / (fabs(x) + EPSILON);
            Jtr[start + i] += (x >= 0) ? 0.5f * w : -0.5f * w;
            
            error += w * (fabs(x) + EPSILON + b);
        }
    }
    
    return error;
}

float computeJacobianTikhonovReg(Eigen::Ref<Eigen::VectorXf> Jtr,
                                 Eigen::Ref<Eigen::MatrixXf> JtJ,
                                 const Eigen::VectorXf& X,
                                 const Eigen::VectorXf& X0,
                                 unsigned int start,
                                 unsigned int size,
                                 const float w)
{
    assert(X.size() >= start + size);
    
    float error = 0.0;
    for(int i = start; i < start + size; ++i)
    {
        JtJ(i, i) += w;
        Jtr[i] += w * (X[i] - X0[i]);
        
        error += w * (X[i] - X0[i]) * (X[i] - X0[i]);
    }
    
    return error;
}

