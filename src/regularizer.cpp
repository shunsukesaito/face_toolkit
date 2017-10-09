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
    
    float a;
    float error = 0.0;
    for(int i = 0; i < size; ++i)
    {
        const float& x = X[start + i];
        if(x < 0){
            JtJ(start + i, start + i) += w * lambda;
            Jtr[start + i] += w * lambda * x;
            
            error += w * lambda * x * x;
        }
        else if(x < l){
            a = 0.5f / l;
            JtJ(start + i, start + i) += w * a;
            Jtr[start + i] += w * a * x;
            
            error += w * a * x * x;
        }
        else if(x > u){
            a = 0.5 * (1.0 + lambda) / u;
            JtJ(start + i, start + i) += w * a;
            Jtr[start + i] += w * (a * x + 0.5 * lambda);
            
            error += w * (a * x * x + 0.5 * lambda * x);
        }
        else{
            JtJ(start + i, start + i) += w * 0.25f / (fabs(X[start + i]) + EPSILON);
            Jtr[start + i] += (x >= 0) ? 0.5f * w : -0.5f * w;
            
            error += w * (fabs(X[start + i]) + EPSILON);
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

