
#pragma once

#include <iostream>
#include <math.h>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define b3(a) block<3,1>(3*(a), 0)

#define bc3(a) block<3,1>(0,a)

#define EPSILON 0.000001f

Eigen::Vector3f getCenter(Eigen::VectorXf& pts);

void calcNormal(Eigen::MatrixX3f& nml,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3i& tri);

void calcTangent(Eigen::MatrixX3f& tan,
                 Eigen::MatrixX3f& btan,
                 const Eigen::VectorXf& pts,
                 const Eigen::MatrixX2f& uvs,
                 const Eigen::MatrixX3i& tripts,
                 const Eigen::MatrixX3i& triuv);

namespace Eigen{
	typedef Matrix<float, 6, 1> Vector6f;
	
    double radiansToDegrees(double x);
    Matrix3f eulerAngleToMat(float angleradX, float angleradY, float angleradZ);
    Vector3f matToEulerAngle(Matrix3f R);
    Vector3f ApplyTransform(const Matrix4f& A, const Vector3f& p);
    
    Vector2f ApplyProjection(const Matrix4f& K, const Matrix4f& RT, const Vector3f& p);
    void EulerAnglesPoseToMatrix(const Vector6f& rt, Matrix4f& RT);

    Matrix4f EulerAnglesPoseToMatrix(const Vector6f& rt);
    void ConvertToEulerAnglesPose(const Matrix4f& RT, Vector6f& rt);
    Vector6f ConvertToEulerAnglesPose(const Matrix4f& RT);
    
    template<class T>
    inline void factorizeLLTSolver(const T& A, Eigen::SimplicialLLT<T>& lltSolver)
    {
        lltSolver.analyzePattern(A);
        lltSolver.factorize(A);
        
        if(lltSolver.info() != Eigen::Success)
        {
            std::cout << "Warning: " << "LLT decomposition is failed." << std::endl;
        }
    }
    
    template<class T>
    inline void factorizeLDLTSolver(const T& A, Eigen::SimplicialLDLT<T>& ldltSolver)
    {
        ldltSolver.analyzePattern(A);
        ldltSolver.factorize(A);
        
        if(ldltSolver.info() != Eigen::Success)
        {
            std::cout << "Warning: " << "LDLT decomposition is failed." << std::endl;
        }
    }
}
