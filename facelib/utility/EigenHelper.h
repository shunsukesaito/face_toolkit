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

void computeAABB(const Eigen::VectorXf& pts, Eigen::Vector3f& vMin, Eigen::Vector3f& vMax);

void baryCentric(const Eigen::Vector3f& p,
                 const Eigen::Vector3f& v1,
                 const Eigen::Vector3f& v2,
                 const Eigen::Vector3f& v3,
                 float &u, float &v, float &w);

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
    
    template <class T>
    Eigen::SparseMatrix<T> load_sparse_matrix(FILE* fp)
    {
        int property[3];
        fread(&property[0], sizeof(int), 3, fp);
        //std::cout << property[0] << " " << property[1] << " " << property[2] << std::endl;
        std::vector<T> data(property[2]);
        std::vector<int> row(property[2]), col(property[2]);
        
        fread(&row[0], sizeof(int), property[2], fp);
        fread(&col[0], sizeof(int), property[2], fp);
        fread(&data[0], sizeof(T), property[2], fp);
        
        std::vector< Eigen::Triplet<T> > triplets;
        for(int i = 0; i < property[2]; ++i)
            triplets.push_back(Eigen::Triplet<T>(row[i],col[i],data[i]));
        Eigen::SparseMatrix<T> sp(property[0],property[1]);
        sp.setFromTriplets(triplets.begin(), triplets.end());
        
        return sp;
    }
    
    template <class T>
    Eigen::Matrix<T,-1,-1> load_matrix(FILE* fp)
    {
        int property[2];
        fread(&property[0], sizeof(int), 2, fp);
        //std::cout << property[0] << " " << property[1] << std::endl;
        Eigen::Matrix<T,-1,-1> mat(property[0],property[1]);
        fread(mat.data(), sizeof(T), property[0]*property[1], fp);
        
        return mat;
    }
    
    template <class T>
    std::vector< Eigen::Triplet<T> > to_triplets(Eigen::SparseMatrix<T> & M){
        std::vector< Eigen::Triplet<T> > v;
        for(int i = 0; i < M.outerSize(); i++)
            for(typename Eigen::SparseMatrix<T>::InnerIterator it(M,i); it; ++it)
                v.push_back(Eigen::Triplet<T>(it.row(),it.col(),it.value()));
        return v;
    }
}
