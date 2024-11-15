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
#include <Eigen/Dense>

#include "fw_utils.h"
#include "obj_loader.h"

void loadBlendshapeFW( const std::string& filename, Eigen::MatrixXf& shape, float scale)
{
    FILE* fp;
    fp = fopen(filename.c_str(), "rb");
    if(!fp){ std::cout << "Loading " << filename << " Failed." << std::endl; return;}
    int nShapes = 0, nVerts = 0, nFaces = 0;
    fread( &nShapes, sizeof(int), 1, fp );			// nShape = 46
    fread( &nVerts, sizeof(int), 1, fp );			// nVerts = 11510
    fread( &nFaces, sizeof(int), 1, fp );			// nFaces = 11540
    
    shape.resize(nVerts*3,nShapes+1);
    // Load neutral expression B_0
    fread( &shape.col(0)[0], sizeof(float), nVerts*3, fp );
    
    // Load other expressions B_i ( 1 <= i <= 46 )
    for( int exprId=0; exprId<nShapes; exprId++ ){
        fread( &shape.col(exprId+1)[0], sizeof(float), nVerts*3, fp );
        shape.col(exprId+1) -= shape.col(0);
    }
    
    fclose( fp );
    
    shape *= scale;
}

void performSVD(const Eigen::MatrixXf& A,
                Eigen::MatrixXf& B,
                Eigen::VectorXf& stddev,
                int tar_dim)
{
    std::cout << "Get AtA..." << std::endl;
    // get AtA
    Eigen::MatrixXf AAt = A*A.transpose();
    
    // run SVD
    std::cout << "Run SVD..." << std::endl;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(AAt,Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // truncate matrix
    Eigen::MatrixXf Uid = svd.matrixU().leftCols(tar_dim);
    
    // reproject ShapeTensor
    std::cout << "Reproject ShapeTensor" << std::endl;
    B = Uid.transpose() * A;
    
    // compute mean and standard deviation
    Eigen::RowVectorXf mean_coeff = Uid.colwise().mean();
    Eigen::RowVectorXf std_dev = ((Uid.rowwise() - mean_coeff).array().square().colwise().sum() / (Uid.rows() - 1)).sqrt();
    
    Eigen::RowVectorXf ub = mean_coeff + 4.0*std_dev;
    Eigen::RowVectorXf lb = mean_coeff - 4.0*std_dev;
    
    std::cout << "mean" << std::endl;
    std::cout << mean_coeff << std::endl;
    std::cout << "std" << std::endl;
    std::cout << std_dev << std::endl;
    std::cout << "max" << std::endl;
    std::cout << Uid.colwise().maxCoeff() << std::endl;
    std::cout << "min" << std::endl;
    std::cout << Uid.colwise().minCoeff() << std::endl;
    std::cout << "ub coverege" << std::endl;
    std::cout << (ub.array() > Uid.colwise().maxCoeff().array()).count() << "/" << ub.size() << std::endl;
    std::cout << "lb coverage" << std::endl;
    std::cout << (lb.array() < Uid.colwise().minCoeff().array()).count() << "/" << lb.size() << std::endl;
    
    stddev = std_dev.transpose();
}

void computeCoreTensor(const std::vector<Eigen::MatrixXf>& in,
                       std::vector<Eigen::MatrixXf>& out,
                       Eigen::VectorXf& mean,
                       Eigen::MatrixXf& w_exp,
                       Eigen::VectorXf& stddev_id,
                       Eigen::VectorXf& stddev_ex,
                       int tar_id,
                       int tar_ex)
{
    const int n_id = in.size();
    const int n_exp = in[0].cols();
    const int n_v = in[0].rows();
    
    if(tar_ex == -1)
        tar_ex = n_exp;
    
    std::vector<Eigen::MatrixXf> shapes = in;
    mean = Eigen::VectorXf::Zero(shapes[0].rows());
    w_exp = Eigen::MatrixXf::Zero(n_v, n_exp-1);
    for(const Eigen::MatrixXf& sp : shapes)
    {
        mean += sp.col(0);
        
        for(int i = 1; i < n_exp; ++i)
        {
            w_exp.col(i-1) += sp.col(i);
        }
    }
    mean /= (float)shapes.size();
    w_exp /= (float)shapes.size();
    
    for(Eigen::MatrixXf& sp : shapes)
    {
        sp.col(0) -= mean;
        
        for(int i = 1; i < n_exp; ++i)
        {
            sp.col(i) -= w_exp.col(i-1);
        }
    }
    
    Eigen::MatrixXf A(n_id,n_exp*n_v);
    for(int i = 0; i < n_id; ++i)
    {
        for(int j = 0; j < n_exp; ++j)
        {
            for(int k = 0; k < n_v; ++k)
            {
                A(i,n_v*j+k) = shapes[i](k,j);
            }
        }
    }
    
    Eigen::MatrixXf B;
    performSVD(A, B, stddev_id, tar_id);
    
    if(tar_ex > 0 && tar_ex < n_exp){
        A.resize(n_exp, tar_id*n_v);
        for(int i = 0; i < n_exp; ++i)
        {
            for(int j = 0; j < tar_id; ++j)
            {
                for(int k = 0; k < n_v; ++k)
                {
                    A(i,j*n_v+k) = B(j, n_v*i+k);
                }
            }
        }
        
        performSVD(A, B, stddev_ex, tar_ex);
        
        A = B;
        for(int i = 0; i < tar_ex; ++i)
        {
            for(int j = 0; j < tar_id; ++j)
            {
                for(int k = 0; k < n_v; ++k)
                {
                    B(j,i*n_v+k) = A(i, n_v*j+k);
                }
            }
        }
    }
    
    out.resize(tar_ex);
    for(int i = 0; i < tar_ex; ++i)
    {
        out[i].resize(n_v, tar_id);
        for(int j = 0; j < tar_id; ++j)
        {
            for(int k = 0; k < n_v; ++k)
            {
                out[i](k,j) = stddev_id(j)*stddev_ex(i)*B(j, n_v*i+k);
            }
        }
    }
    
    stddev_id.setOnes();
    stddev_ex.setOnes();
}

void computePCA(const std::vector<Eigen::MatrixXf>& in,
                Eigen::MatrixXf& w_id,
                Eigen::MatrixXf& w_ex,
                Eigen::VectorXf& mu_id,
                Eigen::VectorXf& stddev_id,
                Eigen::VectorXf& stddev_ex,
                int tar_id,
                int tar_ex)
{
    const int n_id = in.size();
    const int n_exp = in[0].cols();
    const int n_v = in[0].rows();
    
    if(tar_ex == -1)
        tar_ex = n_exp;
    
    std::vector<Eigen::MatrixXf> shapes = in;
    mu_id = Eigen::VectorXf::Zero(n_v);
    Eigen::VectorXf mu_ex = Eigen::VectorXf::Zero(n_v);
    for(const Eigen::MatrixXf& sp : shapes)
    {
        mu_id += sp.col(0);
        
        for(int i = 1; i < n_exp; ++i)
        {
            mu_ex += sp.col(i);
        }
    }
    mu_id /= (float)shapes.size();
    mu_ex /= (float)shapes.size()*(n_exp-1);
    
    Eigen::MatrixXf wID = Eigen::MatrixXf::Zero(n_id, n_v);
    Eigen::MatrixXf wEX = Eigen::MatrixXf::Zero(n_id*(n_exp-1), n_v);
    
    for(int j = 0; j < n_id; ++j)
    {
        wID.col(j) = shapes[j].col(0)-mu_id;

        for(int i = 1; i < n_exp; ++i)
        {
            wEX.col(j*(n_exp-1)+i-1) = shapes[j].col(i) - mu_ex;
        }
    }
    
    performSVD(wID, w_id, stddev_id, tar_id);
    performSVD(wEX, w_ex, stddev_ex, tar_ex);
}
