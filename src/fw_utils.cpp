//
//  fw_utils.cpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 10/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "fw_utils.hpp"

#include <Eigen/Dense>

#include "obj_loader.hpp"

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

void computeCoreTensor(const std::vector<Eigen::MatrixXf>& in,
                       std::vector<Eigen::MatrixXf>& out,
                       Eigen::VectorXf& mean,
                       Eigen::MatrixXf& w_exp,
                       Eigen::VectorXf& stddev,
                       int tar_id)
{
    const int n_id = in.size();
    const int n_exp = in[0].cols();
    const int n_v = in[0].rows();
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
    
    std::cout << "Get AtA..." << std::endl;
    // get AtA
    Eigen::MatrixXf AAt = A*A.transpose();
    
    // run SVD
    std::cout << "Run SVD..." << std::endl;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(AAt,Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // truncate matrix
    Eigen::MatrixXf U = svd.matrixU().leftCols(tar_id);
    
    // reproject ShapeTensor
    std::cout << "Reproject ShapeTensor" << std::endl;
    Eigen::MatrixXf B = U.transpose() * A;
    
    // compute mean and standard deviation
    Eigen::RowVectorXf mean_coeff = U.colwise().mean();
    Eigen::RowVectorXf std_dev = ((U.rowwise() - mean_coeff).array().square().colwise().sum() / (U.rows() - 1)).sqrt();

    Eigen::RowVectorXf ub = mean_coeff + 4.0*std_dev;
    Eigen::RowVectorXf lb = mean_coeff - 4.0*std_dev;

    std::cout << "mean" << std::endl;
    std::cout << mean_coeff << std::endl;
    std::cout << "std" << std::endl;
    std::cout << std_dev << std::endl;
    std::cout << "max" << std::endl;
    std::cout << U.colwise().maxCoeff() << std::endl;
    std::cout << "min" << std::endl;
    std::cout << U.colwise().minCoeff() << std::endl;
    std::cout << "ub coverege" << std::endl;
    std::cout << (ub.array() > U.colwise().maxCoeff().array()).count() << "/" << ub.size() << std::endl;
    std::cout << "lb coverage" << std::endl;
    std::cout << (lb.array() < U.colwise().minCoeff().array()).count() << "/" << lb.size() << std::endl;
    
    out.resize(n_exp);
    for(int i = 0; i < n_exp; ++i)
    {
        out[i].resize(n_v, tar_id);
        for(int j = 0; j < tar_id; ++j)
        {
            for(int k = 0; k < n_v; ++k)
            {
                out[i](k,j) = std_dev(j)*B(j, n_v*i+k);
            }
        }
    }
    std_dev.setOnes();
    
    stddev = std_dev.transpose();
}
