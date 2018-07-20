//
//  test_regressor.cpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 10/5/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#include <map>
#include <utility>
#include "model.h"

namespace cao{
    
static void similarityTransform(cv::Mat_<cv::Vec2f>& out, const cv::Mat_<cv::Vec2f>& x, const cv::Mat_<cv::Vec2f>& y, bool enable_translate)
{
    assert(x.size() == y.size());
    int n_p2d = x.rows;
    double X1 = 0, X2 = 0, Y1 = 0, Y2 = 0, Ysqr = 0, W = (double)n_p2d;
    double C1 = 0, C2 = 0;
    
    for(int i = 0; i < x.rows; ++i)
    {
        X1 += x(i)(0);
        X2 += y(i)(0);
        Y1 += x(i)(1);
        Y2 += y(i)(1);
        Ysqr += y(i)(0)*y(i)(0) + y(i)(1)*y(i)(1);
        C1 += (x(i)(0)*y(i)(0)+x(i)(1)*y(i)(1));
        C2 += (x(i)(1)*y(i)(0)-x(i)(0)*y(i)(1));
    }
    
    cv::Matx44d A(X2, -Y2, W, 0,
                  Y2, X2, 0, W,
                  Ysqr, 0, X2, Y2,
                  0, Ysqr, -Y2, X2);
    cv::Matx41d b(X1, Y1, C1, C2);
    cv::Matx41d solution = A.inv() * b;
    
    cv::Matx22f R;
    cv::Matx21f t;
    R(0, 0) = (float)solution(0);
    R(0, 1) = (float)-solution(1);
    R(1, 0) = (float)solution(1);
    R(1, 1) = (float)solution(0);
    t(0) = (float)solution(2);
    t(1) = (float)solution(3);
    
    out = y.clone();
    for(int i = 0; i < out.rows; ++i)
    {
        out(i) = R*out(i);
        
        if(enable_translate) out(i) += t;
    }
}

static void normalizeWithRot(cv::Mat_<cv::Vec2f>& out,
                             const cv::Mat_<cv::Vec2f>& in,
                             int l_idx,
                             int r_idx)
{
    cv::Scalar mean = cv::mean(in);
    cv::subtract(in, mean, out);
    
    cv::Vec2f l_eye = in(l_idx);
    cv::Vec2f r_eye = in(r_idx);
    float eyes_distance = cv::norm(l_eye - r_eye);
    float scale = 1.0 / eyes_distance;
    
    float x_diff = r_eye(0) - l_eye(0);
    x_diff = x_diff > 1.0e-8 ? x_diff : (x_diff > 0) ? 1.0e-8 : -1.0e-8;
    float theta = -atan((r_eye(1) - l_eye(1)) / x_diff );
    
    // Must do translation first, and then rotation.
    // Therefore, translation is done separately
    cv::Matx22f T;
    T(0, 0) = scale * cos(theta);
    T(0, 1) = -scale * sin(theta);
    T(1, 0) = scale * sin(theta);
    T(1, 1) = scale * cos(theta);
    
    for(int i = 0; i < out.rows; ++i)
    {
        out(i) = T*out(i);
    }
}

static void calcMeanShape(cv::Mat_<cv::Vec2f>& out,
                          const std::vector<cv::Mat_<cv::Vec2f>>& shapes,
                          int l_idx,
                          int r_idx)
{
    out = cv::Mat_<cv::Vec2f>::zeros(shapes[0].size());
    
    for (auto&& shape: shapes)
    {
        cv::Mat_<cv::Vec2f> new_shape;
        normalizeWithRot(new_shape, shape, l_idx, r_idx);
        out += new_shape;
    }
    
    out *= 1.0 / (float)shapes.size();
    
    normalizeWithRot(out, out, l_idx, r_idx);
}
    
static void normalize(cv::Mat_<cv::Vec2f>& out,
                      const cv::Mat_<cv::Vec2f>& in,
                      int l_idx,
                      int r_idx)
{
    cv::Scalar mean = cv::mean(in);
    cv::subtract(in, mean, out);
    
    cv::Vec2f l_eye = in(l_idx);
    cv::Vec2f r_eye = in(r_idx);
    float eyes_distance = cv::norm(l_eye - r_eye);
    
    out *= 1.0 / eyes_distance;
}

bool Model::test(cv::Mat_<float>& X,
                 const cv::Mat_<uchar>& img,
                 const cv::Mat_<cv::Vec2f>& p2d,
                 const cv::Mat_<int>& tri,
                 const DOF& dof)
{
    cv::Mat_<float> cur_RT = X(dof.roiRT()).clone();
    cv::Mat_<float> dist;

    // TODO: p2d to aligned_p2d
    cv::Mat_<cv::Vec2f> aligned_p2d;
    int l_eye, r_eye;
    normalize(aligned_p2d, p2d, l_eye, r_eye);
    
    // Warning: this is approximate kNN! so the randomness may produce inconsistent output
    kdTree_.knnSearch(aligned_p2d, knn_, dist, k_);

    // apply regression to k-NN
    X.setTo(0.0);
    for(int k : knn_)
    {
        cv::Mat_<float> Xk = gt_x_.row(k).clone();
       
        Xk(dof.roiRT()) = cur_RT.clone(); // Note: for normalization test
        for(int i = 0; i < T_; ++i)
        {
            // TODO: compute 2d projection
            cv::Mat_<cv::Vec2f> p2d_k;
            regressors_[i].apply(Xk, p2d_k, img, tri);
           
            // post process (normalize quartanion so that it remains in SO(3).)
            Xk(dof.roiROT()) *= 1.0/cv::norm(Xk(dof.roiROT()));
        }
        X += Xk;
    }
    
    // taking average
    X *= (1.0/(float)k_);

    // post process (normalize quartanion so that it remains in SO(3).)
    X(dof.roiROT()) *= 1.0/cv::norm(X(dof.roiROT()));

    return true;
}

bool Model::test(cv::Mat_<float>& X,
                 const cv::Mat_<uchar>& img,
                 const cv::Mat_<cv::Vec2f>& p2d,
                 const cv::Mat_<bool> pmap,
                 const cv::Rect &rect,
                 const cv::Mat_<int>& tri,
                 const DOF& dof)
{
    cv::Mat_<float> cur_RT = X(dof.roiRT()).clone();
    cv::Mat_<float> dist;

    // TODO: p2d to aligned_p2d
    cv::Mat_<cv::Vec2f> aligned_p2d;
    int l_eye, r_eye;
    normalize(aligned_p2d, p2d, l_eye, r_eye);

    // Warning: this is approximate kNN! so the randomness may produce inconsistent output
    kdTree_.knnSearch(aligned_p2d, knn_, dist, k_);
    
    // apply regression to k-NN
    X.setTo(0.0);
    for(int k : knn_)
    {
        cv::Mat_<float> Xk = gt_x_.row(k).clone();
        
        Xk(dof.roiRT()) = cur_RT.clone(); // Note: for normalization test
        for(int i = 0; i < T_; ++i)
        {
            // TODO: compute 2d projection
            cv::Mat_<float> p2d_k;
            
            regressors_[i].apply(Xk, p2d_k, img, tri, pmap, rect);
            
            // post process (normalize quartanion so that it remains in SO(3).)
            Xk(dof.roiROT()) *= 1.0/cv::norm(Xk(dof.roiROT()));
        }
        X += Xk;
    }
    
    // taking average
    X *= (1.0/(float)k_);
    
    // post process (normalize quartanion so that it remains in SO(3).)
    X(dof.roiROT()) *= 1.0/cv::norm(X(dof.roiROT()));

    return true;
}

bool Model::train(std::string file_path,
                  std::vector<Data>& data,
                  const TrainParams& params)
{
    const DOF& dof = params.dof;
    const int n_data = (int)data.size();
    const int n_p2d = data[0].gt_p2d.rows;
    const int len_x = data[0].gt_x.cols;
    
    std::vector<cv::Mat_<cv::Vec2f>> p2d;
    gt_x_ = cv::Mat_<float>::zeros(n_data, len_x);
    aligned_p2d_ = cv::Mat_<float>::zeros(n_data, n_p2d*2);
    
    for(int i = 0; i < data.size(); ++i)
    {
        p2d.push_back(data[i].gt_p2d);
        gt_x_.row(i) = data[i].gt_x;
        
        cv::Mat_<cv::Vec2f> p2d_nml;
        normalize(p2d_nml, data[i].gt_p2d, params.l_eye_idx, params.r_eye_idx);
        aligned_p2d_.row(i) = cv::Mat_<float>(1,n_p2d,(float*)p2d_nml.data).clone();
    }
    std::cout << "Number of traning data: " << data.size() << std::endl;
    
    cv::Mat_<cv::Vec2f> mean_p2d;
    calcMeanShape(mean_p2d, p2d, params.l_eye_idx, params.r_eye_idx);
    
    std::cout << "Augmenting training data..." << std::endl;
    std::vector<Data> augmented_data;
    augmentData(augmented_data, data, params); // augment test data
    
    std::cout << "Number of augmented training data: " << augmented_data.size() << std::endl;
    
    // update regressor
    regressors_.resize(params.T);
    for(int i = 0; i < regressors_.size(); ++i)
    {
        std::cout << "1st Layer " << i << "th Stage Computing..." << std::endl;
        // TODO: update 2d projection

        regressors_[i].setStage(i);
        
        // NOTE: another trick I found is rescaling residual in every stage of regression actually does harm the performance.
        if(i != 0) regressors_[i].setMeanStddev(regressors_[0].means_, regressors_[0].SDs_);
        regressors_[i].train(augmented_data, mean_p2d, params.tri);
        
        // normalize rotation for all data
        for(auto&& d : augmented_data)
        {
            d.cur_x(dof.roiROT()) *= 1.0/cv::norm(d.cur_x(dof.roiROT()));
        }
    }
    
    // print out the final residual
    float sum_res_err = 0.0;
    float sum_p2d_err = 0.0;
    for(auto&& d : augmented_data)
    {
        sum_res_err += cv::norm(d.gt_x-d.cur_x);
        // TODO: update 2d projection
        for(int j = 0; j < d.cur_p2d.rows; ++j)
        {
            sum_p2d_err += cv::norm(d.cur_p2d(j)-d.gt_p2d(j));
        }
    }
    std::cout << "1st Layer " << params.T -1 << "th Stage Done... Regression Error: " << sum_res_err/(float)augmented_data.size();
    std::cout << " 2D error: " << sum_p2d_err/(float)(augmented_data.size()*n_p2d) << std::endl;
   
    writeCVModel(file_path + "tar.gz");
    writeBinary(file_path + ".bin");

    return true;
}
    
    
void Model::loadCVModel(std::string file_path)
{
    cv::FileStorage model_file;
    model_file.open(file_path, cv::FileStorage::READ);
    if (!model_file.isOpened())
        throw std::runtime_error("Cannot open model file \"" + file_path + "\".");
    
    model_file["gt"] >> gt_x_;
    model_file["aligned_p2d"] >> aligned_p2d_;
    cv::FileNode fn = model_file["training_set"];
    
    // for single regressor
    fn = model_file["regressors"];
    int stage = 0;
    regressors_.clear();
    for (auto it = fn.begin(); it != fn.end(); ++it)
    {
       Regressor r;
       *it >> r;
       r.setStage(stage++);
       regressors_.push_back(r);
    }
    T_ = (int)regressors_.size();
    std::cout << "Regressor Loaded (T: " << T_ << ", K: " << regressors_[0].K_ << ", P: " << regressors_[0].P_ << ")" << std::endl;
    
    kdTree_.build(aligned_p2d_,indexParams_);
}
    
void Model::writeCVModel(std::string file_path)
{
    cv::FileStorage model_file;
    model_file.open(file_path, cv::FileStorage::WRITE);
    model_file << "gt" << gt_x_;
    model_file << "aligned_p2d" << aligned_p2d_;

    model_file << "regressors" << "[";
    for (auto it = regressors_.begin(); it != regressors_.end(); ++it)
        model_file << *it;
    model_file << "]";
    
    model_file.release();
}
    
void Model::writeCVModel(std::string file_path, const TrainParams& params)
{
    cv::FileStorage model_file;
    model_file.open(file_path, cv::FileStorage::WRITE);
    model_file << "gt" << gt_x_;
    model_file << "aligned_p2d" << aligned_p2d_;
    
    model_file << "regressors" << "[";
    for (auto it = regressors_.begin(); it != regressors_.end(); ++it)
        model_file << *it;
    model_file << "]";
    
    model_file << "training_param" << "[";
    model_file << "{";
    model_file << "T" << params.T << "K" << params.K << "P" << params.P << "F" << params.F << "beta" << params.beta;
    model_file << "augsize_exp" << params.n_aug_exp << "augsize_other" << params.n_aug_other;
    model_file << "delta_rotation" << params.dR << "delta_xy" << params.dTxy << "delta_z" << params.dTz << "delta_fl" << params.dFl;
    model_file << "}";
    for(int i = 0; i < params.kappas.size(); ++i)
    {
        model_file << "{" << "kappa" << params.kappas[i] << "}";
    }
    model_file << "]";
    
    model_file.release();
}
    
    
void Model::loadBinary(std::string file_path)
{
    std::cout << "load binary model from " << file_path << std::endl;
    
    FILE* fp;
    fp = fopen(file_path.c_str(), "rb");
    
    int size_list[7];
    fread(&size_list[0], sizeof(int), 7, fp);
    const int bin_size = pow(2,size_list[6]);
    
    // load aligned landmarks
    aligned_p2d_ = cv::Mat_<float>(size_list[0],size_list[2]*2);
    fread(aligned_p2d_.ptr(), sizeof(float), aligned_p2d_.total(), fp);
    
    kdTree_.build(aligned_p2d_,indexParams_);
    
    // load set of ground truth shape vectors
    gt_x_ = cv::Mat_<float>::zeros(size_list[0], size_list[1]);
    fread(gt_x_.ptr(), sizeof(float), gt_x_.total(), fp);
    
    // load values of regressors
    regressors_.resize(size_list[3]);
    T_ = (int)regressors_.size();
    for(auto&& reg : regressors_)
    {
        reg.K_ = size_list[4];
        reg.P_ = size_list[5];
        
        // load pixel sampling location
        std::vector<int> tri_indices(size_list[5]);
        std::vector<float> interp_values(size_list[5]*2);
        fread(&tri_indices[0], sizeof(int), size_list[5], fp);
        fread(&interp_values[0], sizeof(float), size_list[5]*2, fp);
        reg.points_.clear();
        for(int i = 0; i < size_list[5]; ++i)
        {
            reg.points_.push_back(std::pair<int, cv::Vec2f>(tri_indices[i],cv::Vec2f(interp_values[i*2+0],interp_values[i*2+1])));
        }
        
        // load statistics for projection
        reg.means_ = cv::Mat_<float>::zeros(1, size_list[1]);
        reg.SDs_ = cv::Mat_<float>::zeros(1, size_list[1]);
        fread(reg.means_.ptr(), sizeof(float), reg.means_.total(), fp);
        fread(reg.SDs_.ptr(), sizeof(float), reg.SDs_.total(), fp);
        
        // load ferns
        std::vector<float> thresholds(size_list[4]*size_list[6]); // K_*F_
        std::vector<int> shape_index(2*size_list[4]*size_list[6]); // 2*K_*F_
        
        fread(&thresholds[0], sizeof(float), thresholds.size(), fp);
        fread(&shape_index[0], sizeof(int), shape_index.size(), fp);
        
        reg.ferns_.assign(size_list[4], Fern());
        for(int k = 0; k < size_list[4]; ++k)
        {
            reg.ferns_[k].thresholds_.clear();
            reg.ferns_[k].shape_index_.clear();
            
            for(int f = 0; f < size_list[6]; ++f)
            {
                reg.ferns_[k].thresholds_.push_back(thresholds[k*size_list[6]+f]);
                reg.ferns_[k].shape_index_.push_back(std::pair<int, int>(shape_index[k*(size_list[6]*2)+f*2+0],shape_index[k*(size_list[6]*2)+f*2+1]));
            }
            
            reg.ferns_[k].res_ = cv::Mat_<float>::zeros(bin_size, size_list[1]);
            fread(reg.ferns_[k].res_.ptr(), sizeof(float), reg.ferns_[k].res_.total(), fp);
            
            reg.ferns_[k].F_ = size_list[6];
        }
    }
    
    fclose(fp);
}

void Model::writeBinary(std::string file_path)
{
    std::cout << "convert the model into binary: " << file_path << std::endl;
    FILE* fp;
    fp = fopen(file_path.c_str(), "wb");
    
    int size_list[7];
    size_list[0] = gt_x_.rows; // training data size
    size_list[1] = gt_x_.cols; // shape vector size
    size_list[2] = aligned_p2d_.cols; // landmark size
    size_list[3] = T_; // regressor size
    size_list[4] = regressors_[0].K_; // premitive regressor size
    size_list[5] = regressors_[0].P_; // sampling pixel size
    size_list[6] = regressors_[0].ferns_[0].F_; // bin size
    fwrite(&size_list[0], sizeof(int), 7, fp);
    
    // save aligned landmark points
    fwrite(aligned_p2d_.ptr(), sizeof(float), aligned_p2d_.total(), fp);
    
    // save shape vectors
    fwrite(gt_x_.ptr(), sizeof(float), gt_x_.total(), fp);
    
    // save values of regressors
    for(auto&& reg : regressors_)
    {
        // save pixel sampling location
        std::vector<int> tri_indices;
        std::vector<float> interp_values;
        for(auto&& p : reg.points_)
        {
            tri_indices.push_back(p.first);
            interp_values.push_back(p.second[0]);
            interp_values.push_back(p.second[1]);
        }
        fwrite(&tri_indices[0], sizeof(int), size_list[5], fp);
        fwrite(&interp_values[0], sizeof(float), size_list[5]*2, fp);
        
        // save statistics for projection
        fwrite(reg.means_.ptr(), sizeof(float), size_list[1], fp);
        fwrite(reg.SDs_.ptr(), sizeof(float), size_list[1], fp);
        
        // save ferns
        std::vector<float> thresholds; // K_*F_
        std::vector<int> shape_index; // 2*K_*F_
        
        for(auto&& f : reg.ferns_)
        {
            for(int i = 0; i < size_list[6]; ++i)
            {
                thresholds.push_back(f.thresholds_[i]);
                shape_index.push_back(f.shape_index_[i].first);
                shape_index.push_back(f.shape_index_[i].second);
            }
        }
        
        fwrite(&thresholds[0], sizeof(float), thresholds.size(), fp);
        fwrite(&shape_index[0], sizeof(int), shape_index.size(), fp);

        for(auto&& f : reg.ferns_)
        {
            fwrite(f.res_.ptr(), sizeof(float), f.res_.total(), fp);
        }
    }
    
    fclose(fp);
}
    
}
