//
//  DEM_adaptor.cpp
//  FaceFitting
//
//  Created by SaitoShunsuke on 10/12/15.
//  Copyright © 2015 SaitoShunsuke. All rights reserved.
//

#include "dem_adaptor.h"

namespace cao{

bool DEMAdaptor::AddVector(const Data& data)
{
    const DOF& dof = *data.dof;
    cv::Mat_<float> V(4+dof.EX,1);
    data.cur_x(dof.roiROT()).copyTo(V(cv::Rect(0, 0, 4, 1)));
    data.cur_x(dof.roiEX()).copyTo(V(cv::Rect(4,0,dof.EX,1)));
    // representative frame selection
    if(data_.size() > L_){ // if data list is full, check the projection error
        float error = getPCARcnErr(V);
        if(error > thresh_ && Vset_.rows < L_*2.0){
            // concatenate new data to Vset_
            Vset_.push_back(V);
            data_.pop_back();
            data_.push_front(data);

            return true;
        }
        else{
            return false;
        }
    }
    else if(data_.size() == 0){
        Vset_ = V.clone();
        data_.push_back(data); // NOTE: in the old code, expressions are set to zero (not sure why...)

        return false;
    }
    else{
        Vset_.push_back(V);
        data_.push_back(data); // NOTE: in the old code, expressions are set to zero (not sure why...)

        return false;
    }
}

bool DEMAdaptor::AdaptDEM(Data& data)
{
    // apapt identity and focal length using representative frames
    identityAdaptation(data.id_coeff);

    data.fl = focalAdaptation(data.id_coeff);

    // update shape vectors of representative frames
    updateShapeVectors(data.id_coeff, data.fl);

    // update PCA basis
    if(data_.size() >= L_) updatePCA();
    
    return true;
}

void DEMAdaptor::updatePCA()
{
    cv::Mat_<float> eigen;
    
    cv::PCACompute(Vset_, mean_, eigen);

    int N = eigen.rows*0.95;
    M_ = eigen(cv::Rect(0,0,N,eigen.cols)).clone();
}

float DEMAdaptor::getPCARcnErr(const cv::Mat_<float>& V)
{
    assert(data_.size() > L_);
    cv::Mat_<float> out_proj, Vpca;
    cv::PCAProject(V, mean_, M_, out_proj);
    cv::PCABackProject(out_proj, mean_, M_, Vpca);
    return cv::norm(V-Vpca);
}

void DEMAdaptor::identityAdaptation(cv::Mat_<float>& id)
{
//    dem_id_problem_.Initialize();
//
//    Eigen::VectorXd x0 = identity.cast<double>();
//
//    const Eigen::VectorXf& sigma = face_model_->get_sigma_identity();
//    lbfgsb::Solver solver;
//    solver.setDim(NUM_ID); solver.maxIter = 20;
//    solver.setBoxConstriant((-1.0*sigma).cast<double>(), (1.0*sigma).cast<double>());
//    solver.solve(dem_id_problem_, x0);
//
//    identity = x0.cast<float>();
}

float DEMAdaptor::focalAdaptation(const cv::Mat_<float>& id)
{
//    face_model_->UpdateIdentity(identity);
//
//    float lhs = 0, rhs = 0;
//    const float p0 = rep_data_[0].width()/2, q0 = rep_data_[0].height()/2;
//
//    for(auto&& td: rep_data_)
//    {
//        std::vector<Eigen::Vector2f>& s = td.landmarks_;
//        std::vector<Eigen::Vector3f> landmark_points3d;
//        Eigen::VectorXf& shape_v = td.current_vector_;
//
//        hfm::RigidMotion rigid;
//        rigid.rotation_ = Eigen::Quaternion<float>(shape_v(0),shape_v(1),shape_v(2),shape_v(3)).toRotationMatrix();
//        rigid.translation_ = shape_v.block_T;
//
//        const Eigen::VectorXf& expression = shape_v.block_exp;
//        const std::vector<int>& tracking_indices = face_model_->GetTrackingPointsIndex();
//
//        const std::vector<ContourPoint>& contour_list = td.cp_list_;
//        std::vector<Eigen::Vector3f> points3d;
//        for(auto&& cont : contour_list)
//        {
//            auto p1 = face_model_->get_customized_expression_per_vertex(cont.p1, expression);
//            auto p2 = face_model_->get_customized_expression_per_vertex(cont.p2, expression);
//            points3d.push_back(rigid.ApplyToPoint(cont.a*p1+(1.0-cont.a)*p2));
//        }
//
//        // get updated landmarks inside
//        for(int i : tracking_indices)
//        {
//            auto v = face_model_->get_customized_expression_per_vertex(i, expression);
//            points3d.push_back(rigid.ApplyToPoint(v));
//        }
//
//        int idx_count = 0;
//        for(auto&& p : points3d)
//        {
//            lhs += (p(0)*p(0)+p(1)*p(1))/(p(2)*p(2));
//            rhs += ((p0-s[idx_count](0))*p(0)+(q0-s[idx_count](1))*p(1))/p(2);
//            idx_count++;
//        }
//    }
//    if(-rhs/lhs < 500){
//        std::cout << "DEM has some problem..." << std::endl;
//        return 1000;
//    }
//    else
//        return -rhs/lhs;
    return 0.f;
}

void DEMAdaptor::updateShapeVectors(const cv::Mat_<float>& id, float fl)
{
    // if we do not update identity here, blendshape optimization will give us wrong estimation
    for(auto&& td: data_)
    {
        td.fl = fl;
        td.id_coeff = id.clone();

        // get updated 3D contours
//        RigidAlignment(td, face_model_,false,0,0); // for fast computation, let's not update contour
        
        //SolveBlendShape(face_model_, td);
    }
}

void DEMAdaptor::Init()
{
}

void DEMAdaptor::Reset()
{
    data_.clear();
}
    
};//namespace cao
