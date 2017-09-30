//
//  train_util.cpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 9/18/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "train_util.hpp"

namespace cao{

void augmentData(std::vector<Data>& out, const std::vector<Data>& in, const TrainParams& params)
{
    const DOF& dof = params.dof;
    out.clear();
    
    //int count_rot = 0, count_tran = 0, count_bs = 0, count_id = 0;
    
    for(int data_idx = 0; data_idx < in.size(); ++data_idx)
    {
        // for displacement, should probably rescale the displacement based on the bounding box size
        Data data = in[data_idx];
        Eigen::VectorXf id_ori = data.idCoeff; // for random identity
        //face_model_->UpdateIdentity(data.identity_);
        
        // rotation random generation
        for(int i = 0; i < params.n_aug_other; ++i)
        {
            int idx_d = cv::theRNG().uniform(0, in.size());
            if(idx_d == data_idx){
                i--; continue;
            }
            cv::Mat_<float> x = data.gt_x.clone();
            
            // update random displacement vector
            in[idx_d].gt_x(dof.roiDISP()).copyTo(x(dof.roiDISP()));
            
            // generate random rotation
            cv::Mat_<float> quat = in[data_idx].gt_x(dof.roiROT()).clone();
            quat(0) += cv::theRNG().uniform(-params.dR, params.dR);
            quat(1) += cv::theRNG().uniform(-params.dR, params.dR);
            quat(2) += cv::theRNG().uniform(-params.dR, params.dR);
            quat(3) += cv::theRNG().uniform(-params.dR, params.dR);
            
            // normalize the quartanion
            quat *= 1.0/cv::norm(quat);
            quat.copyTo(x(dof.roiROT()));
            
            data.cur_x = x.clone();
            
#ifdef DEBUG_AUGMENTATION
            cv::Mat_<cv::Vec2f> p2d;
            // TODO: update 2d projection
            cv::Mat_<cv::Vec3b> debug_img;
            cv::cvtColor(data.img, debug_img, CV_GRAY2BGR);
            for(int i = 0; i < p2d.rows; ++i)
            {
                cv::circle(debug_img, cv::Point(p2d(i)(0)-data.rect.x,p2d(i)(1)-data.rect.y), 2.0, cv::Scalar(255,0,0), -1);
                cv::circle(debug_img, cv::Point(data.gt_p2d(i)(0)-data.rect.x,data.gt_p2d(i)(1)-data.rect.y), 1.0, cv::Scalar(0,255,0), -1);
            }
            cv::imshow("Augmented Data: Rotation", debug_img);
            cv::waitKey(1);
#endif
            
            out.push_back(data);
        }
        
        // translation random generation
        for(int i = 0; i < params.n_aug_other; ++i)
        {
            int idx_d = cv::theRNG().uniform(0, in.size());
            if(idx_d == data_idx){
                i--; continue;
            }
            cv::Mat_<float> x = data.gt_x.clone();
            
            // update random displacement vector
            // NOTE: it may need modification
            
            in[idx_d].gt_x(dof.roiDISP()).copyTo(x(dof.roiDISP()));
            
            // generate random translation
            cv::Mat_<float> trans = in[data_idx].gt_x(dof.roiTR()).clone();
            trans(0) += cv::theRNG().uniform(-params.dTxy, params.dTxy);
            trans(1) += cv::theRNG().uniform(-params.dTxy, params.dTxy);
            trans(2) += cv::theRNG().uniform(-params.dTz, params.dTz);
            trans.copyTo(x(dof.roiDISP()));
            
            data.cur_x = x.clone();
            
#ifdef DEBUG_AUGMENTATION
            cv::Mat_<cv::Vec2f> p2d;
            // TODO: update 2d projection
            cv::Mat_<cv::Vec3b> debug_img;
            cv::cvtColor(data.img, debug_img, CV_GRAY2BGR);
            for(int i = 0; i < p2d.rows; ++i)
            {
                cv::circle(debug_img, cv::Point(p2d(i)(0)-data.rect.x,p2d(i)(1)-data.rect.y), 2.0, cv::Scalar(255,0,0), -1);
                cv::circle(debug_img, cv::Point(data.gt_p2d(i)(0)-data.rect.x,data.gt_p2d(i)(1)-data.rect.y), 1.0, cv::Scalar(0,255,0), -1);
            }
            cv::imshow("Augmented Data: Translation", debug_img);
            cv::waitKey(1);
#endif
            
           out.push_back(data);
        }
        
        // expression random generation
        for(int i = 0; i < params.n_aug_exp; ++i)
        {
            int idx_d = cv::theRNG().uniform(0, in.size());
            int idx_e = cv::theRNG().uniform(0, in.size());
            if(idx_d == data_idx || idx_e == data_idx){
                i--; continue;
            }
            cv::Mat_<float> x = data.gt_x.clone();
            
            // update random displacement vector
            in[idx_d].gt_x(dof.roiDISP()).copyTo(x(dof.roiDISP()));
            
            // replace the current bs coefficient with random one
            in[idx_e].gt_x(dof.roiEX()).copyTo(x(dof.roiEX()));
            
            data.cur_x = x.clone();
            
#ifdef DEBUG_AUGMENTATION
            cv::Mat_<cv::Vec2f> p2d;
            // TODO: update 2d projection
            cv::Mat_<cv::Vec3b> debug_img;
            cv::cvtColor(data.img, debug_img, CV_GRAY2BGR);
            for(int i = 0; i < p2d.rows; ++i)
            {
                cv::circle(debug_img, cv::Point(p2d(i)(0)-data.rect.x,p2d(i)(1)-data.rect.y), 2.0, cv::Scalar(255,0,0), -1);
                cv::circle(debug_img, cv::Point(data.gt_p2d(i)(0)-data.rect.x,data.gt_p2d(i)(1)-data.rect.y), 1.0, cv::Scalar(0,255,0), -1);
            }
            cv::imshow("Augmented Data: Blend Shape", debug_img);
            cv::waitKey(1);
#endif
            
            out.push_back(data);
        }
        
        // identity random generation
        for(int i = 0; i < params.n_aug_other; ++i)
        {
            int idx_d = cv::theRNG().uniform(0, in.size());
            int idx_i = cv::theRNG().uniform(0, in.size());
            if(idx_d == data_idx || idx_i == data_idx){
                i--; continue;
            }
            // replace the current identity with random one (do not forget to update the ground truth accordingly.)
            data.idCoeff = in[idx_i].idCoeff;
            float total = 0.0;
            for(int k = 0; k < id_ori.size(); ++k)
            {
                total += (data.idCoeff[k]-id_ori[k])*(data.idCoeff[k]-id_ori[k]);
            }
            if(total < 0.1){
                i--; continue;
            }
            
            // TODO: update 2d projection + displacement for ground truth
            
            data.gt_x = data.cur_x.clone();
            
            cv::Mat_<float> x = data.gt_x.clone();
            
            // update random displacement vector
            in[idx_d].gt_x(dof.roiDISP()).copyTo(x(dof.roiDISP()));
            
            data.cur_x = x.clone();
            
#ifdef DEBUG_AUGMENTATION
            cv::Mat_<cv::Vec2f> p2d;
            // TODO: update 2d projection
            cv::Mat_<cv::Vec3b> debug_img;
            cv::cvtColor(data.img, debug_img, CV_GRAY2BGR);
            for(int i = 0; i < p2d.rows; ++i)
            {
                cv::circle(debug_img, cv::Point(p2d(i)(0)-data.rect.x,p2d(i)(1)-data.rect.y), 2.0, cv::Scalar(255,0,0), -1);
                cv::circle(debug_img, cv::Point(data.gt_p2d(i)(0)-data.rect.x,data.gt_p2d(i)(1)-data.rect.y), 1.0, cv::Scalar(0,255,0), -1);
            }
            cv::imshow("Augmented Data: Identity", debug_img);
            cv::waitKey(1);
#endif
            out.push_back(data);
        }
        
        // focal length
        for(int i = 0; i < params.n_aug_other; ++i)
        {
            int idx_d = cv::theRNG().uniform(0, in.size());
            if(idx_d == data_idx){
                i--; continue;
            }
            // apply random focal length
            data.fl += cv::theRNG().uniform(-params.dFl, params.dFl);
            
            // TODO: update rigid head pose + displacement for ground truth
            //RigidAlignment(data, face_model_,false); // no contour update
            //UpdateDisplacement(data, face_model_); // Caution: this part cannot be parallelizable
            
            data.gt_x = data.cur_x.clone();
            
            cv::Mat_<float> x = data.gt_x.clone();
            
            // update random displacement vector
            in[idx_d].gt_x(dof.roiDISP()).copyTo(x(dof.roiDISP()));
            
            data.cur_x = x.clone();
            
#ifdef DEBUG_AUGMENTATION
            cv::Mat_<cv::Vec2f> p2d;
            // TODO: update 2d projection
            cv::Mat_<cv::Vec3b> debug_img;
            cv::cvtColor(data.img, debug_img, CV_GRAY2BGR);
            for(int i = 0; i < p2d.rows; ++i)
            {
                cv::circle(debug_img, cv::Point(p2d(i)(0)-data.rect.x,p2d(i)(1)-data.rect.y), 2.0, cv::Scalar(255,0,0), -1);
                cv::circle(debug_img, cv::Point(data.gt_p2d(i)(0)-data.rect.x,data.gt_p2d(i)(1)-data.rect.y), 1.0, cv::Scalar(0,255,0), -1);
            }
            cv::imshow("Augmented Data: Focal Length", debug_img);
            cv::waitKey(1);
#endif
            out.push_back(data);
        }
    }
}
    
}
