//
//  train_util.cpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 9/18/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "train_util.hpp"

namespace cao{
    
void augmentRotation(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, float dR, const DOF& dof)
{
    Data data = in[idx];
    
    // rotation random generation
    for(int i = 0; i < size; ++i)
    {
        int idx_d = cv::theRNG().uniform(0, in.size());
        if(idx_d == idx){
            i--; continue;
        }
        cv::Mat_<float> x = data.gt_x.clone();
        
        // update random displacement vector
        in[idx_d].gt_x(dof.roiDISP()).copyTo(x(dof.roiDISP()));
        
        // generate random rotation
        cv::Mat_<float> quat = in[idx].gt_x(dof.roiROT()).clone();
        quat(0) += cv::theRNG().uniform(-dR, dR);
        quat(1) += cv::theRNG().uniform(-dR, dR);
        quat(2) += cv::theRNG().uniform(-dR, dR);
        quat(3) += cv::theRNG().uniform(-dR, dR);
        
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

}

void augmentTranslation(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, float dTxy, float dTz, const DOF& dof)
{
    Data data = in[idx];
    
    // translation random generation
    for(int i = 0; i < size; ++i)
    {
        int idx_d = cv::theRNG().uniform(0, in.size());
        if(idx_d == idx){
            i--; continue;
        }
        cv::Mat_<float> x = data.gt_x.clone();
        
        // update random displacement vector
        // NOTE: it may need modification
        
        in[idx_d].gt_x(dof.roiDISP()).copyTo(x(dof.roiDISP()));
        
        // generate random translation
        cv::Mat_<float> trans = in[idx].gt_x(dof.roiTR()).clone();
        trans(0) += cv::theRNG().uniform(-dTxy, dTxy);
        trans(1) += cv::theRNG().uniform(-dTxy, dTxy);
        trans(2) += cv::theRNG().uniform(-dTz, dTz);
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
}

void augmentExpression(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, const DOF& dof)
{
    Data data = in[idx];

    // expression random generation
    for(int i = 0; i < size; ++i)
    {
        int idx_d = cv::theRNG().uniform(0, in.size());
        int idx_e = cv::theRNG().uniform(0, in.size());
        if(idx_d == idx || idx_e == idx){
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
}

void augmentFocalLength(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, float dF, const DOF& dof)
{
    Data data = in[idx];
    
    // focal length
    for(int i = 0; i < size; ++i)
    {
        int idx_d = cv::theRNG().uniform(0, in.size());
        if(idx_d == idx){
            i--; continue;
        }
        // apply random focal length
        data.fl += cv::theRNG().uniform(-dF, dF);
        
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

void augmentIdentity(std::vector<Data>& out, const std::vector<Data>& in, int idx, int size, const DOF& dof)
{
    Data data = in[idx];

    // identity random generation
    for(int i = 0; i < size; ++i)
    {
        int idx_d = cv::theRNG().uniform(0, in.size());
        int idx_i = cv::theRNG().uniform(0, in.size());
        if(idx_d == idx || idx_i == idx){
            i--; continue;
        }
        // replace the current identity with random one (do not forget to update the ground truth accordingly.)
        data.idCoeff = in[idx_i].idCoeff;
        float total = 0.0;
        for(int k = 0; k < data.idCoeff.size(); ++k)
        {
            total += (data.idCoeff[k]-in[idx_i].idCoeff[k])*(data.idCoeff[k]-in[idx_i].idCoeff[k]);
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

}

void augmentOcclusion(std::vector<Data>& in)
{
    for(auto&& td : in)
    {
        cv::Mat mask = td.mask;
        
        cv::Point p1;
        while(1){
            int x = cv::theRNG().uniform(0,mask.cols);
            int y = cv::theRNG().uniform(0,mask.rows);
            if(mask.at<uchar>(y,x) == 255){
                p1 = cv::Point(x,y);
                break;
            }
        }
        int w = mask.cols*cv::theRNG().uniform(0.05, 0.3);
        int h = mask.rows*cv::theRNG().uniform(0.05, 0.3);
        
        cv::Point p2 = p1 + cv::Point(w,h);
        p1 = p1 - cv::Point(w,h);
        
        cv::rectangle(mask,p1,p2,cv::Scalar(0),-1);
    }
}

void augmentData(std::vector<Data>& out, const std::vector<Data>& in, const TrainParams& params)
{
    const DOF& dof = params.dof;
    out.clear();
    
    //int count_rot = 0, count_tran = 0, count_bs = 0, count_id = 0;
    
    for(int data_idx = 0; data_idx < in.size(); ++data_idx)
    {
        Data data = in[data_idx];
        
        augmentRotation(out, in, data_idx, params.n_aug_other, params.dR, dof);
        augmentTranslation(out, in, data_idx, params.n_aug_other, params.dTxy, params.dTz, dof);
        augmentExpression(out, in, data_idx, params.n_aug_exp, dof);
        augmentIdentity(out, in, data_idx, params.n_aug_other, dof);
        augmentFocalLength(out, in, data_idx, params.n_aug_other, params.dFl, dof);
    }
    
    // adding occlusion augmentation
    if(params.with_oh)
        augmentOcclusion(out);
}
    
}
