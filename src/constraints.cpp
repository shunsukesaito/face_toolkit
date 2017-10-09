#include "constraints.h"

static Eigen::Vector2f calcLineSegment(const Eigen::Vector2f& p,
                                       const std::vector<Eigen::Vector3f>& qV,
                                       Eigen::Vector3f& q,
                                       int start_idx,
                                       int end_idx)
{
    assert(start_idx < end_idx);
    
    Eigen::Vector2f pq;
    Eigen::Vector2f n_min;
    Eigen::Vector2f l, n;
    float dist, ratio;
    float mindist = 1.e5;
    for(int i = start_idx; i < end_idx; ++i)
    {
        l = qV[i + 1].segment<2>(0)-qV[i].segment<2>(0);
        n << l[1], -l[0]; n.normalize();
        pq = p - qV[i].segment<2>(0);
        
        ratio = l.dot(pq)/l.squaredNorm();
        if(ratio < 0 || ratio > 1.0)
            continue;
        
        dist = n.dot(pq);
        
        if(dist < mindist){
            n_min = n;
            q = ratio*qV[i + 1]+(1.0-ratio)*qV[i];
            
            mindist = dist;
        }
    }
    
    // if p doesn't lie in the line segments
    if(mindist == 1.e5){
        l = (qV[start_idx + 1].segment<2>(0)-qV[start_idx].segment<2>(0));
        n << l[1], -l[0]; n.normalize();
    
        ratio = l.dot(pq);
        if(ratio < 0){
            q = qV[start_idx];
            return n;
        }
        else{
            q = qV[end_idx];
            l = (qV[end_idx].segment<2>(0)-qV[end_idx - 1].segment<2>(0));
            n << l[1], -l[0]; n.normalize();
            
            return n;
        }
    }
    
    return n_min;
}

void P2P2DC::getIndexList(const std::vector<P2P2DC> &C, std::vector<int> &idxs)
{
    idxs.clear();
    
    for(auto& c : C)
    {
        idxs.push_back(c.v_idx);
    }
}

void P2P2DC::updateConstraints(const std::vector<P2P2DC>& C,
                               const std::vector<Eigen::Vector3f>& qinV,
                               std::vector<Eigen::Vector3f>& qoutV)
{
    qoutV.clear();
    
    for(auto& c : C)
    {
        qoutV.push_back(qinV[c.idx]);
    }
}

void P2P2DC::parseConstraints(std::string file_path,
                              std::vector<P2P2DC>& C)
{
    C.clear();
    std::ifstream fin(file_path);
    int idx, v_idx;
    while(fin.good())
    {
        fin >> idx;
        if(!fin.good()) break;
        fin >> v_idx;
        
        C.push_back(P2P2DC(v_idx, idx));
    }
}

void P2P3DC::getIndexList(const std::vector<P2P3DC>& C,
                          std::vector<int>& idxs)
{
    idxs.clear();
    
    for(auto& c : C)
    {
        idxs.push_back(c.v_idx);
    }
}

void P2P3DC::updateConstraints(const std::vector<P2P3DC>& C,
                                    const std::vector<Eigen::Vector4f>& qinV,
                                    std::vector<Eigen::Vector4f>& qoutV)
{
    qoutV.clear();
    
    for(auto& c : C)
    {
        qoutV.push_back(qinV[c.idx]);
    }
}

void P2P3DC::parseConstraints(std::string file_path,
                              std::vector<P2P3DC>& C)
{
    C.clear();
    std::ifstream fin(file_path);
    int idx, v_idx;
    while(fin.good())
    {
        fin >> idx;
        if(!fin.good()) break;
        fin >> v_idx;

        C.push_back(P2P3DC(v_idx, idx));
    }
}

void P2L2DC::getIndexList(const std::vector<P2L2DC>& C,
                          std::vector<int>& idxs)
{
    idxs.clear();
    
    for(auto& c : C)
    {
        idxs.push_back(c.v_idx);
    }
}

void P2L2DC::updateConstraints(const std::vector<P2L2DC>& C,
                               const std::vector<Eigen::Vector3f>& qinV,
                               const std::vector<Eigen::Vector2f>& pV,
                               std::vector<Eigen::Vector3f>& qoutV,
                               std::vector<Eigen::Vector2f>& nV)
{
    assert(pV.size() == C.size());
    
    qoutV.clear();
    nV.clear();
    
    Eigen::Vector3f q;
    for(int i = 0; i < C.size(); ++i)
    {
        nV.push_back(calcLineSegment(pV[i], qinV, q, C[i].start_idx, C[i].end_idx));
        qoutV.push_back(q);
    }
    
//    cv::Mat_<cv::Vec3b> tmp(720,1280, cv::Vec3b(255,255,255));
//    for(int i = 0; i < pV.size()-1; ++i)
//    {
//        cv::line(tmp, cv::Point(qinV[i](0),qinV[i](1)), cv::Point(qinV[i+1](0),qinV[i+1](1)), cv::Scalar(0,255,0));
//        cv::line(tmp, cv::Point(qoutV[i](0),qoutV[i](1)), cv::Point(qoutV[i](0)+5*nV[i](0),qoutV[i](1)+5*nV[i](1)), cv::Scalar(255,0,0));
//        cv::circle(tmp, cv::Point(pV[i](0),pV[i](1)), 1, cv::Scalar(0,0,255), -1);
//    }
//    
//    cv::imshow("tmp", tmp);
//    cv::waitKey(1);
}

void P2L2DC::parseConstraints(std::string file_path,
                              std::vector<P2L2DC>& C)
{
    C.clear();
    std::ifstream fin(file_path);
    int v_idx, s_idx, e_idx;
    while(fin.good())
    {
        fin >> v_idx;
        if(!fin.good()) break;
        fin >> s_idx;
        fin >> e_idx;
        
        C.push_back(P2L2DC(v_idx, s_idx, e_idx));
    }
}

float computeJacobianPoint2Point3D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const std::vector<Eigen::Vector3f>& pV,
                                   const std::vector<Eigen::Matrix3Xf>& dpV,
                                   const std::vector<Eigen::Vector4f>& qV,
                                   const float& w,
                                   bool robust)
{
    if(dpV.size() == 0 || w == 0.0)
        return 0.0;
    
    assert(pV.size() == dpV.size() && pV.size() == qV.size());
    assert(dpV[0].rows() == JtJ.rows());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    
    float error = 0;
    Eigen::Vector3f pq;
    float w_all;
    if(robust){
        std::vector<float> z, z_sorted;
        for(int i = 0; i < pV.size(); ++i)
        {
            pq = pV[i]-qV[i].segment<3>(0);
            z.push_back(pq.norm());
        }
        z_sorted = z;
        std::sort(z_sorted.begin(), z_sorted.end());
        float sigmasq = 1.43 * z_sorted[z.size()/2];
        sigmasq = sigmasq * sigmasq;
        
        for(int i = 0; i < pV.size(); ++i)
        {
            pq = pV[i]-qV[i].segment<3>(0);
            w_all = w * qV[i][3] * exp(-z[i]*z[i]/(2.0*sigmasq));
            
            Jtr += w_all * dpV[i].transpose() * pq;
            JtJ += w_all * dpV[i].transpose() * dpV[i];
            error += w_all * z[i] * z[i];
        }
    }
    else{
        for(int i = 0; i < pV.size(); ++i)
        {
            pq = pV[i]-qV[i].segment<3>(0);
            w_all = w * qV[i][3];
            Jtr += w_all * dpV[i].transpose() * pq;
            JtJ += w_all * dpV[i].transpose() * dpV[i];
            
            error += w_all * pq.squaredNorm();
        }
    }
    
    return error;
}

float computeJacobianPoint2Plane3D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const std::vector<Eigen::Vector3f>& pV,
                                   const std::vector<Eigen::Matrix3Xf>& dpV,
                                   const std::vector<Eigen::Vector4f>& qV,
                                   const std::vector<Eigen::Vector3f>& nV,
                                   const float& w,
                                   bool robust)
{
    if(dpV.size() == 0 || w == 0.0)
        return 0.0;
    
    assert(pV.size() == dpV.size() && pV.size() == qV.size() && pV.size() == nV.size());
    assert(dpV[0].rows() == JtJ.rows());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    
    float error = 0;
    float npq;
    Eigen::VectorXf ndp;
    float w_all;
    if(robust){
        std::vector<float> z, z_sorted;
        for(int i = 0; i < pV.size(); ++i)
        {
            npq = nV[i].dot(pV[i]-qV[i].segment<3>(0));
            z.push_back(npq);
            z_sorted.push_back(fabs(npq));
        }
        std::sort(z_sorted.begin(), z_sorted.end());
        float sigmasq = 1.43 * z_sorted[z.size()/2];
        sigmasq = sigmasq * sigmasq;
        
        for(int i = 0; i < pV.size(); ++i)
        {
            ndp = dpV[i].transpose() * nV[i];
            w_all = w * qV[i][3] * exp(-z[i] * z[i] / (2.0 * sigmasq));
            
            Jtr += w_all * z[i] * ndp;
            JtJ += w_all * ndp * ndp.transpose();
            error += w_all * z[i] * z[i];
        }
    }
    else{
        for(int i = 0; i < pV.size(); ++i)
        {
            npq = nV[i].dot(pV[i]-qV[i].segment<3>(0));
            ndp = dpV[i].transpose() * nV[i];
            w_all = w * qV[i][3];
            
            Jtr += w_all * npq * ndp;
            JtJ += w_all * ndp * ndp.transpose();
            error += w_all * npq * npq;
        }
    }
    
    return error;
}

float computeJacobianPoint2Point2D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const std::vector<Eigen::Vector2f>& pV,
                                   const std::vector<Eigen::Matrix2Xf>& dpV,
                                   const std::vector<Eigen::Vector3f>& qV,
                                   const float& w,
                                   bool robust)
{
    if(dpV.size() == 0 || w == 0.0)
        return 0.0;
    
    assert(pV.size() == dpV.size() && pV.size() == qV.size());
    assert(dpV[0].rows() == JtJ.rows());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    
    float error = 0;
    Eigen::Vector2f pq;
    float w_all;
    if(robust){
        std::vector<float> z, z_sorted;
        for(int i = 0; i < pV.size(); ++i)
        {
            pq = pV[i]-qV[i].segment<2>(0);
            z.push_back(pq.norm());
        }
        z_sorted = z;
        std::sort(z_sorted.begin(), z_sorted.end());
        float sigmasq = 1.43 * z_sorted[z.size()/2];
        sigmasq = sigmasq * sigmasq;
        
        
        for(int i = 0; i < pV.size(); ++i)
        {
            pq = pV[i]-qV[i].segment<2>(0);
            w_all = w * qV[i][2] * exp(-z[i]*z[i]/(2.0*sigmasq));
            
            Jtr += w_all * dpV[i].transpose() * pq;
            JtJ += w_all * dpV[i].transpose() * dpV[i];
            error += w_all * z[i] * z[i];
        }
    }
    else{
        for(int i = 0; i < pV.size(); ++i)
        {
            pq = pV[i]-qV[i].segment<2>(0);
            w_all = w * qV[i][2];
            Jtr += w_all * dpV[i].transpose() * pq;
            JtJ += w_all * dpV[i].transpose() * dpV[i];
            
            error += w_all * pq.squaredNorm();
        }
    }
    
    return error;
}

float computeJacobianPoint2Point2D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const std::vector<Eigen::Vector2f>& pV,
                                   const std::vector<Eigen::Matrix2Xf>& dpV,
                                   const std::vector<Eigen::Vector3f>& qV,
                                   const float& w,
                                   bool robust,
                                   std::vector<int>& idx)
{
    if(dpV.size() == 0 || w == 0.0)
        return 0.0;
    
    assert(pV.size() == dpV.size());
    assert(qV.size() == idx.size());
    assert(dpV[0].rows() == JtJ.rows());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    
    float error = 0;
    Eigen::Vector2f pq;
    float w_all;
    if(robust){
        std::vector<float> z, z_sorted;
        for(int i = 0; i < qV.size(); ++i)
        {
            pq = pV[idx[i]]-qV[i].segment<2>(0);
            z.push_back(pq.norm());
        }
        z_sorted = z;
        std::sort(z_sorted.begin(), z_sorted.end());
        float sigmasq = 1.43 * z_sorted[z.size()/2];
        sigmasq = sigmasq * sigmasq;
        
        for(int i = 0; i < qV.size(); ++i)
        {
            pq = pV[idx[i]]-qV[i].segment<2>(0);
            w_all = w * qV[i][2] * exp(-z[i]*z[i]/(2.0*sigmasq));
            
            Jtr += w_all * dpV[idx[i]].transpose() * pq;
            JtJ += w_all * dpV[idx[i]].transpose() * dpV[idx[i]];
            error += w_all * z[i] * z[i];
        }
    }
    else{
        for(int i = 0; i < qV.size(); ++i)
        {
            pq = pV[idx[i]]-qV[i].segment<2>(0);
            w_all = w * qV[i][2];
            
            Jtr += w_all * dpV[idx[i]].transpose() * pq;
            JtJ += w_all * dpV[idx[i]].transpose() * dpV[idx[i]];
            
            error += w_all * pq.squaredNorm();
        }
    }
    
    return error;
}

float computeJacobianPoint2Line2D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                  Eigen::Ref<Eigen::MatrixXf> JtJ,
                                  const std::vector<Eigen::Vector2f>& pV,
                                  const std::vector<Eigen::Matrix2Xf>& dpV,
                                  const std::vector<Eigen::Vector3f>& qV,
                                  const std::vector<Eigen::Vector2f>& nV,
                                  const float& w,
                                  bool robust)
{
    if(dpV.size() == 0 || w == 0.0)
        return 0.0;
    
    assert(pV.size() == dpV.size() && pV.size() == qV.size() && pV.size() == nV.size());
    assert(dpV[0].rows() == JtJ.rows());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    
    float error = 0;
    float npq;
    float w_all;
    Eigen::VectorXf ndp;
    if(robust){
        std::vector<float> z, z_sorted;
        for(int i = 0; i < pV.size(); ++i)
        {
            npq = nV[i].dot(pV[i]-qV[i].segment<2>(0));
            z.push_back(npq);
            z_sorted.push_back(fabs(npq));
        }
        std::sort(z_sorted.begin(), z_sorted.end());
        float sigmasq = 1.43 * z_sorted[z.size()/2];
        sigmasq = sigmasq * sigmasq;
        
        
        for(int i = 0; i < pV.size(); ++i)
        {
            ndp = dpV[i].transpose() * nV[i];
            w_all = w * qV[i][2] * exp(-z[i] * z[i] / (2.0 * sigmasq));
            
            Jtr += w_all * z[i] * ndp;
            JtJ += w_all * ndp * ndp.transpose();
            error += w_all * z[i] * z[i];
        }
    }
    else{
        for(int i = 0; i < pV.size(); ++i)
        {
            npq = nV[i].dot(pV[i]-qV[i].segment<2>(0));
            ndp = dpV[i].transpose() * nV[i];
            w_all = w * qV[i][2];
            
            Jtr += w_all * npq * ndp;
            JtJ += w_all * ndp * ndp.transpose();
            
            error += w_all * npq * npq;
        }
    }
    
    return error;
}

float computeJacobianPoint2Line2D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                  Eigen::Ref<Eigen::MatrixXf> JtJ,
                                  const std::vector<Eigen::Vector2f>& pV,
                                  const std::vector<Eigen::Matrix2Xf>& dpV,
                                  const std::vector<Eigen::Vector3f>& qV,
                                  const std::vector<Eigen::Vector2f>& nV,
                                  const float& w,
                                  bool robust,
                                  std::vector<int>& idx)
{
    if(dpV.size() == 0 || w == 0.0)
        return 0.0;
    
    assert(pV.size() == dpV.size());
    assert(qV.size() == idx.size());
    assert(nV.size() == qV.size());
    assert(dpV[0].rows() == JtJ.rows());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    
    float error = 0;
    float npq;
    float w_all;
    Eigen::VectorXf ndp;
    if(robust){
        std::vector<float> z, z_sorted;
        for(int i = 0; i < qV.size(); ++i)
        {
            npq = nV[i].dot(pV[idx[i]]-qV[i].segment<2>(0));
            z.push_back(npq);
            z_sorted.push_back(fabs(npq));
        }
        std::sort(z_sorted.begin(), z_sorted.end());
        float sigmasq = 1.43 * z_sorted[z.size()/2];
        sigmasq = sigmasq * sigmasq;
        
        
        for(int i = 0; i < qV.size(); ++i)
        {
            ndp = dpV[idx[i]].transpose() * nV[i];
            w_all = w * qV[i][2] * exp(-z[i] * z[i] / (2.0 * sigmasq));
            
            Jtr += w_all * z[i] * ndp;
            JtJ += w_all * ndp * ndp.transpose();
            error += w_all * z[i] * z[i];
        }
    }
    else{
        for(int i = 0; i < qV.size(); ++i)
        {
            npq = nV[i].dot(pV[idx[i]]-qV[i].segment<2>(0));
            ndp = dpV[idx[i]].transpose() * nV[i];
            w_all = w * qV[i][2];
            
            Jtr += w_all * npq * ndp;
            JtJ += w_all * ndp * ndp.transpose();
            error += w_all * npq * npq;
        }
    }
    
    return error;
}
