#include "face_gradient.h"

void computeV(const FaceData& fd,
              const std::vector<int>& idx,
              std::vector<Eigen::Vector3f>& V)
{
    if(V.size() != idx.size())
        V.resize(idx.size());
    
    for(int i = 0; i < idx.size(); ++i)
    {
        V[i] = fd.computeV(idx[i]);
    }
}

void computeVertexWiseGradPosition2D(std::vector<Eigen::Vector2f>& pV,
                                     std::vector<Eigen::Matrix2Xf>& dpV,
                                     const FaceData& fd,
                                     const Eigen::Vector6f& rtc,
                                     const Eigen::Vector6f& rtf,
                                     const Eigen::Matrix4f& I,
                                     const DOF& dof,
                                     const std::vector<int>& vert_list)
{
    const Eigen::VectorXf& V = fd.pts_;
    
    if (vert_list.size() == 0){
        if (pV.size() != V.size()/3)
            pV.assign(V.size()/3, Eigen::Vector2f::Zero());
        if (dpV.size() != V.size()/3)
            dpV.assign(V.size()/3, Eigen::Matrix2Xf::Zero(2, dof.pos()));
    }
    else{
        if (pV.size() != vert_list.size())
            pV.assign(vert_list.size(), Eigen::Vector2f::Zero());
        if (dpV.size() != vert_list.size())
            dpV.assign(vert_list.size(), Eigen::Matrix2Xf::Zero(2, dof.pos()));
    }
    
    // compute position term per vertex
    Eigen::Matrix4f RTc = Eigen::EulerAnglesPoseToMatrix(rtc);
    Eigen::MatrixXf RTf = Eigen::EulerAnglesPoseToMatrix(rtf);
    const Eigen::Matrix3f& Rc = RTc.block<3,3>(0, 0);
    const Eigen::Matrix3f& Rf = RTf.block<3,3>(0, 0);
    
    Eigen::Matrix3f dRc[3];
    gradRotEuler(rtc[0], rtc[1], rtc[2], dRc);
    
    Eigen::Matrix3f dRf[3];
    gradRotEuler(rtf[0], rtf[1], rtf[2], dRf);
    
    int cur_dof;
    Eigen::Vector3f RTv, RTRTv, IRTRTv;
    Eigen::Matrix<float, 2, 3> dPi, dI, dIRc, dIRcf;
    if(vert_list.size() != 0)
        for (int i = 0; i < vert_list.size(); ++i)
        {
            cur_dof = 0;
            Eigen::Ref<Eigen::MatrixXf> dpdid  = dpV[i].block(0,0,2,dof.ID);         cur_dof += dof.ID;
            Eigen::Ref<Eigen::MatrixXf> dpdex  = dpV[i].block(0,cur_dof,2,dof.EX);   cur_dof += dof.EX;
            Eigen::Ref<Eigen::MatrixXf> dpdrf  = dpV[i].block(0,cur_dof,2,dof.fROT); cur_dof += dof.fROT;
            Eigen::Ref<Eigen::MatrixXf> dpdtrf = dpV[i].block(0,cur_dof,2,dof.fTR);  cur_dof += dof.fTR;
            Eigen::Ref<Eigen::MatrixXf> dpdrc  = dpV[i].block(0,cur_dof,2,dof.cROT); cur_dof += dof.cROT;
            Eigen::Ref<Eigen::MatrixXf> dpdtrc = dpV[i].block(0,cur_dof,2,dof.cTR);  cur_dof += dof.cTR;
            Eigen::Ref<Eigen::MatrixXf> dpdcam = dpV[i].block(0,cur_dof,2,dof.CAM);
            
            const Eigen::Vector3f& v = V.b3(vert_list[i]);
            RTv = ApplyTransform(RTf, v);
            RTRTv = ApplyTransform(RTc, RTv);
            RTRTv(2) = (RTRTv(2) > EPSILON) ? RTRTv(2) : EPSILON;
            IRTRTv = I.block<3,3>(0,0) * RTRTv;
            
            dPi << 1.0/IRTRTv(2), 0 , -IRTRTv(0)/IRTRTv(2)/IRTRTv(2), 0, 1.0/IRTRTv(2), -IRTRTv(1)/IRTRTv(2)/IRTRTv(2);
            dI = dPi * I.block<3,3>(0,0);
            dIRc = dI * Rc;
            dIRcf = dIRc * Rf;
            
            pV[i](0) = IRTRTv(0)/IRTRTv(2);
            pV[i](1) = IRTRTv(1)/IRTRTv(2);
            
            if(dof.ID != 0)   gradV(dpdid, dIRcf, fd.dID(vert_list[i], dof.ID));
            if(dof.EX != 0)   gradV(dpdex, dIRcf, fd.dEX(vert_list[i], dof.EX));
            if(dof.fROT != 0) gradROT(dpdrf, dIRc, dRf, v);
            if(dof.fTR != 0)  gradTR(dpdtrf, dIRc);
            if(dof.cROT != 0) gradROT(dpdrc, dI, dRc, RTv);
            if(dof.cTR != 0)  gradTR(dpdtrc, dI);
            if(dof.CAM != 0)  gradCAM(dpdcam, dPi, RTRTv, dof.CAM);
        }
    else
        for (int i = 0; i < V.size()/3; ++i)
        {
            cur_dof = 0;
            Eigen::Ref<Eigen::MatrixXf> dpdid  = dpV[i].block(0,0,2,dof.ID);         cur_dof += dof.ID;
            Eigen::Ref<Eigen::MatrixXf> dpdex  = dpV[i].block(0,cur_dof,2,dof.EX);   cur_dof += dof.EX;
            Eigen::Ref<Eigen::MatrixXf> dpdrf  = dpV[i].block(0,cur_dof,2,dof.fROT); cur_dof += dof.fROT;
            Eigen::Ref<Eigen::MatrixXf> dpdtrf = dpV[i].block(0,cur_dof,2,dof.fTR);  cur_dof += dof.fTR;
            Eigen::Ref<Eigen::MatrixXf> dpdrc  = dpV[i].block(0,cur_dof,2,dof.cROT); cur_dof += dof.cROT;
            Eigen::Ref<Eigen::MatrixXf> dpdtrc = dpV[i].block(0,cur_dof,2,dof.cTR);  cur_dof += dof.cTR;
            Eigen::Ref<Eigen::MatrixXf> dpdcam = dpV[i].block(0,cur_dof,2,dof.CAM);
            
            const Eigen::Vector3f& v = V.b3(i);
            RTv = ApplyTransform(RTf, v);
            RTRTv = ApplyTransform(RTc, RTv);
            RTRTv(2) = (RTRTv(2) > EPSILON) ? RTRTv(2) : EPSILON;
            IRTRTv = I.block<3,3>(0,0) * RTRTv;
            
            dPi << 1.0/IRTRTv(2), 0 , -IRTRTv(0)/IRTRTv(2)/IRTRTv(2), 0, 1.0/IRTRTv(2), -IRTRTv(1)/IRTRTv(2)/IRTRTv(2);
            dI = dPi * I.block<3,3>(0,0);
            dIRc = dI * Rc;
            dIRcf = dIRc * Rf;
            
            pV[i](0) = IRTRTv(0)/IRTRTv(2);
            pV[i](1) = IRTRTv(1)/IRTRTv(2);
            
            if(dof.ID != 0)   gradV(dpdid, dIRcf, fd.dID(i, dof.ID));
            if(dof.EX != 0)   gradV(dpdex, dIRcf, fd.dEX(i, dof.EX));
            if(dof.fROT != 0) gradROT(dpdrf, dIRc, dRf, v);
            if(dof.fTR != 0)  gradTR(dpdtrf, dIRc);
            if(dof.cROT != 0) gradROT(dpdrc, dI, dRc, RTv);
            if(dof.cTR != 0)  gradTR(dpdtrc, dI);
            if(dof.CAM != 0)  gradCAM(dpdcam, dPi, RTRTv, dof.CAM);
        }
}

void computeVertexWiseGradPosition2D(std::vector<Eigen::Vector2f>& pV,
                                     std::vector<Eigen::Matrix2Xf>& dpV,
                                     const std::vector<Eigen::Vector3f>& V,
                                     const FaceData& fd,
                                     const Eigen::Vector6f& rtc,
                                     const Eigen::Vector6f& rtf,
                                     const Eigen::Matrix4f& I,
                                     const DOF& dof,
                                     const std::vector<int>& vert_list)
{
    assert(vert_list.size() == V.size());
    if(vert_list.size() == 0) return;
    
    if (pV.size() != vert_list.size())
        pV.assign(vert_list.size(), Eigen::Vector2f::Zero());
    if (dpV.size() != vert_list.size())
        dpV.assign(vert_list.size(), Eigen::Matrix2Xf::Zero(2,dof.pos()));
    
    // compute position term per vertex
    Eigen::Matrix4f RTc = Eigen::EulerAnglesPoseToMatrix(rtc);
    Eigen::MatrixXf RTf = Eigen::EulerAnglesPoseToMatrix(rtf);
    const Eigen::Matrix3f& Rc = RTc.block<3,3>(0, 0);
    const Eigen::Matrix3f& Rf = RTf.block<3,3>(0, 0);
    
    Eigen::Matrix3f dRc[3];
    gradRotEuler(rtc[0], rtc[1], rtc[2], dRc);
    
    Eigen::Matrix3f dRf[3];
    gradRotEuler(rtf[0], rtf[1], rtf[2], dRf);
    
    int cur_dof;
    Eigen::Vector3f RTv, RTRTv, IRTRTv;
    Eigen::Matrix<float, 2, 3> dPi, dI, dIRc, dIRcf;
    for (int i = 0; i < vert_list.size(); ++i)
    {
        cur_dof = 0;
        Eigen::Ref<Eigen::MatrixXf> dpdid  = dpV[i].block(0,0,2,dof.ID);         cur_dof += dof.ID;
        Eigen::Ref<Eigen::MatrixXf> dpdex  = dpV[i].block(0,cur_dof,2,dof.EX);   cur_dof += dof.EX;
        Eigen::Ref<Eigen::MatrixXf> dpdrf  = dpV[i].block(0,cur_dof,2,dof.fROT); cur_dof += dof.fROT;
        Eigen::Ref<Eigen::MatrixXf> dpdtrf = dpV[i].block(0,cur_dof,2,dof.fTR);  cur_dof += dof.fTR;
        Eigen::Ref<Eigen::MatrixXf> dpdrc  = dpV[i].block(0,cur_dof,2,dof.cROT); cur_dof += dof.cROT;
        Eigen::Ref<Eigen::MatrixXf> dpdtrc = dpV[i].block(0,cur_dof,2,dof.cTR);  cur_dof += dof.cTR;
        Eigen::Ref<Eigen::MatrixXf> dpdcam = dpV[i].block(0,cur_dof,2,dof.CAM);
        
        RTv = ApplyTransform(RTf, V[i]);
        RTRTv = ApplyTransform(RTc, RTv);
        RTRTv(2) = (RTRTv(2) > EPSILON) ? RTRTv(2) : EPSILON;
        IRTRTv = I.block<3,3>(0,0) * RTRTv;
        
        dPi << 1.0/IRTRTv(2), 0 , -IRTRTv(0)/IRTRTv(2)/IRTRTv(2), 0, 1.0/IRTRTv(2), -IRTRTv(1)/IRTRTv(2)/IRTRTv(2);
        dI = dPi * I.block<3,3>(0,0);
        dIRc = dI * Rc;
        dIRcf = dIRc * Rf;

        pV[i](0) = IRTRTv(0)/IRTRTv(2);
        pV[i](1) = IRTRTv(1)/IRTRTv(2);
        
        if(dof.ID != 0)   gradV(dpdid, dIRcf, fd.dID(vert_list[i], dof.ID));
        if(dof.EX != 0)   gradV(dpdex, dIRcf, fd.dEX(vert_list[i], dof.EX));
        if(dof.fROT != 0) gradROT(dpdrf, dIRc, dRf, V[i]);
        if(dof.fTR != 0)  gradTR(dpdtrf, dIRc);
        if(dof.cROT != 0) gradROT(dpdrc, dI, dRc, RTv);
        if(dof.cTR != 0)  gradTR(dpdtrc, dI);
        if(dof.CAM != 0)  gradCAM(dpdcam, dPi, RTRTv, dof.CAM);
    }
}

void computeVertexWiseGradPosition3D(std::vector<Eigen::Vector3f>& pV,
                                     std::vector<Eigen::Matrix3Xf>& dpV,
                                     const FaceData& fd,
                                     const Eigen::Vector6f& rt,
                                     const DOF& dof,
                                     const std::vector<int>& vert_list)
{
    const Eigen::VectorXf& V = fd.pts_;
    
    if (vert_list.size() == 0){
        if (pV.size() != V.size()/3)
            pV.assign(V.size()/3, Eigen::Vector3f::Zero());
        if (dpV.size() != V.size()/3)
            dpV.assign(V.size()/3, Eigen::Matrix3Xf::Zero(3, dof.pos()));
    }
    else{
        if (pV.size() != vert_list.size())
            pV.assign(vert_list.size(), Eigen::Vector3f::Zero());
        if (dpV.size() != vert_list.size())
            dpV.assign(vert_list.size(), Eigen::Matrix3Xf::Zero(3, dof.pos()));
    }
    
    // compute position term per vertex
    Eigen::Matrix4f RTf = Eigen::EulerAnglesPoseToMatrix(rt);
    const Eigen::Matrix3f& R = RTf.block<3,3>(0, 0);
    const Eigen::Vector3f& t = RTf.block<3,1>(0,3);
    
    Eigen::Matrix3f dR[3];
    gradRotEuler(rt[0], rt[1], rt[2], dR);
    
    int cur_dof;
    if(vert_list.size() != 0)
        for (int i = 0; i < vert_list.size(); ++i)
        {
            cur_dof = 0;
            Eigen::Ref<Eigen::MatrixXf> dpdid  = dpV[i].block(0,0,3,dof.ID);         cur_dof += dof.ID;
            Eigen::Ref<Eigen::MatrixXf> dpdex  = dpV[i].block(0,cur_dof,3,dof.EX);   cur_dof += dof.EX;
            Eigen::Ref<Eigen::MatrixXf> dpdrf  = dpV[i].block(0,cur_dof,3,dof.fROT); cur_dof += dof.fROT;
            Eigen::Ref<Eigen::MatrixXf> dpdtrf = dpV[i].block(0,cur_dof,3,dof.fTR);
            
            const Eigen::Vector3f& v = V.b3(vert_list[i]);
            pV[i] = ApplyTransform(RTf, v);
            pV[i](2) = (pV[i](2) > EPSILON) ? pV[i](2) : EPSILON;
            
            if(dof.ID != 0)   gradV(dpdid, R, fd.dID(vert_list[i], dof.ID));
            if(dof.EX != 0)   gradV(dpdex, R, fd.dEX(vert_list[i], dof.EX));
            if(dof.fROT != 0) gradROT(dpdrf, Eigen::MatrixXf(), dR, v);
            if(dof.fTR != 0)  gradTR(dpdtrf, Eigen::MatrixXf());
        }
    else
        for (int i = 0; i < V.size()/3; ++i)
        {
            cur_dof = 0;
            Eigen::Ref<Eigen::MatrixXf> dpdid  = dpV[i].block(0,0,3,dof.ID);         cur_dof += dof.ID;
            Eigen::Ref<Eigen::MatrixXf> dpdex  = dpV[i].block(0,cur_dof,3,dof.EX);   cur_dof += dof.EX;
            Eigen::Ref<Eigen::MatrixXf> dpdrf  = dpV[i].block(0,cur_dof,3,dof.fROT); cur_dof += dof.fROT;
            Eigen::Ref<Eigen::MatrixXf> dpdtrf = dpV[i].block(0,cur_dof,3,dof.fTR);
            
            const Eigen::Vector3f& v = V.b3(i);
            pV[i] = ApplyTransform(RTf, v);
            pV[i](2) = (pV[i](2) > EPSILON) ? pV[i](2) : EPSILON;

            if(dof.ID != 0)   gradV(dpdid, R, fd.dID(i, dof.ID));
            if(dof.EX != 0)   gradV(dpdex, R, fd.dEX(i, dof.EX));
            if(dof.fROT != 0) gradROT(dpdrf, Eigen::MatrixXf(), dR, v);
            if(dof.fTR != 0)  gradTR(dpdtrf, Eigen::MatrixXf());
        }
}

void computeVertexWiseGradNormal(std::vector<Eigen::Vector3f>& nV,
                                 std::vector<Eigen::Matrix3Xf>& dnV,
                                 const FaceData& fd,
                                 const DOF& dof)
{
    const Eigen::VectorXf& V = fd.pts_;
    const Eigen::MatrixX3i& tri = fd.model_->tri_pts_;
    
    Eigen::Vector3f f;
    std::vector<Eigen::RowVectorXf> df(3, Eigen::RowVectorXf::Zero(dof.ID + dof.EX));
    std::vector<Eigen::Vector3f> fV(V.size()/3, Eigen::Vector3f::Zero());
    std::vector<Eigen::Matrix3Xf> dfV(V.size()/3, Eigen::Matrix3Xf::Zero(3, dof.ID + dof.EX));
    Eigen::Vector3f v1_v0, v2_v0;
    for (int i = 0; i < tri.rows(); ++i)
    {
        const int& idx0 = tri(i, 0);
        const int& idx1 = tri(i, 1);
        const int& idx2 = tri(i, 2);
        
        v1_v0 = V.b3(idx1) - V.b3(idx0);
        v2_v0 = V.b3(idx2) - V.b3(idx0);
        
        const Eigen::Matrix3Xf& id_edge1 = fd.dIDEdge(i, 0);
        const Eigen::Matrix3Xf& id_edge2 = fd.dIDEdge(i, 1);
        const Eigen::Matrix3Xf& ex_edge1 = fd.dEXEdge(i, 0);
        const Eigen::Matrix3Xf& ex_edge2 = fd.dEXEdge(i, 1);
        
        gradF(df[0], v1_v0, v2_v0, id_edge1, id_edge2, ex_edge1, ex_edge2, 1, 2, dof.ID, dof.EX);
        gradF(df[1], v1_v0, v2_v0, id_edge1, id_edge2, ex_edge1, ex_edge2, 2, 0, dof.ID, dof.EX);
        gradF(df[2], v1_v0, v2_v0, id_edge1, id_edge2, ex_edge1, ex_edge2, 0, 1, dof.ID, dof.EX);
        f = (v1_v0).cross(v2_v0);
        
        fV[idx0] += f;
        fV[idx1] += f;
        fV[idx2] += f;
        
        dfV[idx0].row(0) += df[0];
        dfV[idx1].row(0) += df[0];
        dfV[idx2].row(0) += df[0];
        
        dfV[idx0].row(1) += df[1];
        dfV[idx1].row(1) += df[1];
        dfV[idx2].row(1) += df[1];
        
        dfV[idx0].row(2) += df[2];
        dfV[idx1].row(2) += df[2];
        dfV[idx2].row(2) += df[2];
    }
    
    nV = fV;
    for (int i = 0; i < V.size()/3; ++i)
    {
        float g = fV[i].norm();
        if (g < EPSILON) g = EPSILON;
        dnV[i] = (g*dfV[i] - 1.0f / g * fV[i] * fV[i].transpose() * dfV[i]) / (g*g);
        
        nV[i] /= g; // compute normal
    }
}

float computeJacobianSymmetry(Eigen::Ref<Eigen::VectorXf> Jtr,
                              Eigen::Ref<Eigen::MatrixXf> JtJ,
                              const FaceData& fd,
                              const DOF& dof,
                              const float& w,
                              bool withexp)
{
	if ((dof.ID == 0 && !withexp) || (dof.ID + dof.EX == 0 && withexp)) return 0.0;

	Eigen::Vector3f v;
    Eigen::MatrixXf dv;
    float err = 0.0;
	for (int i = 0; i < fd.model_->n_sym_pair(); i++)
	{
        if(withexp)
            fd.dSym(i, 0, dof.ID, dof.EX, v, dv); // x plane symmetry
        else
            fd.dSym(i, 0, dof.ID, 0, v, dv);
        
		Jtr += w * dv.transpose() * v;
		JtJ += w * dv.transpose() * dv;
        
        err += w * v.squaredNorm();
	}
    
    return err;
}

float computeJacobianPoint2Point3D(Eigen::Ref<Eigen::VectorXf> Jtr,
                                   Eigen::Ref<Eigen::MatrixXf> JtJ,
                                   const FaceData& fd,
                                   const std::vector< int >& index_list,
                                   const DOF& dof,
                                   const float w)
{
	Eigen::Vector3f p12;
	Eigen::Matrix3Xf dp12 = Eigen::Matrix3Xf::Zero(3, dof.all());
    const Eigen::VectorXf& V = fd.pts_;
    float err = 0.0;
	for (int i = 0; i < index_list.size(); i += 2)
	{
		const int& i1 = index_list[i];
		const int& i2 = index_list[i + 1];
		const Eigen::Vector3f& v1 = V.b3(i1);
		const Eigen::Vector3f& v2 = V.b3(i2);
        
		p12 = v1 - v2;

		if (dof.ID != 0)
            dp12.block(0, 0, 3, dof.ID) = fd.dID(i1, dof.ID) - fd.dID(i2, dof.ID);
        if (dof.EX != 0)
			dp12.block(0, dof.ID, 3, dof.EX) = fd.dEX(i1, dof.EX) - fd.dEX(i2, dof.EX);

		Jtr += w * dp12.transpose() * p12;
		JtJ += w * dp12.transpose() * dp12;
        
        err += w * p12.squaredNorm();
	}
    
    return err;
}

void setFaceVector(Eigen::Ref<Eigen::VectorXf> X,
                   Eigen::Vector6f& rt,
                   const FaceData& fd,
                   const DOF& dof)
{
    bool tinvar = false, tvar = false;
    if(X.size() == dof.face()){
        tinvar = true; tvar = true;
    }
    else if(X.size() == dof.ftinvar()){
        tinvar = true;
    }
    else if(X.size() == dof.tvar()){
        tvar = true;
    }
    else{
        throw std::runtime_error("Error: face vector dimention does not match");
    }
    assert(dof.AL <= fd.alCoeff.size());
    assert(dof.ID <= fd.idCoeff.size());
    assert(dof.EX <= fd.exCoeff.size());
    
    int cur_dof = 0;
    if(tinvar){
        for (int i = 0; i < dof.AL; ++i)
        {
            X[cur_dof + i] = fd.alCoeff[i];
        }
        cur_dof += dof.AL;
        for (int i = 0; i < dof.ID; ++i)
        {
            X[cur_dof + i] = fd.idCoeff[i];
        }
        cur_dof += dof.ID;
    }
    if(tvar){
        for (int i = 0; i < dof.EX; ++i)
        {
            X[cur_dof + i] = fd.exCoeff[i];
        }
        cur_dof += dof.EX;
        
        Eigen::ConvertToEulerAnglesPose(fd.RT, rt);
        if (dof.fROT == 3 || dof.fTR == 3){
            for (int i = 0; i < dof.fROT; ++i)
            {
                X[cur_dof + i] = rt[i];
            }
            cur_dof += dof.fROT;
            
            for (int i = 0; i < dof.fTR; ++i)
            {
                X[cur_dof + i] = rt[3 + i];
            }
            cur_dof += dof.fTR;
        }
        
        assert(dof.SH == 27 || dof.SH == 9 || dof.SH == 0);
        switch(dof.SH)
        {
            case 27:
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 9; ++j)
                    {
                        X[cur_dof + i * 9 + j] = fd.SH(i,j);
                    }
                }
                break;
            case 9:
                for (int j = 0; j < 9; ++j)
                {
                    X[cur_dof + j] = fd.SH(0,j);
                }
                break;
            default:
                break;
        }
    }
}

void setCameraVector(Eigen::Ref<Eigen::VectorXf> X,
                     Eigen::Vector6f& rt,
                     const Camera& camera,
                     const DOF& dof)
{
    assert(X.size() == dof.camera());
    
    int cur_dof = 0;
	Eigen::ConvertToEulerAnglesPose(camera.extrinsic_, rt);
	if (dof.cROT == 3 || dof.cTR == 3){

		for (int i = 0; i < dof.cROT; ++i)
		{
			X[cur_dof + i] = rt[i];
		}
        cur_dof += dof.cROT;
		for (int i = 0; i < dof.cTR; ++i)
		{
			X[cur_dof + i] = rt[3 + i];
		}
        cur_dof += dof.cTR;
	}

    const Eigen::Matrix4f& I = camera.intrinsic_;
	
	// for debugging
	const float scale = 0.1f;
	switch (dof.CAM){
	case 1:
		X[cur_dof + 0] = scale*I(0, 0);
		break;
	case 2:
		X[cur_dof + 0] = scale*I(0, 0);
		X[cur_dof + 1] = scale*I(1, 1);
	case 3:
		X[cur_dof + 0] = scale*I(0, 0);
		X[cur_dof + 1] = scale*I(0, 2);
		X[cur_dof + 2] = scale*I(1, 2);
		break;
	case 4:
		X[cur_dof + 0] = scale*I(0, 0);
		X[cur_dof + 1] = scale*I(1, 1);
		X[cur_dof + 2] = scale*I(0, 2);
		X[cur_dof + 3] = scale*I(1, 2);
		break;
	default:
		break;
	}
}

void loadFaceVector(const Eigen::VectorXf& X,
                    Eigen::Vector6f& rt,
                    FaceData& fd,
                    const DOF& dof)
{
    bool tinvar = false, tvar = false;
    if(X.size() == dof.face()){
        tinvar = true; tvar = true;
    }
    else if(X.size() == dof.ftinvar()){
        tinvar = true;
    }
    else if(X.size() == dof.tvar()){
        tvar = true;
    }
    else{
        throw std::runtime_error("Error: face vector dimention does not match");
    }
    assert(dof.AL <= fd.alCoeff.size());
    assert(dof.ID <= fd.idCoeff.size());
    assert(dof.EX <= fd.exCoeff.size());
    
    int cur_dof = 0;
    if(tinvar){
        for (int j = 0; j < dof.AL; ++j)
        {
            fd.alCoeff[j] = X[cur_dof + j];
        }
        cur_dof += dof.AL;
        
        for (int j = 0; j < dof.ID; ++j)
        {
            fd.idCoeff[j] = X[cur_dof + j];
        }
        cur_dof += dof.ID;
    }
    
    if(tvar){
        for (int j = 0; j < dof.EX; ++j)
        {
            fd.exCoeff[j] = X[cur_dof + j];
        }
        cur_dof += dof.EX;

        for (int i = 0; i < dof.fROT; ++i)
        {
            rt[i] = X[cur_dof + i];
        }
        cur_dof += dof.fROT;

        for (int i = 0; i < dof.fTR; ++i)
        {
            rt[3 + i] = X[cur_dof + i];
        }
        cur_dof += dof.fTR;

        if (dof.fROT != 0 || dof.fTR != 0){
            fd.RT = Eigen::EulerAnglesPoseToMatrix(rt);
        }
        
        assert(dof.SH == 27 || dof.SH == 9 || dof.SH == 0);
        
        switch(dof.SH)
        {
            case 27:
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 9; ++j)
                    {
                        fd.SH(i,j) = X[cur_dof + i * 9 + j];
                    }
                }
                break;
            case 9:
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 9; ++j)
                    {
                        fd.SH(i,j) = X[cur_dof + j];
                    }
                }
                break;
            default:
                break;
        }
    }
}

void loadCameraVector(const Eigen::VectorXf& X,
                      Eigen::Vector6f& rt,
                      Camera& camera,
                      const DOF& dof)
{
    assert(X.size() == dof.camera());
    
    int cur_dof = 0;
    // update camera
    for (int i = 0; i < dof.cROT; ++i)
    {
        rt[i] = X[cur_dof + i];
    }
    cur_dof += dof.cROT;
    
    for (int i = 0; i < dof.cTR; ++i)
    {
        rt[3 + i] = X[cur_dof + i];
    }
    cur_dof += dof.cTR;
    
    if (dof.cROT != 0 || dof.cTR != 0){
        camera.extrinsic_ = Eigen::EulerAnglesPoseToMatrix(rt);
    }
    
    const float scale = 10.0f;
    Eigen::Matrix4f& I = camera.intrinsic_;
    switch (dof.CAM){
        case 1:
            I(0, 0) = scale*X[cur_dof + 0];
            I(1, 1) = scale*X[cur_dof + 0];
            break;
        case 2:
            I(0, 0) = scale*X[cur_dof + 0];
            I(1, 1) = scale*X[cur_dof + 1];
            break;
        case 3:
            I(0, 0) = scale*X[cur_dof + 0];
            I(1, 1) = scale*X[cur_dof + 0];
            I(0, 2) = scale*X[cur_dof + 1];
            I(1, 2) = scale*X[cur_dof + 2];
            break;
        case 4:
            I(0, 0) = scale*X[cur_dof + 0];
            I(1, 1) = scale*X[cur_dof + 1];
            I(0, 2) = scale*X[cur_dof + 2];
            I(1, 2) = scale*X[cur_dof + 3];
            break;
        default:
            break;
    }
    cur_dof += dof.CAM;
}
