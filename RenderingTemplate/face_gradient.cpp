#include "face_gradient.h"

void computeEdgeBasis(std::vector<std::array<Eigen::Matrix3Xf, 2>>& id_edge,
                      std::vector<std::array<Eigen::Matrix3Xf, 2>>& exp_edge,
                      const Eigen::MatrixXf& w_id,
                      const Eigen::MatrixXf& w_exp,
                      const Eigen::MatrixX3i& tri,
                      const DOF& dof)
{
	for (int i = 0; i < tri.rows(); ++i)
	{
		if (dof.ID != 0){
			id_edge[i][0] = w_id.block(3 * tri(i,1), 0, 3, dof.ID) - w_id.block(3 * tri(i,0), 0, 3, dof.ID);
			id_edge[i][1] = w_id.block(3 * tri(i,2), 0, 3, dof.ID) - w_id.block(3 * tri(i,0), 0, 3, dof.ID);
		}

		if (dof.EX != 0){
			exp_edge[i][0] = w_exp.block(3 * tri(i,1), 0, 3, dof.EX) - w_exp.block(3 * tri(i,0), 0, 3, dof.EX);
			exp_edge[i][1] = w_exp.block(3 * tri(i,2), 0, 3, dof.EX) - w_exp.block(3 * tri(i,0), 0, 3, dof.EX);
		}
	}
}

void jacobianID(Eigen::MatrixX2f& result,
                const Eigen::Vector3f& pp,
                const float& zsq,
                const Eigen::Matrix3Xf& id,
                const Eigen::Matrix4f& I,
                const Eigen::Matrix3f& R,
                unsigned int start,
                unsigned int size)
{
    assert(id.cols() >= size);
    assert(result.rows() >= start + size);
    
    Eigen::RowVector2f dpdx;
    // jacobian for identity
    Eigen::Vector3f dpp;
    for (int i = 0; i < size; ++i)
    {
        dpp = R * Eigen::Vector3f(id(0, i), id(1, i), id(2, i));
        
        dpdx << I(0,0)*(dpp[0] * pp[2] - dpp[2] * pp[0]),
        I(1,1)*(dpp[1] * pp[2] - dpp[2] * pp[1]);
        dpdx *= 1.0f / zsq;
        
        result.row(start + i) = dpdx;
    }
}

void jacobianEX(Eigen::MatrixX2f& result,
                const Eigen::Vector3f& pp,
                const float& zsq,
                const Eigen::Matrix3Xf& ex,
                const Eigen::Matrix4f& I,
                const Eigen::Matrix3f& R,
                unsigned int start,
                unsigned int size)
{
    assert(ex.cols() >= size);
    assert(result.rows() >= start + size);
    
    Eigen::RowVector2f dpdx;
    // jacobian for identity
    Eigen::Vector3f dpp;
    for (int i = 0; i < size; ++i)
    {
        dpp = R * Eigen::Vector3f(ex(0, i), ex(1, i), ex(2, i));
        
        dpdx << I(0,0)*(dpp[0] * pp[2] - dpp[2] * pp[0]),
        I(1,1)*(dpp[1] * pp[2] - dpp[2] * pp[1]);
        dpdx *= 1.0f / zsq;
        
        result.row(start + i) = dpdx;
    }
}

void jacobianROT(Eigen::MatrixX2f& result,
                 const Eigen::Vector3f& pp,
                 const Eigen::Vector3f& p3,
                 const float& zsq,
                 const Eigen::Matrix4f& I,
                 const std::vector<Eigen::Matrix3f>& RdRs,
                 unsigned int start,
                 unsigned int size)
{
    assert(size <= 3);
    assert(result.rows() >= start + size);
    Eigen::Vector3f RdRp;
    for (int i = 0; i < size; ++i)
    {
        RdRp = RdRs[i] * p3;
        result(start + i, 0) = I(0, 0)*(RdRp[0] * pp[2] - pp[0] * RdRp[2]) / zsq;
        result(start + i, 1) = I(1, 1)*(RdRp[1] * pp[2] - pp[1] * RdRp[2]) / zsq;
    }
}

void jacobianTR(Eigen::MatrixX2f& result,
                const Eigen::Vector3f& pp,
                const Eigen::Vector3f& p3,
                const float& zsq,
                const Eigen::Matrix4f& I,
                const Eigen::Matrix4f& RTc,
                unsigned int start,
                unsigned int size)
{
    assert(size <= 3);
    assert(result.rows() >= start + size);
    
    for (int i = 0; i < size; ++i)
    {
        result(start + i, 0) = I(0, 0) * (RTc(0, i)*pp[2] - pp[0]*RTc(2, i)) / zsq;
        result(start + i, 1) = I(1, 1) * (RTc(1, i)*pp[2] - pp[1]*RTc(2, i)) / zsq;
    }
}

void jacobianCAM(Eigen::MatrixX2f& result,
                 const Eigen::Vector3f& pp,
                 const float& z,
                 const float& scale,
                 unsigned int start,
                 unsigned int size)
{
    assert(size <= 3);
    assert(result.rows() >= start + size);
    
    switch (size)
    {
        case 1:
            result(start + 0, 0) = scale*pp[0] / z;
            result(start + 0, 1) = scale*pp[1] / z;
            break;
        case 2:
            result(start + 0, 0) = scale*pp[0] / z;
            result(start + 0, 1) = 0.0f;
            result(start + 1, 0) = 0.0f;
            result(start + 1, 1) = scale*pp[1] / z;
            break;
        case 3:
            result(start + 0, 0) = scale*pp[0] / z;
            result(start + 0, 1) = 0.0f;
            result(start + 1, 0) = 0.0f;
            result(start + 1, 1) = scale*pp[1] / z;
            result(start + 2, 0) = scale*1.0f;
            result(start + 2, 1) = 0.0f;
            break;
        case 4:
            result(start + 0, 0) = scale*pp[0] / z;
            result(start + 0, 1) = 0.0f;
            result(start + 1, 0) = 0.0f;
            result(start + 1, 1) = scale*pp[1] / z;
            result(start + 2, 0) = scale*1.0f;
            result(start + 2, 1) = 0.0f;
            result(start + 3, 0) = 0.0f;
            result(start + 3, 1) = scale*1.0f;
            break;
        case 0:
            break;
    }
}

// TODO: need to pass cameraID. Otherwise, camera jacobian cannot be computed.
void computeJacobiansP2D(Eigen::MatrixX2f& result,
                         Eigen::Vector2f& p2,
                         const Eigen::Vector3f& p3,
                         const Eigen::Matrix3Xf& id,
                         const Eigen::Matrix3Xf& ex,
                         const Eigen::Matrix4f& RTall,//RTCam*RTFace
                         const Eigen::Matrix4f& RTc,
                         const std::vector<Eigen::Matrix3f>& RdRs,
                         const Eigen::Matrix4f& I,
                         const DOF& dof)
{
	Eigen::Vector3f pp = ApplyTransform(RTall, p3);
	float z = (pp[2] > EPSILON) ? pp[2] : EPSILON;
	p2(0) = I(0, 0) * pp[0] / z + I(0, 2);
	p2(1) = I(1, 1) * pp[1] / z + I(1, 2);

	Eigen::RowVector2f dpdx;
	// jacobian for identity
	Eigen::Vector3f dpp;
	const Eigen::Matrix3f& R = RTall.block<3, 3>(0, 0);
    
    int cur_dof = 0;
    jacobianID(result, pp, z, id, I, R, cur_dof, dof.ID); cur_dof += dof.ID;
    jacobianEX(result, pp, z, ex, I, R, cur_dof, dof.EX); cur_dof += dof.EX; cur_dof += dof.AL;
    jacobianROT(result, pp, p3, z, I, RdRs, cur_dof, dof.ROT); cur_dof += dof.ROT;
    jacobianTR(result, pp, p3, z, I, RTc, cur_dof, dof.TR); cur_dof += dof.TR;
    jacobianCAM(result, pp, z, 10.f, cur_dof, dof.CAM);
}

void computeJacobiansP3D(Eigen::MatrixX3f& result,
                         const Eigen::Vector3f& p3,
                         const Eigen::Matrix3Xf& id,
                         const Eigen::Matrix3Xf& ex,
                         const Eigen::Matrix4f& RTf,
                         const std::vector<Eigen::Matrix3f>& dRs,
                         const DOF& dof)
{
	Eigen::Vector3f pp = ApplyTransform(RTf, p3);

	Eigen::RowVector3f dpdx;
	Eigen::Vector3f dpp;
	// jacobian for identity
	const Eigen::Matrix3f& R = RTf.block<3, 3>(0,0);
	for (int i = 0; i < dof.ID; ++i)
	{
		dpp = R * Eigen::Vector3f(id(0, i), id(1, i), id(2, i));

		result(i, 0) = dpp[0];
		result(i, 1) = dpp[1];
		result(i, 2) = dpp[2];
	}

	// jacobian for expression
	for (int i = 0; i < dof.EX; ++i)
	{
		dpp = R*Eigen::Vector3f(ex(0, i), ex(1, i), ex(2, i));

		result(dof.ID + i, 0) = dpp[0];
		result(dof.ID + i, 1) = dpp[1];
		result(dof.ID + i, 2) = dpp[2];
	}

	// jacobian for rotation
	Eigen::Vector3f dRp;
	for (int i = 0; i < dof.ROT; ++i)
	{
		dRp = dRs[i] * p3;

		result(dof.ID + dof.EX + dof.AL + i, 0) = dRp[0];
		result(dof.ID + dof.EX + dof.AL + i, 1) = dRp[1];
		result(dof.ID + dof.EX + dof.AL + i, 2) = dRp[2];
	}

	// jacobian for translation
	for (int i = 0; i < dof.TR; ++i)
	{
		result(dof.ID + dof.EX + dof.AL + dof.ROT + i, i) = 1.0f;
	}
}

void computeJacobiansF(Eigen::VectorXf& result,
                       const Eigen::Vector3f& v1_v0,
                       const Eigen::Vector3f& v2_v0,
                       const Eigen::Matrix3Xf& i1_i0,
                       const Eigen::Matrix3Xf& i2_i0,
                       const Eigen::Matrix3Xf& e1_e0,
                       const Eigen::Matrix3Xf& e2_e0,
                       const int index0,
                       const int index1,
                       const DOF& dof)
{
	for (int i = 0; i < dof.ID; ++i)
	{
		result[i] = v1_v0[index0] * i2_i0.bc3(i)[index1] + i1_i0.bc3(i)[index0] * v2_v0[index1]
			- v1_v0[index1] * i2_i0.bc3(i)[index0] - i1_i0.bc3(i)[index1] * v2_v0[index0];
	}
	for (int i = 0; i < dof.EX; ++i)
	{
		result[dof.ID + i] = v1_v0[index0] * e2_e0.bc3(i)[index1] + e1_e0.bc3(i)[index0] * v2_v0[index1]
			- v1_v0[index1] * e2_e0.bc3(i)[index0] - e1_e0.bc3(i)[index1] * v2_v0[index0];
	}
}

void computeJacobianPCAReg(Eigen::VectorXf& Jtr,
                           Eigen::MatrixXf& JtJ,
                           const Eigen::VectorXf& X,
                           const Eigen::VectorXf& sigma,
                           unsigned int start,
                           unsigned int size,
                           const float& w)
{
    assert(sigma.size() >= size);
    assert(X.size() == Jtr.size());
    assert(Jtr.size() == JtJ.cols() && Jtr.size() == JtJ.rows());
    assert(X.size() >= start + size);
    
    for (int i = 0; i < size; ++i)
    {
        JtJ(start + i, start + i) += w / sigma[i] / sigma[i];
        Jtr[start + i] += w * X[start + i] / sigma[i] / sigma[i];
    }
}

void computeJacobianTikhonovReg(Eigen::VectorXf& Jtr,
                                Eigen::MatrixXf& JtJ,
                                const Eigen::VectorXf& X,
                                const Eigen::VectorXf& X0,
                                unsigned int start,
                                unsigned int size,
                                const float w)
{
    assert(X.size() >= start + size);
    
    for(int i = start; i < start + size; ++i)
    {
        JtJ(i, i) += w;
        Jtr[i] += w * (X[i] - X0[i]);
    }
}

// NOTE: it assume 68 landmarks (0-17 are on the contour)
// the current contour optimization is to be removed
void computeCorresContourLand2D(std::vector<TriPoint>& pBary,
                                const std::vector<Eigen::Vector3f>& lands,
                                const cv::Mat_<cv::Vec4f>& normal,
                                const cv::Mat_<cv::Vec4f>& baryc,
                                const cv::Mat_<cv::Vec4f>& triIdx,
                                unsigned int level)
{
	pBary.clear();
	//debug_pos.clear();
	const int maxIter = 100;
	// for 0 and 16 we need special care
	auto color = normal.clone();
	Eigen::Vector2f pN;
	Eigen::Vector2f p;
	int failure_count = 0;
	float scale = 1.0f / pow(2.0f, level);
	for (int i = 0; i < 17; ++i)
	{
		if (i == 0){
			p = (lands[i + 1].segment<2>(0) - lands[i].segment<2>(0)).normalized();
			pN(0) = p(1); pN(1) = -p(0);
		}
		else if (i == 16){
			p = (lands[i].segment<2>(0) - lands[i - 1].segment<2>(0)).normalized();
			pN(0) = p(1); pN(1) = -p(0);
		}
		else{
			Eigen::Vector2f p1 = lands[i].segment<2>(0) - lands[i - 1].segment<2>(0);
			Eigen::Vector2f p2 = lands[i + 1].segment<2>(0) - lands[i].segment<2>(0);
			float p1norm = p1.norm(); float p2norm = p2.norm();
			pN(0) = p2norm*p1(1) + p1norm*p2(1); pN(1) = -(p2norm*p1(0) + p1norm*p2(0));
			pN.normalize();
		}

		p = scale * lands[i].segment<2>(0);
		// if the point is out of face area, face should exist towards inside
		int iter = 0;
		if (p(0) < 0.f || p(1) < 0.f || p(0) >= normal.cols || p(1) >= normal.rows){
			pBary.push_back(TriPoint(Eigen::Vector3i(0, 0, 0), Eigen::Vector3f(0.f, 0.f, 0.f), 0.f));
			//debug_pos.push_back(Eigen::Vector2f(-1, -1));
			failure_count++;
			continue;
		}

		if (normal((int)p(1), (int)p(0))[3] != 0.0f){
			pN = -pN;

			for (iter = 0; iter < maxIter; ++iter)
			{
				p += pN;
				if (p(0) < 0.f || p(1) < 0.f || p(0) >= normal.cols || p(1) >= normal.rows) break;
				color((int)p[1], (int)p[0]) = cv::Vec4f(1.0f, 0.0f, 0.0f, 1.0f);
				if (normal((int)p(1), (int)p(0))[3] == 0.0f){
					p -= pN;
					assert(normal((int)p(1), (int)p(0))[3]);
					const auto& t = triIdx((int)p(1), (int)p(0));
					const auto& b = baryc((int)p(1), (int)p(0));
					const auto& n = normal((int)p(1), (int)p(0));
					pBary.push_back(TriPoint(Eigen::Vector3i(t[0] + 0.5f, t[1] + 0.5f, t[2] + 0.5f), Eigen::Vector3f(b[0], b[1], b[2]), std::min(std::max(0.2f, 1.0f - n.dot(cv::Vec4f(0.f, 0.f, -1.f, 0.f))), 1.0f)));
					//debug_pos.push_back(Eigen::Vector2f(p(0), p(1)));
					break;
				}
			}
			if (pBary.size() != i + 1){
				pBary.push_back(TriPoint(Eigen::Vector3i(0, 0, 0), Eigen::Vector3f(0.f, 0.f, 0.f), 0.f));
				//debug_pos.push_back(Eigen::Vector2f(-1, -1));
				failure_count++;
			}
		}
		else{
			for (iter = 0; iter < maxIter; ++iter)
			{
				p += pN;
				if (p(0) < 0.f || p(1) < 0.f || p(0) >= normal.cols || p(1) >= normal.rows) break;
				color((int)p[1], (int)p[0]) = cv::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
				if (normal((int)p(1), (int)p(0))[3] != 0.0f){
					const auto& t = triIdx((int)p(1), (int)p(0));
					const auto& b = baryc((int)p(1), (int)p(0));
					const auto& n = normal((int)p(1), (int)p(0));
					pBary.push_back(TriPoint(Eigen::Vector3i(t[0] + 0.5f, t[1] + 0.5f, t[2] + 0.5f), Eigen::Vector3f(b[0], b[1], b[2]), std::min(std::max(0.2f, 1.0f - n.dot(cv::Vec4f(0, 0, -1.f, 0))), 1.0f)));
					//debug_pos.push_back(Eigen::Vector2f(p(0), p(1)));
					break;
				}
			}
			if (pBary.size() != i + 1){
				pBary.push_back(TriPoint(Eigen::Vector3i(0, 0, 0), Eigen::Vector3f(0.f, 0.f, 0.f), 0.f));
				//debug_pos.push_back(Eigen::Vector2f(-1, -1));
				failure_count++;
			}
		}
	}

	assert(pBary.size() == 17);

	cv::imwrite("test.png", 255.0f*color);

	if (failure_count > 2){
		std::cout << "Error: it couldn't find " << failure_count << " correspondenses." << std::endl;
	}
}

// the current contour optimization is to be removed
void computeJacobianContour(Eigen::VectorXf& Jtr,
                            Eigen::MatrixXf& JtJ,
                            const std::vector<Eigen::Vector2f>& pV,
                            const std::vector<Eigen::MatrixX2f>& dpV,
                            const std::vector<TriPoint>& pBary,
                            const std::vector<Eigen::Vector3f>& land,
                            float w)
{
    assert(pV.size() == dpV.size());
    assert(pBary.size() == land.size());
    
	Eigen::Vector2f p;
	Eigen::MatrixX2f dp;
	
    w *= 1.0f/(float)(land.size());
    for (int i = 0; i < pBary.size(); ++i)
	{
        if(pBary[i].w != 0.f){
            p =  pBary[i].ust[0] * pV[pBary[i].idx[0]] + pBary[i].ust[1] * pV[pBary[i].idx[1]] + pBary[i].ust[2] * pV[pBary[i].idx[2]];
            dp = pBary[i].ust[0] * dpV[pBary[i].idx[0]] + pBary[i].ust[1] * dpV[pBary[i].idx[1]] + pBary[i].ust[2] * dpV[pBary[i].idx[2]];

            Jtr += w * land[i](2) * pBary[i].w * dp * (p - land[i].segment<2>(0));
            JtJ += w * land[i](2) * pBary[i].w * dp * dp.transpose();
        }
	}
}

// TODO: the current implementation is not efficient. dp12 is unchanged. so it'd ideal to precompute them and load it for every iteration.
void computeJacobianSymmetry(Eigen::VectorXf& Jtr,
                             Eigen::MatrixXf& JtJ,
                             const Eigen::VectorXf& id_delta,
                             const Eigen::VectorXf& ex_delta,
                             const Eigen::MatrixXf& w_id,
                             const Eigen::MatrixXf& w_exp,
                             const std::vector< int > sym_list,
                             const DOF& dof,
                             const float& w,
                             bool withexp)
{
	if ((dof.ID == 0 && !withexp) || (dof.ID + dof.EX == 0 && withexp)) return;

	Eigen::Vector3f p12;
	std::vector<Eigen::Matrix3Xf> dp12(sym_list.size() / 2, Eigen::Matrix3Xf::Zero(3, dof.all()));

	for (int i = 0; i < sym_list.size(); i += 2)
	{
		const int& i1 = sym_list[i];
		const int& i2 = sym_list[i + 1];
		const Eigen::Vector3f& p1id = id_delta.b3(i1);
		const Eigen::Vector3f& p2id = id_delta.b3(i2);
        const Eigen::Vector3f& p1ex = ex_delta.b3(i1);
        const Eigen::Vector3f& p2ex = ex_delta.b3(i2);
        
		p12[0] = p1id[0] + p2id[0];
		p12[1] = p1id[1] - p2id[1];
		p12[2] = p1id[2] - p2id[2];

		dp12[i / 2].block(0, 0, 1, dof.ID) = w_id.block(i1 * 3 + 0, 0, 1, dof.ID) + w_id.block(i2 * 3 + 0, 0, 1, dof.ID);
		dp12[i / 2].block(1, 0, 2, dof.ID) = w_id.block(i1 * 3 + 1, 0, 2, dof.ID) - w_id.block(i2 * 3 + 1, 0, 2, dof.ID);
        
        if(withexp){
            p12[0] += p1ex[0] + p2ex[0];
            p12[1] += p1ex[1] - p2ex[1];
            p12[2] += p1ex[2] - p2ex[2];
            
            dp12[i / 2].block(0, dof.ID, 1, dof.EX) = w_exp.block(i1 * 3 + 0, 0, 1, dof.EX) + w_exp.block(i2 * 3 + 0, 0, 1, dof.EX);
            dp12[i / 2].block(1, dof.ID, 2, dof.EX) = w_exp.block(i1 * 3 + 1, 0, 2, dof.EX) - w_exp.block(i2 * 3 + 1, 0, 2, dof.EX);
        }

		Jtr += w * dp12[i / 2].transpose() * p12;
		JtJ += w * dp12[i / 2].transpose() * dp12[i / 2];
	}
}

void computeJacobianPairClose(Eigen::VectorXf& Jtr,
                              Eigen::MatrixXf& JtJ,
                              const Eigen::VectorXf& V,
                              const Eigen::MatrixXf& w_id,
                              const Eigen::MatrixXf& w_exp,
                              const std::vector< int >& index_list,
                              const DOF& dof,
                              const float w)
{
	Eigen::Vector3f p12;
	Eigen::Matrix3Xf dp12 = Eigen::Matrix3Xf::Zero(3, dof.all());
	for (int i = 0; i < index_list.size(); i += 2)
	{
		const int& i1 = index_list[i];
		const int& i2 = index_list[i + 1];
		const Eigen::Vector3f& v1 = V.b3(i1);
		const Eigen::Vector3f& v2 = V.b3(i2);

		p12 = v1 - v2;

		if (dof.ID != 0)
			dp12.block(0, 0, 3, dof.ID) = w_id.block(i1 * 3 + 0, 0, 3, dof.ID) - w_id.block(i2 * 3 + 0, 0, 3, dof.ID);
		if (dof.EX != 0)
			dp12.block(0, dof.ID, 3, dof.EX) = w_exp.block(i1 * 3 + 0, 0, 3, dof.EX) - w_exp.block(i2 * 3 + 0, 0, 3, dof.EX);

		Jtr += w * dp12.transpose() * p12;
		JtJ += w * dp12.transpose() * dp12;
	}
}

float computeJacobianPoint2Point3D(Eigen::VectorXf& Jtr,
                                  Eigen::MatrixXf& JtJ,
                                  const std::vector<Eigen::Vector4f>& pV,
                                  const std::vector<Eigen::MatrixX3f>& dpV,
                                  const std::vector<Eigen::Vector3f>& qV,
                                  const float& w)
{
    float error = 0;
    Eigen::Vector3f pq;
    for(int i = 0; i < pV.size(); ++i)
    {
        pq = pV[i].segment<3>(0)-qV[i];
    
        Jtr += w*pV[i][3]*dpV[i]*pq;
        JtJ += w*pV[i][3]*dpV[i]*dpV[i].transpose();
        
        error += w * pV[i][3] * pq.squaredNorm();
    }
    
    return error;
}

float computeJacobianPoint2Plane3D(Eigen::VectorXf& Jtr,
                                  Eigen::MatrixXf& JtJ,
                                  const std::vector<Eigen::Vector4f>& pV,
                                  const std::vector<Eigen::MatrixX3f>& dpV,
                                  const std::vector<Eigen::Vector3f>& qV,
                                  const std::vector<Eigen::Vector3f>& nV,
                                  const float& w)
{
    float error = 0;
    float npq;
    Eigen::VectorXf ndp;
    for(int i = 0; i < pV.size(); ++i)
    {
        npq = nV[i].dot(pV[i].segment<3>(0)-qV[i]);
        ndp = dpV[i] * nV[i];
        Jtr += w * pV[i][3] * npq * ndp;
        JtJ += w * pV[i][3] * ndp * ndp.transpose();
        
        error += w * pV[i][3] * npq * npq;
    }
    
    return error;
}

float computeJacobianPoint2Point2D(Eigen::VectorXf& Jtr,
                                   Eigen::MatrixXf& JtJ,
                                   const std::vector<Eigen::Vector3f>& pV,
                                   const std::vector<Eigen::MatrixX2f>& dpV,
                                   const std::vector<Eigen::Vector2f>& qV,
                                   const float& w)
{
    float error = 0;
    Eigen::Vector2f pq;
    for(int i = 0; i < pV.size(); ++i)
    {
        pq = pV[i].segment<2>(0)-qV[i];
        
        Jtr += w*pV[i][2]*dpV[i]*pq;
        JtJ += w*pV[i][2]*dpV[i]*dpV[i].transpose();
        
        error += w * pV[i][2] * pq.squaredNorm();
    }
    
    return error;
}

float computeJacobianPoint2Line2D(Eigen::VectorXf& Jtr,
                                  Eigen::MatrixXf& JtJ,
                                  const std::vector<Eigen::Vector3f>& pV,
                                  const std::vector<Eigen::MatrixX2f>& dpV,
                                  const std::vector<Eigen::Vector2f>& qV,
                                  const std::vector<Eigen::Vector2f>& nV,
                                  const float& w)
{
    float error = 0;
    float npq;
    Eigen::VectorXf ndp;
    for(int i = 0; i < pV.size(); ++i)
    {
        npq = nV[i].dot(pV[i].segment<2>(0)-qV[i]);
        ndp = dpV[i] * nV[i];
        Jtr += w * pV[i][2] * npq * ndp;
        JtJ += w * pV[i][2] * ndp * ndp.transpose();
        
        error += w * pV[i][2] * npq * npq;
    }
    
    return error;
}

void computeVertexWiseNormalTerm(std::vector<Eigen::Vector3f>& nV,
                                 std::vector<Eigen::MatrixXf>& dnV,
                                 const Eigen::VectorXf& V,
                                 const Eigen::MatrixX3f& tri,
                                 const std::vector<std::array<Eigen::Matrix3Xf, 2>>& id_edge,
                                 const std::vector<std::array<Eigen::Matrix3Xf, 2>>& ex_edge,
                                 const DOF& dof)
{
	Eigen::Vector3f f;
	std::vector<Eigen::VectorXf> df(3, Eigen::VectorXf::Zero(dof.ID + dof.EX));
	std::vector<Eigen::Vector3f> fV(V.size()/3, Eigen::Vector3f::Zero());
	std::vector<Eigen::MatrixX3f> dfV(V.size()/3, Eigen::MatrixX3f::Zero(dof.ID + dof.EX, 3));
	Eigen::Vector3f v1_v0, v2_v0;
	for (int i = 0; i < tri.rows(); ++i)
	{
        const int& idx0 = tri(i, 0);
		const int& idx1 = tri(i, 1);
		const int& idx2 = tri(i, 2);

		v1_v0 = V.b3(idx1) - V.b3(idx0);
		v2_v0 = V.b3(idx2) - V.b3(idx0);

		computeJacobiansF(df[0], v1_v0, v2_v0, id_edge[i][0], id_edge[i][1], ex_edge[i][0], ex_edge[i][1], 1, 2, dof);
		computeJacobiansF(df[1], v1_v0, v2_v0, id_edge[i][0], id_edge[i][1], ex_edge[i][0], ex_edge[i][1], 2, 0, dof);
		computeJacobiansF(df[2], v1_v0, v2_v0, id_edge[i][0], id_edge[i][1], ex_edge[i][0], ex_edge[i][1], 0, 1, dof);
		f = (v1_v0).cross(v2_v0);

		fV[idx0] += f;
		fV[idx1] += f;
		fV[idx2] += f;

		dfV[idx0].col(0) += df[0];
		dfV[idx1].col(0) += df[0];
		dfV[idx2].col(0) += df[0];

		dfV[idx0].col(1) += df[1];
		dfV[idx1].col(1) += df[1];
		dfV[idx2].col(1) += df[1];

		dfV[idx0].col(2) += df[2];
		dfV[idx1].col(2) += df[2];
		dfV[idx2].col(2) += df[2];
	}

	nV = fV;
	for (int i = 0; i < V.size()/3; ++i)
	{
		float g = fV[i].norm();
		if (g < EPSILON) g = EPSILON;
		dnV[i] = (g*dfV[i] - (1.0f / g * dfV[i] * fV[i])*fV[i].transpose()) / (g*g);

		nV[i] /= g; // compute normal
	}
}

void computeVertexWisePositionGradient2D(std::vector<Eigen::Vector2f>& pV,
                                         std::vector<Eigen::MatrixX2f>& dpV,
                                         const Eigen::VectorXf& V,
                                         const Eigen::MatrixXf& w_id,
                                         const Eigen::MatrixXf& w_exp,
                                         const Eigen::Matrix4f& RTc,
                                         const Eigen::Vector6f& rt,
                                         const Eigen::Matrix4f& I,
                                         const DOF& dof,
                                         const std::vector<int>& vert_list)
{
	if (vert_list.size() == 0){
		if (pV.size() != V.size()/3)
			pV.assign(V.size()/3, Eigen::Vector2f::Zero());
		if (dpV.size() != V.size()/3)
            dpV.assign(V.size()/3, Eigen::MatrixX2f::Zero(dof.all(), 2));
	}
	else{
		if (pV.size() != vert_list.size())
			pV.assign(vert_list.size(), Eigen::Vector2f::Zero());
		if (dpV.size() != vert_list.size())
			dpV.assign(vert_list.size(), Eigen::MatrixX2f::Zero(dof.all(), 2));
	}

	// compute position term per vertex
	Eigen::Matrix4f RTall = RTc*Eigen::EulerAnglesPoseToMatrix(rt);
	const Eigen::Matrix3f& R = RTc.block<3,3>(0, 0);

	std::vector<Eigen::Matrix3f> RdRs(3);
	// Warning: it's not aligned with wiki (https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions)
	// looks like there's weird convenction in euler angle in mLib
	// this part could be problematic when moving to other coordinate system
	float sPhi = sin(rt[0]);
	float cPhi = cos(rt[0]);
	float sTheta = sin(rt[1]);
	float cTheta = cos(rt[1]);
	float sPsi = sin(rt[2]);
	float cPsi = cos(rt[2]);

	RdRs[0] << 0, sPhi*sPsi + cPhi*sTheta*cPsi, cPhi*sPsi - sPhi*sTheta*cPsi,
		0, -sPhi*cPsi + cPhi*sTheta*sPsi, -cPhi*cPsi - sPhi*sTheta*sPsi,
		0, cPhi*cTheta, -sPhi*cTheta;
	RdRs[1] << -sTheta*cPsi, sPhi*cTheta*cPsi, cPhi*cTheta*cPsi,
		-sTheta*sPsi, sPhi*cTheta*sPsi, cPhi*cTheta*sPsi,
		-cTheta, -sPhi*sTheta, -cPhi*sTheta;
	RdRs[2] << -cTheta*sPsi, -cPhi*cPsi - sPhi*sTheta*sPsi, sPhi*cPsi - cPhi*sTheta*sPsi,
		cTheta*cPsi, -cPhi*sPsi + sPhi*sTheta*cPsi, sPhi*sPsi + cPhi*sTheta*cPsi,
		0, 0, 0;

	RdRs[0] = R * RdRs[0];
	RdRs[1] = R * RdRs[1];
	RdRs[2] = R * RdRs[2];

    for (int i = 0; i < vert_list.size(); ++i)
    {
        const Eigen::Matrix3Xf& w_idV = w_id.block(3 * vert_list[i], 0, 3, (dof.ID>0 ? dof.ID : 1));
        const Eigen::Matrix3Xf& w_expV = w_exp.block(3 * vert_list[i], 0, 3, (dof.EX>0 ? dof.EX : 1));
        computeJacobiansP2D(dpV[i], pV[i], V.b3(vert_list[i]), w_idV, w_expV, RTall, RTc, RdRs, I, dof);
    }
}

void computeVertexWisePositionGradient3D(std::vector<Eigen::Vector3f>& pV,
                                         std::vector<Eigen::MatrixX3f>& dpV,
                                         const Eigen::VectorXf& V,
                                         const Eigen::MatrixXf& w_id,
                                         const Eigen::MatrixXf& w_exp,
                                         const Eigen::Vector6f& rt,
                                         const DOF& dof,
                                         const std::vector<int>& vert_list)
{
	if (vert_list.size() == 0){
		if (pV.size() != V.size()/3)
			pV.assign(V.size()/3, Eigen::Vector3f::Zero());
		if (dpV.size() != V.size()/3)
			dpV.assign(V.size()/3, Eigen::MatrixX3f::Zero(dof.all(), 3));
	}
	else{
		if (pV.size() != vert_list.size())
			pV.assign(vert_list.size(), Eigen::Vector3f::Zero());
		if (dpV.size() != vert_list.size())
			dpV.assign(vert_list.size(), Eigen::MatrixX3f::Zero(dof.all(), 3));
	}

	// compute position term per vertex
	Eigen::Matrix4f RTf = Eigen::EulerAnglesPoseToMatrix(rt);
	Eigen::Matrix3f R = RTf.block<3,3>(0, 0);
	Eigen::Vector3f t(RTf(0, 3), RTf(1, 3), RTf(2, 3));

	std::vector<Eigen::Matrix3f> dRs(3);
	// Warning: it's not aligned with wiki (https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions)
	// looks like there's weird convenction in euler angle in mLib
	// this part could be problematic when moving to other coordinate system
	float sPhi = sin(rt[0]);
	float cPhi = cos(rt[0]);
	float sTheta = sin(rt[1]);
	float cTheta = cos(rt[1]);
	float sPsi = sin(rt[2]);
	float cPsi = cos(rt[2]);

	dRs[0] << 0, sPhi*sPsi + cPhi*sTheta*cPsi, cPhi*sPsi - sPhi*sTheta*cPsi,
		0, -sPhi*cPsi + cPhi*sTheta*sPsi, -cPhi*cPsi - sPhi*sTheta*sPsi,
		0, cPhi*cTheta, -sPhi*cTheta;
	dRs[1] << -sTheta*cPsi, sPhi*cTheta*cPsi, cPhi*cTheta*cPsi,
		-sTheta*sPsi, sPhi*cTheta*sPsi, cPhi*cTheta*sPsi,
		-cTheta, -sPhi*sTheta, -cPhi*sTheta;
	dRs[2] << -cTheta*sPsi, -cPhi*cPsi - sPhi*sTheta*sPsi, sPhi*cPsi - cPhi*sTheta*sPsi,
		cTheta*cPsi, -cPhi*sPsi + sPhi*sTheta*cPsi, sPhi*sPsi + cPhi*sTheta*cPsi,
		0, 0, 0;

    for (int i = 0; i < vert_list.size(); ++i)
    {
        pV[i] = R*V.b3(vert_list[i]) + t;
        const Eigen::Matrix3Xf& w_idV = w_id.block(3 * vert_list[i], 0, 3, (dof.ID>0 ? dof.ID : 1));
        const Eigen::Matrix3Xf& w_expV = w_exp.block(3 * vert_list[i], 0, 3, (dof.EX>0 ? dof.EX : 1));
        computeJacobiansP3D(dpV[i], V.b3(vert_list[i]), w_idV, w_expV, RTf, dRs, dof);
    }
}

void setFaceVector(Eigen::VectorXf& X,
                   std::vector<Eigen::Matrix4f>& Is,
                   Eigen::Vector6f& rt,
                   const FaceParams& param,
                   const std::vector<Camera>& cameras,
                   const DOF& dof)
{
	if (X.size() != dof.all()){
		X.resize(dof.all());
	}

	for (int i = 0; i < dof.ID; ++i)
	{
		X[i] = param.idCoeff[i];
	}

	for (int i = 0; i < dof.EX; ++i)
	{
		X[dof.ID + i] = param.exCoeff[i];
	}

	for (int i = 0; i < dof.AL; ++i)
	{
		X[dof.ID + dof.EX + i] = param.alCoeff[i];
	}

	Eigen::ConvertToEulerAnglesPose(param.RT, rt);
	if (dof.ROT == 3 || dof.TR == 3){

		for (int i = 0; i < dof.ROT; ++i)
		{
			X[dof.ID + dof.EX + dof.AL + i] = rt[i];
		}

		for (int i = 0; i < dof.TR; ++i)
		{
			X[dof.ID + dof.EX + dof.AL + dof.ROT + i] = rt[3 + i];
		}
	}

	if(Is.size() != cameras.size()) Is.resize(cameras.size());
	for (int i = 0; i < Is.size(); ++i)
	{
        Is[i] = Eigen::Matrix4f::Identity();
		Is[i].block(0,0,3,3) = cameras[i].intrinsic_;
	}
	
	// for debugging
	const float scale = 0.1f;
	assert(Is.size() == 1);
	switch (dof.CAM){
	case 1:
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0] = scale*Is[0](0, 0);
		break;
	case 2:
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0] = scale*Is[0](0, 0);
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 1] = scale*Is[0](1, 1);
	case 3:
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0] = scale*Is[0](0, 0);
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 1] = scale*Is[0](1, 1);
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 2] = scale*Is[0](0, 2);
		break;
	case 4:
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0] = scale*Is[0](0, 0);
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 1] = scale*Is[0](1, 1);
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 2] = scale*Is[0](0, 2);
		X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 3] = scale*Is[0](1, 2);
		break;
	default:
		break;
	}

	assert(dof.SH == 27 || dof.SH == 9 || dof.SH == 0);
    switch(dof.SH)
    {
        case 27:
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 9; ++j)
                {
                    X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + i * 9 + j] = param.SH[j][i];
                }
            }
            break;
        case 9:
            for (int j = 0; j < 9; ++j)
            {
                X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + j] = (1.f / 3.f)*(param.SH[j][0] + param.SH[j][1] + param.SH[j][2]);
            }
            break;
        default:
            break;
    }
}

void setFaceVector(Eigen::VectorXf& X,
                   Eigen::Matrix4f &I,
                   Eigen::Vector6f& rt,
                   const FaceParams& param,
                   const Camera& camera,
                   const DOF& dof)
{
    std::vector<Eigen::Matrix4f> Is(1, I);
    std::vector<Camera> cameras(1, camera);
    
    setFaceVector(X, Is, rt, param, cameras, dof);
}

void loadFaceVector(const Eigen::VectorXf& X,
                    std::vector<Eigen::Matrix4f>& Is,
                    Eigen::Vector6f& rt,
                    FaceParams& param,
                    std::vector<Camera>& cameras,
                    const DOF& dof)
{
	for (int j = 0; j < dof.ID; ++j)
	{
		param.idCoeff[j] = X[j];
	}

	for (int j = 0; j < dof.EX; ++j)
	{
		param.exCoeff[j] = X[dof.ID + j];
	}

	for (int j = 0; j < dof.AL; ++j)
	{
		param.alCoeff[j] = X[dof.ID + dof.EX + j];
	}

	// update camera
	for (int i = 0; i < dof.ROT; ++i)
	{
		rt[i] = X[dof.ID + dof.EX + dof.AL + i];
	}
	for (int i = 0; i < dof.TR; ++i)
	{
		rt[3 + i] = X[dof.ID + dof.EX + dof.AL + dof.ROT + i];
	}

	if (dof.ROT != 0 || dof.TR != 0){
        param.RT = Eigen::EulerAnglesPoseToMatrix(rt);
	}

	const float scale = 10.0f;
	assert(Is.size() == 1);
	switch (dof.CAM){
	case 1:
		Is[0](0, 0) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0];
		Is[0](1, 1) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0];
		break;
	case 2:
		Is[0](0, 0) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0];
		Is[0](1, 1) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 1];
		break;
	case 3:
		Is[0](0, 0) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0];
		Is[0](1, 1) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 1];
		Is[0](0, 2) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 2];
		break;
	case 4:
		Is[0](0, 0) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 0];
		Is[0](1, 1) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 1];
		Is[0](0, 2) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 2];
		Is[0](1, 2) = scale*X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + 3];
		break;
	default:
		break;
	}
    
    cameras[0].intrinsic_ = Is[0];
    
	assert(dof.SH == 27 || dof.SH == 9 || dof.SH == 0);
    switch(dof.SH)
    {
        case 27:
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 9; ++j)
                {
                    param.SH[j][i] = X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + i * 9 + j];
                }
            }
            break;
        case 9:
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 9; ++j)
                {
                    param.SH[j][i] = X[dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + j];
                }
            }
            break;
        default:
            break;
    }
}

void loadFaceVector(const Eigen::VectorXf& X,
                    Eigen::Matrix4f& I,
                    Eigen::Vector6f& rt,
                    FaceParams& param,
                    Camera& camera,
                    const DOF& dof)
{
    std::vector<Eigen::Matrix4f> Is(1, I);
    std::vector<Camera> cameras(1, camera);
    
    loadFaceVector(X, Is, rt, param, cameras, dof);
    
    I = Is[0];
    camera = cameras[0];
}
