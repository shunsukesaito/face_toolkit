
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

inline void calcNormal(Eigen::MatrixX3f& nml,
                       const Eigen::VectorXf& pts,
                       const Eigen::MatrixX3i& tri)
{
    nml = Eigen::MatrixX3f::Zero(pts.size()/3,3);
    for(int i = 0; i < tri.rows(); ++i)
    {
        const Eigen::Vector3f& p0 = pts.b3(tri(i,0));
        const Eigen::Vector3f& p1 = pts.b3(tri(i,1));
        const Eigen::Vector3f& p2 = pts.b3(tri(i,2));
        
        Eigen::Vector3f n = (p1-p0).cross(p2-p0);
        
        nml.row(tri(i,0)) += n;
        nml.row(tri(i,1)) += n;
        nml.row(tri(i,2)) += n;
    }
    
    for(int i = 0; i < nml.rows(); ++i)
    {
        nml.row(i).normalize();
    }
}

inline void calcTangent(Eigen::MatrixX3f& tan,
                        Eigen::MatrixX3f& btan,
                        const Eigen::VectorXf& pts,
                        const Eigen::MatrixX2f& uvs,
                        const Eigen::MatrixX3i& tripts,
                        const Eigen::MatrixX3i& triuv)
{
    assert(tripts.size() == triuv.size());
    
    tan = Eigen::MatrixX3f::Zero(pts.size()/3,3);
    btan = Eigen::MatrixX3f::Zero(pts.size()/3,3);
    
    Eigen::VectorXf count = Eigen::VectorXf::Zero(pts.size()/3);
    for (int i = 0; i < tripts.rows(); i++)
    {
        const Eigen::Vector3f& p0 = pts.b3(tripts(i,0));
        const Eigen::Vector3f& p1 = pts.b3(tripts(i,1));
        const Eigen::Vector3f& p2 = pts.b3(tripts(i,2));
        
        const Eigen::RowVector2f& uv0 = uvs.row(triuv(i,0));
        const Eigen::RowVector2f& uv1 = uvs.row(triuv(i,1));
        const Eigen::RowVector2f& uv2 = uvs.row(triuv(i,2));
        
        Eigen::Matrix<float,3,2> W(3, 2);
        Eigen::Matrix2f UV(2, 2);
        
        W.col(0) = p1-p0;
        W.col(1) = p2-p0;
        
        UV.col(0) = (uv1-uv0).transpose();
        UV.col(1) = (uv2-uv0).transpose();
        
        W = W * UV.inverse();
        
        tan.row(tripts(i,0)) += W.col(0).transpose();
        tan.row(tripts(i,1)) += W.col(0).transpose();
        tan.row(tripts(i,2)) += W.col(0).transpose();
        
        btan.row(tripts(i,0)) += W.col(1).transpose();
        btan.row(tripts(i,1)) += W.col(1).transpose();
        btan.row(tripts(i,2)) += W.col(1).transpose();
        
        count(tripts(i,0)) += 1.f;
        count(tripts(i,1)) += 1.f;
        count(tripts(i,2)) += 1.f;
    }
    tan = tan.array().colwise() / count.array();
    btan = btan.array().colwise() / count.array();
}


namespace Eigen{
	typedef Matrix<float, 6, 1> Vector6f;
	
	inline double radiansToDegrees(double x) {
		return x * (180.0 / M_PI);
	}

	inline Matrix3f eulerAngleToMat(float angleradX, float angleradY, float angleradZ)
	{
		Eigen::Matrix3f Rx, Ry, Rz;
		float sinangleX = std::sin(angleradX);
		float cosangleX = std::cos(angleradX);

		Rx << 1.f, 0, 0,
			0, cosangleX, -sinangleX,
			0, sinangleX, cosangleX;

		float sinangleY = std::sin(angleradY);
		float cosangleY = std::cos(angleradY);

		Ry << cosangleY, 0, sinangleY,
			0, 1.f, 0,
			-sinangleY, 0, cosangleY;

		float sinangleZ = std::sin(angleradZ);
		float cosangleZ = std::cos(angleradZ);
		Rz << cosangleZ, -sinangleZ, 0,
			sinangleZ, cosangleZ, 0,
			0, 0, 1;

		return Rz * Ry * Rx;
	}
    
    inline Vector3f matToEulerAngle(Matrix3f R)
    {
        float eps = (float)0.00001;
        
        float psi, theta, phi; // x,y,z axis angles
        if (std::abs(R(2, 0) - 1) > eps || std::abs(R(2, 0) + 1) > eps) {
            theta = -std::asin(R(2, 0)); // \pi - theta
            float costheta = std::cos(theta);
            psi = std::atan2(R(2, 1) / costheta, R(2, 2) / costheta);
            phi = std::atan2(R(1, 0) / costheta, R(0, 0) / costheta);
        }
        else {
            phi = 0;
            float delta = std::atan2(R(0, 1), R(0, 2));
            if (std::abs(R(2, 0) + 1) > eps) {
                theta = (float)(M_PI / 2.0);
                psi = phi + delta;
            }
            else {
                theta = (float)(-M_PI / 2.0);
                psi = -phi + delta;
            }
        }
        
        return Eigen::Vector3f(psi, theta, phi);
    }

	inline Vector3f ApplyTransform(const Matrix4f& A, const Vector3f& p)
	{
		return A.block<3, 3>(0, 0) * p + A.block<3, 1>(0, 3);
	}
    
    inline Vector2f ApplyProjection(const Matrix4f& K, const Matrix4f& RT, const Vector3f& p)
    {
        Eigen::Vector4f krt_p = K * RT * Eigen::Vector4f(p[0],p[1],p[2],1.0);
        
        return Eigen::Vector2f(krt_p(0)/krt_p(2),krt_p(1)/krt_p(2));
    }

	inline void EulerAnglesPoseToMatrix(const Vector6f& rt, Matrix4f& RT)
	{
		RT.setIdentity();
		RT.block<3, 3>(0, 0) = eulerAngleToMat(rt[0], rt[1], rt[2]);
		RT.block<3, 1>(0, 3) = rt.block<3, 1>(3, 0);
	}

	inline Matrix4f EulerAnglesPoseToMatrix(const Vector6f& rt)
	{
		Matrix4f RT = Matrix4f::Identity();

		RT.setIdentity();
		RT.block<3, 3>(0, 0) = eulerAngleToMat(rt[0], rt[1], rt[2]);
		RT.block<3, 1>(0, 3) = rt.block<3, 1>(3, 0);

		return RT;
	}

	inline void ConvertToEulerAnglesPose(const Matrix4f& RT, Vector6f& rt)
	{
		const Matrix3f& R = RT.block<3, 3>(0, 0);
		const Vector3f& tr = RT.block<3, 1>(0, 3);

		float eps = (float)0.00001;

		float psi, theta, phi; // x,y,z axis angles
		if (std::abs(R(2, 0) - 1) > eps || std::abs(R(2, 0) + 1) > eps) {
			theta = -std::asin(R(2, 0)); // \pi - theta
			float costheta = std::cos(theta);
			psi = std::atan2(R(2, 1) / costheta, R(2, 2) / costheta);
			phi = std::atan2(R(1, 0) / costheta, R(0, 0) / costheta);
		}
		else {
			phi = 0;
			float delta = std::atan2(R(0, 1), R(0, 2));
			if (std::abs(R(2, 0) + 1) > eps) {
				theta = (float)(M_PI / 2.0);
				psi = phi + delta;
			}
			else {
				theta = (float)(-M_PI / 2.0);
				psi = -phi + delta;
			}
		}

		rt << psi, theta, phi, tr[0], tr[1], tr[2];
	}

	inline Vector6f ConvertToEulerAnglesPose(const Matrix4f& RT)
	{
		const Matrix3f& R = RT.block<3,3>(0, 0);
		const Vector3f& tr = RT.block<3,1>(0, 3);

		float eps = (float)0.00001;

		float psi, theta, phi; // x,y,z axis angles
		if (std::abs(R(2, 0) - 1) > eps || std::abs(R(2, 0) + 1) > eps) {
			theta = -std::asin(R(2, 0)); // \pi - theta
			float costheta = std::cos(theta);
			psi = std::atan2(R(2, 1) / costheta, R(2, 2) / costheta);
			phi = std::atan2(R(1, 0) / costheta, R(0, 0) / costheta);
		}
		else {
			phi = 0;
			float delta = std::atan2(R(0, 1), R(0, 2));
			if (std::abs(R(2, 0) + 1) > eps) {
				theta = (float)(M_PI / 2.0);
				psi = phi + delta;
			}
			else {
				theta = (float)(-M_PI / 2.0);
				psi = -phi + delta;
			}
		}

		Eigen::Vector6f rt;

		rt << psi, theta, phi, tr[0], tr[1], tr[2];

		return rt;
	}
    
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
    
}
