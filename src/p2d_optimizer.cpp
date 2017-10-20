#include <fstream>
#include <stdlib.h>
#include "p2d_optimizer.h"
#include "minitrace.h"


#ifdef WITH_IMGUI
void P2DFitParams::updateIMGUI()
{
    if (ImGui::CollapsingHeader("P2DFit Parameters")){
        ImGui::Checkbox("Run", &run_);

        ImGui::Checkbox("robust", &robust_);
        ImGui::InputInt("maxIter", &maxIter_);
        ImGui::InputInt("DOF ID", &dof.ID);
        ImGui::InputInt("DOF EX", &dof.EX);
        ImGui::InputInt("DOF fROT", &dof.fROT);
        ImGui::InputInt("DOF fTR", &dof.fTR);
        ImGui::InputInt("DOF cROT", &dof.cROT);
        ImGui::InputInt("DOF cTR", &dof.cTR);
        ImGui::InputInt("DOF CAM", &dof.CAM);
        ImGui::InputFloat("w P2P", &w_p2p_);
        ImGui::InputFloat("w P2L", &w_p2l_);
        ImGui::InputFloat("w PCA ex", &w_reg_pca_ex_);
        ImGui::InputFloat("w PCA id", &w_reg_pca_id_);
        ImGui::InputFloat("GN threshold", &gn_thresh_);
        ImGui::InputFloat("MC threshold", &mclose_thresh_);
        ImGui::InputFloat("Ang threshold", &angle_thresh_);
    }
}
#endif

void compute_rigid_motion(const Eigen::Matrix4f &intrinsic,
						  const std::vector<Eigen::Vector3f> &p3d,
                          const std::vector<Eigen::Vector3f> &q2d,
                          Eigen::Matrix4f &extrinsic)
{
    MTR_SCOPE("LandmarkFitter", "compute_rigid_motion");
    assert(p3d.size() == q2d.size());
    
    std::vector<cv::Point3f> modelPoints;
    for(auto &p : p3d)
        modelPoints.push_back(cv::Point3f(p[0],p[1],p[2]));
    
    std::vector<cv::Point2f> projectedPoints;
    for(auto& p : q2d)
        projectedPoints.push_back(cv::Point2f(p[0],p[1]));
    
    cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            cameraMatrix.at<double>(i,j) = static_cast<double>(intrinsic(i,j));
    
    cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;
    
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat cvR(3,3,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    
    if(!cv::solvePnP(modelPoints, projectedPoints, cameraMatrix, distCoeffs, rvec, tvec)){
        std::cout << "Error: Rigid optimization is failed." << std::endl;
    }
    
    cv::Rodrigues(rvec, cvR);
    
    // transposed
    for (int f=0; f<3; f++)
        for (int c=0; c<3; c++)
            extrinsic(f,c) = static_cast<float>(cvR.at<double>(f,c));
    
	extrinsic(0, 3) = static_cast<float>(tvec.at<double>(0));
	extrinsic(1, 3) = static_cast<float>(tvec.at<double>(1));
	extrinsic(2, 3) = static_cast<float>(tvec.at<double>(2));
}

bool RigidAlignment(const std::vector<Eigen::Vector3f> &q,
                    const std::vector<Eigen::Vector3f> &p,
                    Camera &camera)
{
    MTR_SCOPE("LandmarkFitter", "RigidAlignment");

    compute_rigid_motion(camera.intrinsic_, p, q, camera.extrinsic_);
        
    // if tz < 0, it's fitting face behind the camera
    // so need to convert the coordinate to in front of camera
    if(camera.extrinsic_(2,3) < 0){
		Eigen::Vector3f t;
		Eigen::Matrix3f R;

        t = camera.extrinsic_.block<3,1>(0,3);
        R = camera.extrinsic_.block<3,3>(0,0);

        Eigen::VectorXf RT(6);
        RT.b3(0) = R.eulerAngles(0, 1, 2);
        RT.b3(1) = t;
        RT(0) *= -1.0;
        RT(1) *= -1.0;
        RT(2) += (float)M_PI;
        RT.b3(1) *= -1.0;

		R = Eigen::AngleAxisf(RT[0], Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(RT[1], Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(RT[2], Eigen::Vector3f::UnitZ());

        camera.extrinsic_.block<3,1>(0,3) = RT.segment(3, 3);
        camera.extrinsic_.block<3,3>(0,0) = R;        
    }
  
    return true;
}

struct ErrP2D
{
    float p2p = 0.0;
    float p2l = 0.0;
    float pca_id = 0.0;
    float pca_ex = 0.0;
    
    friend std::ostream& operator<<(std::ostream& os, const ErrP2D& err)
    {
        os << "errTot: " << err.p2p + err.p2l + err.pca_id + err.pca_ex << " errP2P: " << err.p2p << " errP2L: " << err.p2l;
        os << " errPCAID: " << err.pca_id << " errPCAEX: " << err.pca_ex;
        return os;
    }
};

static void computeP2DJacobian(Eigen::VectorXf& Jtr,
                               Eigen::MatrixXf& JtJ,
                               const FaceData& fd,
                               const Camera& camera,
                               const Eigen::Vector6f& rtf,
                               const Eigen::Vector6f& rtc,
                               const std::vector<Eigen::Vector3f>& q2V,
                               const std::vector<P2P2DC>& CP2P,
                               const std::vector<P2L2DC>& CP2L,
                               const P2DFitParams& params,
                               ErrP2D& err)
{
    const DOF& dof = params.dof;
    
    // if this part shows down, the data below can be wrapped
    std::vector<Eigen::Vector2f> p_p2p, p_p2l;
    std::vector<Eigen::Matrix2Xf> dp_p2p, dp_p2l;
    std::vector<Eigen::Vector3f> V_p2p, V_p2l;
    
    std::vector<Eigen::Vector3f> q_p2p;
    std::vector<Eigen::Vector3f> q_p2l;
    std::vector<Eigen::Vector2f> n_p2l;
    std::vector<int> idx_p2p;
    std::vector<int> idx_p2l;
    
    // TODO:
    //faceModel.UpdateFaceContour(cameras[j]);
    
    // TODO:
    //if ((land[61] - land[67]).norm() < params.mc_thresh_ &&
    //    (land[62] - land[66]).norm() < params.mc_thresh_ &&
    //    (land[63] - land[65]).norm() < params.mc_thresh_)
    //    computeJacobianMouthClose(Jtr, JtJ, faceModel, dof, params.w_mc_);
    P2P2DC::getIndexList(CP2P, idx_p2p);
    P2L2DC::getIndexList(CP2L, idx_p2l);
    
    computeV(fd, idx_p2p, V_p2p);
    computeV(fd, idx_p2l, V_p2l);
    
    computeVertexWiseGradPosition2D(p_p2p, dp_p2p, V_p2p, fd, rtc, rtf, camera.intrinsic_, dof, idx_p2p);
    computeVertexWiseGradPosition2D(p_p2l, dp_p2l, V_p2l, fd, rtc, rtf, camera.intrinsic_, dof, idx_p2l);
    
    P2P2DC::updateConstraints(CP2P, q2V, q_p2p);
    P2L2DC::updateConstraints(CP2L, q2V, p_p2l, q_p2l, n_p2l);
    
    // compute landmark jacobian
    err.p2p += computeJacobianPoint2Point2D(Jtr, JtJ, p_p2p, dp_p2p, q_p2p, params.w_p2p_, params.robust_);
    err.p2l += computeJacobianPoint2Line2D(Jtr, JtJ, p_p2l, dp_p2l, q_p2l, n_p2l, params.w_p2l_, params.robust_);
    
}

static void computeRegularizerJacobian(Eigen::VectorXf& Jtr,
                                       Eigen::MatrixXf& JtJ,
                                       const Eigen::VectorXf& X,
                                       const FaceData& fd,
                                       const P2DFitParams& params,
                                       int start_id,
                                       int start_ex,
                                       ErrP2D& err)
{
    const DOF& dof = params.dof;
    
    const Eigen::VectorXf& sigma_id = fd.model_->sigmaID();
    const Eigen::VectorXf& sigma_ex = fd.model_->sigmaEX();
    
    err.pca_id += computeJacobianPCAReg(Jtr, JtJ, X, sigma_id, start_id, dof.ID, params.w_reg_pca_id_);
    err.pca_ex += computeJacobianPCAReg(Jtr, JtJ, X, sigma_ex, start_ex, dof.EX, params.w_reg_pca_ex_);
}

void P2DGaussNewton(FaceData& fd,
                    Camera& camera,
                    const std::vector<P2P2DC>& CP2P,
                    std::vector<P2L2DC>& CP2L,
                    const std::vector<Eigen::Vector3f>& q2V,
                    const P2DFitParams& params)
{
    MTR_SCOPE("LandmarkFitter", "Landmark2DFittingMultiView");
    const DOF& dof = params.dof;
    
    Eigen::Vector6f rtf, rtc;
    Eigen::VectorXf X(dof.all());
    Eigen::VectorXf dX(dof.all());
    
    Eigen::Ref<Eigen::VectorXf> Xf = X.segment(0,dof.face());
    Eigen::Ref<Eigen::VectorXf> Xc = X.segment(dof.face(),dof.camera());
    setFaceVector(Xf, rtf, fd, dof);
    setCameraVector(Xc, rtc, camera, dof);

    Eigen::MatrixXf JtJ = Eigen::MatrixXf::Zero(dof.all(), dof.all());
    Eigen::VectorXf Jtr = Eigen::VectorXf::Zero(dof.all());

    for (int i = 0; i < params.maxIter_; ++i)
    {
        JtJ.setZero();
        Jtr.setZero();
        
        ErrP2D err;
        
        computeP2DJacobian(Jtr, JtJ, fd, camera, rtf, rtc, q2V, CP2P, CP2L, params, err);
        computeRegularizerJacobian(Jtr, JtJ, X, fd, params, 0, dof.ID, err);
        
        for(int j = 0; j < Jtr.size(); ++j)
        {
            JtJ(j,j) += 1.e-5;
        }
        
        dX = JtJ.ldlt().solve(Jtr);
        X -= dX;
        
        std::cout << "iter " << i << " errTot: " << err.p2p + err.p2l << " errP2P:" << err.p2p << " errP2L:" << err.p2l << "|dX|:" << dX.norm() << std::endl;
        
        loadFaceVector(X.segment(0,dof.face()), rtf, fd, dof);
        loadCameraVector(X.segment(dof.face(),dof.camera()), rtc, camera, dof);
                         
        if (dX.norm() < params.gn_thresh_) break;
    }
}

void P2DGaussNewton(std::vector<FaceData>& fd,
                    Camera& camera,
                    const std::vector<P2P2DC>& CP2P,
                    std::vector<P2L2DC>& CP2L,
                    const std::vector<std::vector<Eigen::Vector3f>>& q2V,
                    const P2DFitParams& params)
{
    MTR_SCOPE("LandmarkFitter", "Landmark2DFittingMultiView");
    assert(fd.size() == q2V.size());
    const DOF& dof = params.dof;
    const int n_frame = q2V.size();
    const int dof_all = dof.tvar()*n_frame+dof.tinvar();
    
    std::vector<Eigen::Vector6f> rtf(n_frame);
    Eigen::Vector6f rtc;
    Eigen::VectorXf X(dof_all);
    Eigen::VectorXf dX(dof_all);
   
    {
        Eigen::Ref<Eigen::VectorXf> Xfinvar = X.segment(0,dof.ftinvar());
        setFaceVector(Xfinvar, rtf[0], fd[0], dof);
        Eigen::Ref<Eigen::VectorXf> Xc = X.segment(dof.ftinvar(),dof.camera());
        setCameraVector(Xc, rtc, camera, dof);
        for(int i = 0; i < n_frame; ++i)
        {
            Eigen::Ref<Eigen::VectorXf> Xf = X.segment(dof.tinvar()+i*dof.tvar(),dof.tvar());
            setFaceVector(Xf, rtf[i], fd[i], dof);
        }
    }
    
    // ID, camera parameters, (EX, RT) * #frames
    Eigen::MatrixXf JtJ = Eigen::MatrixXf::Zero(dof_all, dof_all);
    Eigen::VectorXf Jtr = Eigen::VectorXf::Zero(dof_all);
    
    Eigen::MatrixXf JtJe = Eigen::MatrixXf::Zero(dof.all(), dof.all());
    Eigen::VectorXf Jtre = Eigen::VectorXf::Zero(dof.all());
    
    for (int i = 0; i < params.maxIter_; ++i)
    {
        JtJ.setZero();
        Jtr.setZero();
        
        ErrP2D err;
        for(int j = 0; j < n_frame; ++j)
        {
            JtJe.setZero();
            Jtre.setZero();
            
            computeP2DJacobian(Jtre, JtJe, fd[j], camera, rtf[j], rtc, q2V[j], CP2P, CP2L, params, err);
            computeRegularizerJacobian(Jtre, JtJe, X, fd[j], params, 0, dof.tinvar()+j*dof.tvar(), err);

            // block diagonal part
            JtJ.block(0,0,dof.ID,dof.ID) += JtJe.block(0,0,dof.ID,dof.ID);
            JtJ.block(dof.ID,dof.ID,dof.camera(),dof.camera()) += JtJe.block(dof.face(),dof.face(),dof.camera(),dof.camera());
            JtJ.block(dof.tinvar(),dof.tinvar(),dof.tvar(),dof.tvar()) += JtJe.block(dof.ID,dof.ID,dof.tvar(),dof.tvar());
            
            // off diagonal part
            JtJ.block(0,dof.ID,dof.ID,dof.camera()) += JtJe.block(0,dof.face(),dof.ID,dof.camera());
            JtJ.block(dof.ID,0,dof.camera(),dof.ID) += JtJe.block(dof.face(),0,dof.camera(),dof.ID);
            JtJ.block(0,dof.tinvar()+j*dof.tvar(),dof.ID,dof.tvar()) += JtJe.block(0,dof.ID,dof.ID,dof.tvar());
            JtJ.block(dof.tinvar()+j*dof.tvar(),0,dof.tvar(),dof.ID) += JtJe.block(dof.ID,0,dof.tvar(),dof.ID);
            JtJ.block(dof.ID,dof.tinvar()+j*dof.tvar(),dof.camera(),dof.tvar()) += JtJe.block(dof.face(),dof.ID,dof.camera(),dof.tvar());
            JtJ.block(dof.tinvar()+j*dof.tvar(),dof.ID,dof.tvar(),dof.camera()) += JtJe.block(dof.ID,dof.face(),dof.tvar(),dof.camera());
            
            Jtr.segment(0, dof.ID) += Jtre.segment(0, dof.ID);
            Jtr.segment(dof.ID, dof.camera()) += Jtre.segment(dof.face(), dof.camera());
            Jtr.segment(dof.tinvar()+j*dof.tvar(), dof.tvar()) += Jtre.segment(dof.ID, dof.tvar());
        }
        
        for(int j = 0; j < Jtr.size(); ++j)
        {
            JtJ(j,j) += 1.e-5;
        }
        
        Eigen::SparseMatrix<float> JtJsp = JtJ.sparseView();
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> ldlt;
        factorizeLDLTSolver(JtJsp, ldlt);
        dX = ldlt.solve(Jtr);
        X -= dX;
        
        Eigen::Ref<Eigen::VectorXf> Xfinvar = X.segment(0,dof.ftinvar());
        loadFaceVector(Xfinvar, rtf[0], fd[0], dof);
        Eigen::Ref<Eigen::VectorXf> Xc = X.segment(dof.ftinvar(),dof.camera());
        setCameraVector(Xc, rtc, camera, dof);
        for(int j = 0; j < n_frame; ++j)
        {
            Eigen::Ref<Eigen::VectorXf> Xf = X.segment(dof.tinvar()+j*dof.tvar(),dof.tvar());
            setFaceVector(Xf, rtf[j], fd[j], dof);
        }
        
        std::cout << "iter " << i << " " << err << std::endl;

        if (dX.norm() < params.gn_thresh_) break;
    }
}
