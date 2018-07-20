#include "p2d_optimizer.h"

#include <fstream>
#include <stdlib.h>

#include <opencv2/opencv.hpp>

#include <minitrace.h>

#include <utility/str_utils.h>

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

bool P2DFitParams::loadParamFromTxt(std::string file)
{
    std::ifstream fin(file);
    if(!fin.is_open()){
        std::cout << "Warning: failed parsing p2dfit params from " << file << std::endl;
        return false;
    }
    
    std::string label, val;
    while(std::getline(fin, label, ' '))
    {
        if (label.empty())
            continue;
        std::getline(fin, val);
        if(label.find("DOF") != std::string::npos){
            std::vector<int> da = string2arrayi(val);
            if(da.size() != 9)
                continue;
            dof = DOF(da[0],da[1],da[2],da[3],da[4],da[5],da[6],da[7],da[8]);
        }
        if(label.find("maxIter") != std::string::npos){
            maxIter_ = std::stoi(val);
        }
        if(label.find("run") != std::string::npos){
            run_ = bool(std::stoi(val));
        }
        if(label.find("verbose") != std::string::npos){
            verbose_ = bool(std::stoi(val));
        }
        if(label.find("robust") != std::string::npos){
            robust_ = bool(std::stoi(val));
        }
        if(label.find("gn_thresh") != std::string::npos){
            gn_thresh_ = std::stof(val);
        }
        if(label.find("angle_thresh") != std::string::npos){
            angle_thresh_ = std::stof(val);
        }
        if(label.find("mclose_thresh") != std::string::npos){
            mclose_thresh_ = std::stof(val);
        }
        if(label.find("w_reg_pca_id") != std::string::npos){
            w_reg_pca_id_ = std::stof(val);
        }
        if(label.find("w_reg_pca_ex") != std::string::npos){
            w_reg_pca_ex_ = std::stof(val);
        }
        if(label.find("w_p2p") != std::string::npos){
            w_p2p_ = std::stof(val);
        }
        if(label.find("w_p2l") != std::string::npos){
            w_p2l_ = std::stof(val);
        }
    }
    
    return true;
}

bool P2DFitParams::saveParamToTxt(std::string file)
{
    std::ofstream fout(file);
    if(!fout.is_open()){
        std::cout << "Warning: failed writing p2dfit params to " << file << std::endl;
        return false;
    }
    
    fout << "DOF: " << dof.ID << " " << dof.EX << " " << dof.AL << " " << dof.fROT << " ";
    fout << dof.fTR << " " << dof.cROT << " " << dof.cTR << " " << dof.CAM << " " << dof.SH << std::endl;
    fout << "maxIter: " << maxIter_ << std::endl;
    fout << "run: " << run_ << std::endl;
    fout << "verbose: " << verbose_ << std::endl;
    fout << "robust: " << robust_ << std::endl;
    fout << "gn_thresh: " << gn_thresh_ << std::endl;
    fout << "angle_thresh: " << angle_thresh_ << std::endl;
    fout << "mclose_thresh: " << mclose_thresh_ << std::endl;
    fout << "w_reg_pca_id: " << w_reg_pca_id_ << std::endl;
    fout << "w_reg_pca_ex: " << w_reg_pca_ex_ << std::endl;
    fout << "w_p2p: " << w_p2p_ << std::endl;
    fout << "w_p2l: " << w_p2l_ << std::endl;
    
    fout.close();
    
    return true;
}

#ifdef WITH_IMGUI
void P2DFitParams::updateIMGUI()
{
    if (ImGui::CollapsingHeader("P2DFit Parameters")){
        if (ImGui::Button("Load"))
            loadParamFromTxt("p2d.ini");
        if (ImGui::Button("Save"))
            saveParamToTxt("p2d.ini");
        if (ImGui::Button("OneTimeRun"))
            onetime_run_ = true;

        ImGui::Checkbox("Run", &run_);
        ImGui::Checkbox("verbose", &verbose_);
        
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

void P2DGaussNewton(std::vector<FaceData>& fd,
                    std::vector<Camera>& cameras,
                    const MFMVCaptureData& data,
                    const std::vector<P2P2DC>& CP2P,
                    std::vector<P2L2DC>& CP2L,
                    const P2DFitParams& params)
{
    MTR_SCOPE("LandmarkFitter", "Landmark2DFittingMultiView");
    assert(fd.size() == data.frames_.size());
    const DOF& dof = params.dof;
    const int n_frame = data.frames_.size();
    const int n_camera = cameras.size();
    const int dof_all = dof.tvar()*n_frame+dof.ftinv()+dof.camera()*n_camera;
    const int dof_tinv = dof.ftinv()+dof.camera()*n_camera;
    std::vector<Eigen::Vector6f> rtf(n_frame);
    std::vector<Eigen::Vector6f> rtc(n_camera);
    Eigen::VectorXf X(dof_all);
    Eigen::VectorXf dX(dof_all);
    
    {
        Eigen::Ref<Eigen::VectorXf> Xfinv = X.segment(0,dof.ftinv());
        setFaceVector(Xfinv, rtf[0], fd[0], dof);
        for(int i = 0; i < n_camera; ++i)
        {
            Eigen::Ref<Eigen::VectorXf> Xc = X.segment(dof.ftinv()+dof.camera()*i,dof.camera());
            setCameraVector(Xc, rtc[i], cameras[i], dof);
        }
        for(int i = 0; i < n_frame; ++i)
        {
            Eigen::Ref<Eigen::VectorXf> Xf = X.segment(dof_tinv+i*dof.tvar(),dof.tvar());
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
            for(int k = 0; k < n_camera; ++k)
            {
                JtJe.setZero();
                Jtre.setZero();

                computeP2DJacobian(Jtre, JtJe, fd[j], cameras[k], rtf[j], rtc[k], data.frames_[j].val_[k].q2V_, CP2P, CP2L, params, err);
                computeRegularizerJacobian(Jtre, JtJe, X, fd[j], params, 0, dof_tinv+j*dof.tvar(), err);
                
                // block diagonal part
                JtJ.block(0,0,dof.ftinv(),dof.ftinv()) += JtJe.block(0,0,dof.ftinv(),dof.ftinv());
                JtJ.block(dof.ftinv()+k*dof.camera(),dof.ftinv()+k*dof.camera(),dof.camera(),dof.camera()) += JtJe.block(dof.face(),dof.face(),dof.camera(),dof.camera());
                JtJ.block(dof_tinv+j*dof.tvar(),dof_tinv+j*dof.tvar(),dof.tvar(),dof.tvar()) += JtJe.block(dof.ftinv(),dof.ftinv(),dof.tvar(),dof.tvar());
                
                // off diagonal part
                JtJ.block(0,dof.ftinv()+dof.camera()*k,dof.ftinv(),dof.camera()) += JtJe.block(0,dof.face(),dof.ftinv(),dof.camera());
                JtJ.block(dof.ftinv()+dof.camera()*k,0,dof.camera(),dof.ftinv()) += JtJe.block(dof.face(),0,dof.camera(),dof.ftinv());
                JtJ.block(0,dof_tinv+j*dof.tvar(),dof.ftinv(),dof.tvar()) += JtJe.block(0,dof.ftinv(),dof.ftinv(),dof.tvar());
                JtJ.block(dof_tinv+j*dof.tvar(),0,dof.tvar(),dof.ftinv()) += JtJe.block(dof.ftinv(),0,dof.tvar(),dof.ftinv());
                JtJ.block(dof.ftinv()+dof.camera()*k,dof_tinv+j*dof.tvar(),dof.camera(),dof.tvar()) += JtJe.block(dof.face(),dof.ftinv(),dof.camera(),dof.tvar());
                JtJ.block(dof_tinv+j*dof.tvar(),dof.ftinv()+dof.camera()*k,dof.tvar(),dof.camera()) += JtJe.block(dof.ftinv(),dof.face(),dof.tvar(),dof.camera());
                
                Jtr.segment(0, dof.ftinv()) += Jtre.segment(0, dof.ftinv());
                Jtr.segment(dof.ftinv()+dof.camera()*k, dof.camera()) += Jtre.segment(dof.face(), dof.camera());
                Jtr.segment(dof_tinv+j*dof.tvar(), dof.tvar()) += Jtre.segment(dof.ftinv(), dof.tvar());
            }
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
        
        {
            Eigen::Ref<Eigen::VectorXf> Xfinv = X.segment(0,dof.ftinv());
            loadFaceVector(Xfinv, rtf[0], fd[0], dof);
            for(int j = 0; j < n_camera; ++j)
            {
                Eigen::Ref<Eigen::VectorXf> Xc = X.segment(dof.ftinv()+dof.camera()*j,dof.camera());
                loadCameraVector(Xc, rtc[j], cameras[j], dof);
            }
            for(int j = 0; j < n_frame; ++j)
            {
                Eigen::Ref<Eigen::VectorXf> Xf = X.segment(dof_tinv+j*dof.tvar(),dof.tvar());
                loadFaceVector(Xf, rtf[j], fd[j], dof);
            }
        }
        std::cout << i << "thItr " << err << std::endl;
        
        if (dX.norm() < params.gn_thresh_) break;
    }
}
