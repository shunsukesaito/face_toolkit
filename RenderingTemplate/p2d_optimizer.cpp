#include <fstream>
#include <stdlib.h>
#include "p2d_optimizer.h"
#include "minitrace.h"

#ifdef WITH_IMGUI
void P2DFitParams::updateIMGUI()
{
    if (ImGui::CollapsingHeader("P2DFit Parameters")){
        ImGui::Checkbox("robust", &robust_);
        ImGui::InputInt("maxIter", &maxIter_);
        ImGui::InputInt("DOF ID", &dof.ID);
        ImGui::InputInt("DOF EX", &dof.EX);
        ImGui::InputInt("DOF ROT", &dof.ROT);
        ImGui::InputInt("DOF TR", &dof.TR);
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

void P2DFittingMultiView(FaceParams& fParam,
                         std::vector< Camera >& cameras,
                         const FaceModel& fModel,
                         const std::vector<P2P2DC>& C_P2P,
                         std::vector<P2L2DC>& C_P2L,
                         const std::vector<std::vector<Eigen::Vector3f>>& q2V,
                         const P2DFitParams& params)
{
    MTR_SCOPE("LandmarkFitter", "Landmark2DFittingMultiView");
    const DOF& dof = params.dof;
    
    const Eigen::MatrixXf& w_id = fModel.w_id_;
    const Eigen::MatrixXf& w_ex = fModel.w_ex_;
    const Eigen::VectorXf& sigma_id = fModel.sigma_id_;
    const Eigen::VectorXf& sigma_ex = fModel.sigma_ex_;
    const Eigen::MatrixX2i& sym_list = fModel.sym_list_;
    
    std::vector<Eigen::Matrix4f> Is;
    Eigen::Vector6f rt;
    Eigen::VectorXf X(dof.all());
    Eigen::VectorXf dX(dof.all());
    
    setFaceVector(X, Is, rt, fParam, cameras, dof);
    
    std::vector<Eigen::Vector2f> p_p2p, p_p2l;
    std::vector<Eigen::MatrixX2f> dp_p2p, dp_p2l;
    std::vector<Eigen::Vector3f> V_p2p, V_p2l;
    Eigen::MatrixXf JtJ = Eigen::MatrixXf::Zero(dof.all(), dof.all());
    Eigen::VectorXf Jtr = Eigen::VectorXf::Zero(dof.all());
    
    std::vector<Eigen::Vector3f> q_p2p;
    std::vector<Eigen::Vector3f> q_p2l;
    std::vector<Eigen::Vector2f> n_p2l;
    std::vector<int> idx_p2p;
    std::vector<int> idx_p2l;
    
    for (int i = 0; i < params.maxIter_; ++i)
    {
        JtJ.setZero();
        Jtr.setZero();
        
        float err_p2p = 0.0, err_p2l = 0.0;
        for (int j = 0; j < cameras.size(); ++j)
        {
            const Eigen::Matrix4f& RTc = cameras[j].extrinsic_;
            Eigen::Matrix4f RTall = RTc * fParam.RT;
            Eigen::Vector6f rtall = Eigen::ConvertToEulerAnglesPose(RTall);
            
            if (q2V[j].size() != 0){
                //faceModel.UpdateFaceContour(cameras[j]);
                //std::vector<int> indices = faceModel.get_feature_indices();
                //std::vector<Eigen::Vector3f> q = q2V[j];
                
                // TODO:
                //if ((land[61] - land[67]).norm() < params.mc_thresh_ &&
                //    (land[62] - land[66]).norm() < params.mc_thresh_ &&
                //    (land[63] - land[65]).norm() < params.mc_thresh_)
                //    computeJacobianMouthClose(Jtr, JtJ, faceModel, dof, params.w_mc_);
                P2P2DC::getIndexList(C_P2P, idx_p2p);
                P2L2DC::getIndexList(C_P2L, idx_p2l);
                
                computeV(fParam, fModel, idx_p2p, V_p2p);
                computeV(fParam, fModel, idx_p2l, V_p2l);
                
                computeVertexWisePositionGradient2D(p_p2p, dp_p2p, V_p2p, w_id, w_ex, RTc, rt, Is[j], dof, idx_p2p);
                computeVertexWisePositionGradient2D(p_p2l, dp_p2l, V_p2l, w_id, w_ex, RTc, rt, Is[j], dof, idx_p2l);

                P2P2DC::updateConstraints(C_P2P, q2V[j], q_p2p);
                P2L2DC::updateConstraints(C_P2L, q2V[j], p_p2l, q_p2l, n_p2l);

                // compute landmark jacobian
                err_p2p += computeJacobianPoint2Point2D(Jtr, JtJ, p_p2p, dp_p2p, q_p2p, params.w_p2p_, params.robust_);
                err_p2l += computeJacobianPoint2Line2D(Jtr, JtJ, p_p2l, dp_p2l, q_p2l, n_p2l, params.w_p2l_, params.robust_);
            }
        }
        
        int cur_pos = 0;
        computeJacobianPCAReg(Jtr, JtJ, X, sigma_id, 0, dof.ID, params.w_reg_pca_id_); cur_pos += dof.ID;
        computeJacobianPCAReg(Jtr, JtJ, X, sigma_ex, cur_pos, dof.EX, params.w_reg_pca_ex_); cur_pos += dof.EX;
        
        for(int j = 0; j < Jtr.size(); ++j)
        {
            JtJ(j,j) += 1.e-6;
        }
        
        dX = JtJ.ldlt().solve(Jtr);
        X -= dX;
        
        std::cout << "iter " << i << " errP2P = " << err_p2p << " errP2L = " << err_p2l << "|dX| = " << dX.norm() << std::endl;
        loadFaceVector(X, Is, rt, fParam, cameras, dof);
        
        if (dX.norm() < params.gn_thresh_) break;
    }
}

/*void Landmark2DFittingWithContourMultiView(F2FFaceModel& faceModel,
                                           std::vector<hfm::Camera>& cameras,
                                           render::Renderer& faceModelRenderer,
                                           const std::vector<std::vector<Eigen::Vector3f>>& lands,
                                           const Land2DFitParams& params,
                                           std::shared_ptr<spdlog::logger> logger)
{
    MTR_SCOPE("LandmarkFitter", "Landmark2DFittingWithContourMultiView");
    const DOF& dof = params.dof;
    
    std::vector<Eigen::Matrix4f> Is;
    Eigen::Vector6f rt;
    Eigen::VectorXf X(dof.ID + dof.EX + dof.ROT + dof.TR + dof.CAM);
    Eigen::VectorXf dX(dof.ID + dof.EX + dof.ROT + dof.TR + dof.CAM);
    
    setFaceVector(X, Is, rt, faceModel, cameras, dof);
    
    std::vector<std::vector<TriPoint>> tripoints(cameras.size());
    for (int j = 0; j < cameras.size(); ++j)
    {
        faceModelRenderer.render_f2f(faceModel, cameras[j], render::RenderParams(0, 0, 0, 0, 1, 1, 1), cv::Mat(), logger);
        
        // checking landmark contour
        if (lands[j].size() != 0)
            computeCorresContourLand2D(tripoints[j], lands[j], faceModelRenderer.getNormalsCV(), faceModelRenderer.getvBarycentricCV(), faceModelRenderer.getvIndicesCV(), 0);
    }
    
    std::vector<Eigen::Vector2f> pV(faceModel.get_number_of_vertices());
    std::vector<Eigen::MatrixX2f> dpV(faceModel.get_number_of_vertices(), Eigen::MatrixX2f::Zero(dof.ID + dof.EX + dof.ROT + dof.TR + dof.CAM, 2));
    Eigen::MatrixXf JtJ = Eigen::MatrixXf::Zero(dof.ID + dof.EX + dof.ROT + dof.TR + dof.CAM, dof.ID + dof.EX + dof.ROT + dof.TR + dof.CAM);
    Eigen::VectorXf Jtr = Eigen::VectorXf::Zero(dof.ID + dof.EX + dof.ROT + dof.TR + dof.CAM);
    for (int i = 0; i < params.maxIter_; ++i)
    {
        JtJ.setZero();
        Jtr.setZero();
        
        for (int j = 0; j < cameras.size(); ++j)
        {
            const Eigen::Matrix4f& RTCamera = cameras[j].get_extrinsic();
            computeVertexWisePositionGradient2D(pV, dpV, faceModel, RTCamera, rt, Is[j], dof);
            
            if (lands[j].size() != 0){
                
                if ((lands[j][61] - lands[j][67]).norm() < params.mc_thresh_ &&
                    (lands[j][62] - lands[j][66]).norm() < params.mc_thresh_ &&
                    (lands[j][63] - lands[j][65]).norm() < params.mc_thresh_){
                    computeJacobianMouthClose(Jtr, JtJ, faceModel, dof, params.w_mc_);
                }
                
                std::vector<Eigen::Vector3f> lands_in(lands[j].begin()+17,lands[j].end());
                std::vector<Eigen::Vector3f> lands_cont(lands[j].begin(),lands[j].begin()+17);
                std::vector<int> indices_in(faceModel.get_feature_indices().begin()+17,faceModel.get_feature_indices().end());
                
                computeJacobianLand2D(Jtr, JtJ, pV, dpV, lands_in, params.w_landin_, indices_in);
                // compute landmark jacobian
                computeJacobianContour(Jtr, JtJ, pV, dpV, tripoints[j], lands_cont, params.w_landcont_);
            }
        }
        
        computeJacobianSymmetry(Jtr, JtJ, faceModel, dof, params.w_sym_);
        
        computeJacobianReg(Jtr, JtJ, X, faceModel, dof, params.w_reg_);
        
        dX = JtJ.ldlt().solve(Jtr);
        X -= dX;
        
        logger->info("|dX| = {}", dX.norm());
        loadFaceVector(X, Is, rt, faceModel, cameras, dof);
        
        if (dX.norm() < params.gn_thresh_) break;
    }
}*/

