
#include "f2f_optimizer.h"
#include "minitrace.h"

#ifdef WITH_IMGUI
void F2FParams::updateIMGUI()
{
    
}
#endif

// renderTarget contains
// positions,normals,colors,vIndices,vBarycentric,texCoords
// compute jacobian for color consistency term
// | renderTarget(i,j)(x) - inputRGB(i,j) |
void F2FGaussNewtonMultiView(FaceParams& fParam,
                             std::vector<Camera>& cameras,
                             F2FRenderer& renderer,
                             const FaceModel& fModel,
                             const std::vector< cv::Mat_<cv::Vec4f> >& inputRGBs,
                             const std::vector<P2P2DC>& C_P2P,
                             std::vector<P2L2DC>& C_P2L,
                             const std::vector< std::vector<Eigen::Vector3f> >& q2V,
                             unsigned int level,
                             const F2FParams& params,
                             std::shared_ptr<spdlog::logger> logger)
{
    MTR_SCOPE("GaussNewton", "GaussNewtonMultiView");
    assert(q2V.size() == cameras.size());
    assert(inputRGBs.size() == cameras.size());
    
    const DOF& dof = params.dof;
    const int n_vert = fModel.mu_id_.size()/3;
    const Eigen::MatrixX3i& tri = fModel.tri_pts_;
    const std::vector<std::array<Eigen::Matrix3Xf, 2>>& id_edge = fModel.id_edge_;
    const std::vector<std::array<Eigen::Matrix3Xf, 2>>& ex_edge = fModel.ex_edge_;
    const Eigen::MatrixXf& w_id = fModel.w_id_;
    const Eigen::MatrixXf& w_ex = fModel.w_ex_;
    const Eigen::MatrixXf& w_cl = fModel.w_cl_;
    const Eigen::VectorXf& sigma_id = fModel.sigma_id_;
    const Eigen::VectorXf& sigma_ex = fModel.sigma_ex_;
    const Eigen::VectorXf& sigma_cl = fModel.sigma_cl_;
    const Eigen::MatrixX2i& sym_list = fModel.sym_list_;
    
    Eigen::VectorXf X(dof.all());
    Eigen::VectorXf dX(dof.all());
    Eigen::Vector6f rt, rtEx;
    std::vector<Eigen::Matrix4f> Is(cameras.size());
    
    std::vector<cv::Mat> dIxs(cameras.size()), dIys(cameras.size());
    std::vector<cv::Mat_<cv::Vec4f>> renderTarget;
    
    std::vector<Eigen::Vector2f> pV(n_vert);
    std::vector<Eigen::MatrixX2f> dpV(n_vert, Eigen::MatrixX2f::Zero(dof.all(), 2));
    std::vector<Eigen::Vector3f> nV(n_vert);
    std::vector<Eigen::MatrixXf> dnV(n_vert, Eigen::MatrixX3f::Zero(dof.ID + dof.EX, 3));
    Eigen::MatrixXf JtJ = Eigen::MatrixXf::Zero(dof.all(), dof.all());
    Eigen::VectorXf Jtr = Eigen::VectorXf::Zero(dof.all());
    
    std::vector<Eigen::Vector3f> q_p2p;
    std::vector<Eigen::Vector3f> q_p2l;
    std::vector<Eigen::Vector2f> n_p2l;
    std::vector<int> idx_p2p;
    std::vector<int> idx_p2l;
    
    clock_t tm0, tm1;
    tm0 = clock();
    
    // compute image gradient
    if (params.verbose_)
        logger->info("	Computing Image Gradient...");

    MTR_BEGIN("GaussNewton", "cv::Scharr");
    for (int i = 0; i < inputRGBs.size(); ++i)
    {
        cv::Mat tmp;
        cv::cvtColor(inputRGBs[i], tmp, CV_BGRA2BGR);
        
        cv::Scharr(tmp, dIxs[i], CV_32F, 1, 0);
        cv::Scharr(tmp, dIys[i], CV_32F, 0, 1);
    }
    MTR_END("GaussNewton", "cv::Scharr");
    
    tm1 = clock(); logger->info(" img grad: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    tm1 = clock(); logger->info(" setFaceVec: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    // the following part is always the same, so it'd be better to keep it
    if (params.verbose_)
        logger->info("	Computing Edge Basis...");
    
    tm1 = clock(); logger->info(" computeEdgeBasis: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    if (params.verbose_)
        logger->info("	First Color Evaluation + Contour Line Search...");
    
    
    tm1 = clock(); logger->info(" renderFace: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    setFaceVector(X, Is, rt, fParam, cameras, dof);
    
    tm1 = clock(); logger->info(" data/cucg init: ", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    for (int i = 0; i < params.maxIter_[level]; ++i)
    {
        JtJ.setZero();
        Jtr.setZero();
        
        const Eigen::VectorXf& V = fParam.pts_;
        const Eigen::VectorXf& d_ex = fParam.d_ex_;
        const Eigen::VectorXf& d_id = fParam.d_id_;
        
        // render face
        if (params.verbose_)
            logger->info("	Computing Vert-wise Normal and its Gradient...");;
        
        if (params.w_pix_ != 0.f)
            computeVertexWiseNormalTerm(nV, dnV, V, tri, id_edge, ex_edge, dof);
        
        for (int j = 0; j < cameras.size(); ++j)
        {
            const int w = inputRGBs[j].cols;
            const int h = inputRGBs[j].rows;

            const Eigen::Matrix4f& RTc = cameras[j].extrinsic_;
            Eigen::Matrix4f RTEx = RTc * fParam.RT;
            Eigen::ConvertToEulerAnglesPose(RTEx, rtEx);

            if (params.verbose_)
                logger->info("	Rendering Multi-pass for Jacobian Computation...");
            
            renderer.render(w, h, cameras[j], fParam, renderTarget);
            
            const auto& renderRGB = renderTarget[F2FRenderer::RT_NAMES::diffuse];

            if (params.verbose_)
                logger->info("	Computing Vert-wise Position and its Gradient...");
            
            computeVertexWisePositionGradient2D(pV, dpV, V, w_id, w_ex, RTc, rt, Is[j], dof);
            
            P2P2DC::getIndexList(C_P2P, idx_p2p);
            P2L2DC::getIndexList(C_P2L, idx_p2l);
            
            std::vector<Eigen::Vector2f> p_p2l;
            for(int idx : idx_p2l)
                p_p2l.push_back(pV[idx]);
            
            // P2P remains same for the whole iterations. no need to execute here...
            P2P2DC::updateConstraints(C_P2P, q2V[j], q_p2p);
            P2L2DC::updateConstraints(C_P2L, q2V[j], p_p2l, q_p2l, n_p2l);
         
            if (params.verbose_)
                logger->info("	Computing Pixel-wise Color Jacobian...");
            
            if (params.w_pix_ != 0.f)
                F2FRenderer::computeJacobianColor(Jtr, JtJ, w_cl, pV, dpV, nV, dnV, fParam.SH, renderTarget, renderRGB,
                                                  inputRGBs[j], dIxs[j], dIys[j], dof, params.w_pix_);
            
            if (q2V[j].size() != 0){
                // TODO:
                //if ((q2V[j][61] - q2V[j][67]).norm() < params.mc_thresh_ &&
                //    (q2V[j][62] - q2V[j][66]).norm() < params.mc_thresh_ &&
                //    (q2V[j][63] - q2V[j][65]).norm() < params.mc_thresh_)
                //    computeJacobianMouthClose(Jtr, JtJ, faceModel, dof, params.w_mc_);

                if (fabs(Eigen::radiansToDegrees(rtEx[0])) < params.angle_thresh_ &&
                    fabs(Eigen::radiansToDegrees(rtEx[1])) < params.angle_thresh_ &&
                    fabs(Eigen::radiansToDegrees(rtEx[2])) < params.angle_thresh_){
                    if (params.verbose_)
                        logger->info("	Computing Landmark Jacobian...");
                    
                    // compute landmark jacobian
                    computeJacobianPoint2Point2D(Jtr, JtJ, pV, dpV, q_p2p, params.w_landin_, params.robust_, idx_p2p);
                    computeJacobianPoint2Line2D(Jtr, JtJ, pV, dpV, q_p2l, n_p2l, params.w_landcont_, params.robust_, idx_p2l);
                }
            }
            tm1 = clock(); logger->info(" t5: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
        }
        
        tm1 = clock(); logger->info(" camDataSet: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
        
        if (dof.ID)
        {
            // head test debug
            //computeJacobianModel3D(Jtr, JtJ, faceModel, faceModel.GetHeadVerts(), dof, params.w_head_, faceModel.get_head_indices());
            
            computeJacobianSymmetry(Jtr, JtJ, d_id, d_ex, w_id, w_ex, sym_list, dof, params.w_sym_, params.with_exp_);
        }
        if (params.verbose_)
            logger->info("	Computing Regularization Jacobian...");
        
        int cur_pos = 0;
        computeJacobianPCAReg(Jtr, JtJ, X, sigma_id, 0, dof.ID, params.w_reg_pca_id_); cur_pos += dof.ID;
        computeJacobianPCAReg(Jtr, JtJ, X, sigma_ex, cur_pos, dof.EX, params.w_reg_pca_ex_); cur_pos += dof.EX;
        computeJacobianPCAReg(Jtr, JtJ, X, sigma_cl, cur_pos, dof.AL, params.w_reg_pca_cl_);

        // update x
        if (params.verbose_)
            logger->info("	Solving Gauss-Newton Step...");
        
        dX = JtJ.ldlt().solve(Jtr);
        X -= dX;
        
        if (params.verbose_)
            logger->info("	Updating Face...");
        
        loadFaceVector(X, Is, rt, fParam, cameras, dof);
        fParam.updateAll(fModel);
        
        if (params.verbose_)
            logger->info("	Error Evaluation...");
        
        //logger->info("iter: {} E = {} (Eprev-Ecur) = {} |dX| = {} ", i, ErrCur, ErrPrev - ErrCur, dX.norm());
        
        if (dX.norm() < params.gn_thresh_) break;
    }
}

void F2FHierarchicalGaussNewtonMultiView(FaceParams& fParam,
                                         std::vector< Camera >& cameras,
                                         F2FRenderer& renderer,
                                         const FaceModel& faceModel,
                                         const std::vector< cv::Mat_<cv::Vec4f> >& inputRGBs,
                                         const std::vector<P2P2DC>& C_P2P,
                                         std::vector<P2L2DC>& C_P2L,
                                         const std::vector<std::vector<Eigen::Vector3f>>& q2V,
                                         const F2FParams& params,
                                         std::shared_ptr<spdlog::logger> logger )
{
    MTR_SCOPE("GaussNewton", "HierarchicalGaussNewtonMultiView");
    if (params.maxIter_.size() == 0) return;
    
    const int hieLev = params.maxIter_.size();
    // build image hierarchy 
    std::vector< std::vector< cv::Mat_<cv::Vec4f> > > inputHieRGB(hieLev);
    const unsigned int width = inputRGBs[0].cols;
    const unsigned int height = inputRGBs[0].rows;
    
    inputHieRGB[0].resize(inputRGBs.size());
    MTR_BEGIN("GaussNewton", "cv::GaussianBlur");
    for (int i = 0; i < inputRGBs.size(); ++i)
    {
        inputHieRGB[0][i] = inputRGBs[i].clone();
        cv::resize(inputHieRGB[0][i], inputHieRGB[0][i], cv::Size(width, height));
        cv::GaussianBlur(inputHieRGB[0][i], inputHieRGB[0][i], cv::Size(params.smoothLev_ * 2 + 1, params.smoothLev_ * 2 + 1), 0.0f);
    }
    MTR_END("GaussNewton", "cv::GaussianBlur");
    MTR_BEGIN("GaussNewton", "cv::pyrDown");
    for (int i = 1; i < hieLev; ++i)
    {
        inputHieRGB[i].resize(inputRGBs.size());
        for (int j = 0; j < inputRGBs.size(); ++j)
        {
            cv::pyrDown(inputHieRGB[i - 1][j], inputHieRGB[i][j]);
        }
    }
    MTR_END("GaussNewton", "cv::pyrDown");
    for (int i = hieLev - 1; i >= 0; --i)
    {
        F2FGaussNewtonMultiView(fParam, cameras, renderer, faceModel, inputHieRGB[i], C_P2P, C_P2L, q2V, i, params);
    }
}

