
#include "f2f_optimizer.h"
#include "minitrace.h"

#ifdef WITH_IMGUI
void F2FParams::updateIMGUI()
{
    if (ImGui::CollapsingHeader("F2F Parameters")){
        ImGui::Checkbox("Run", &run_);
        
        ImGui::Checkbox("verbose", &verbose_);
        ImGui::Checkbox("robust", &robust_);
        ImGui::Checkbox("sym including exp", &sym_with_exp_);
        ImGui::InputIntn("maxIter", &maxIter_[0], maxIter_.size());
        ImGui::InputInt("DOF ID", &dof.ID);
        ImGui::InputInt("DOF EX", &dof.EX);
        ImGui::InputInt("DOF AL", &dof.AL);
        ImGui::InputInt("DOF fROT", &dof.fROT);
        ImGui::InputInt("DOF fTR", &dof.fTR);
        ImGui::InputInt("DOF cROT", &dof.cROT);
        ImGui::InputInt("DOF cTR", &dof.cTR);
        ImGui::InputInt("DOF SH", &dof.SH);
        ImGui::InputInt("DOF CAM", &dof.CAM);
        ImGui::InputFloat("w P2P", &w_p2p_);
        ImGui::InputFloat("w P2L", &w_p2l_);
        ImGui::InputFloat("w P3D", &w_p3d_);
        ImGui::InputFloat("w color", &w_pix_);
        ImGui::InputFloat("w sym", &w_sym_);
        ImGui::InputFloat("w PCA ex", &w_reg_pca_ex_);
        ImGui::InputFloat("w PCA id", &w_reg_pca_id_);
        ImGui::InputFloat("w PCA cl", &w_reg_pca_cl_);
        ImGui::InputInt("smoothLev", &smoothLev_);
        ImGui::InputFloat("GN threshold", &gn_thresh_);
        ImGui::InputFloat("MC threshold", &mclose_thresh_);
    }
}
#endif

struct F2FData
{
    std::vector<cv::Mat_<cv::Vec4f>> renderTarget;
    std::vector<Eigen::Vector2f> pV;
    std::vector<Eigen::Matrix2Xf> dpV;
    std::vector<Eigen::Vector3f> nV;
    std::vector<Eigen::Matrix3Xf> dnV;
    
    F2FData(int nv, const DOF& dof) :
    pV(nv), dpV(nv, Eigen::Matrix2Xf::Zero(2, dof.all())),
    nV(nv), dnV(nv, Eigen::Matrix3Xf::Zero(3, dof.ID + dof.EX)) {}
};

static void computeF2FJacobian(Eigen::VectorXf& Jtr,
                               Eigen::MatrixXf& JtJ,
                               F2FData data,
                               F2FRenderer& renderer,
                               const FaceData& fd,
                               const Camera& camera,
                               const Eigen::Vector6f& rtf,
                               const Eigen::Vector6f& rtc,
                               const cv::Mat_<cv::Vec4f>& inputRGB,
                               const cv::Mat& dIx,
                               const cv::Mat& dIy,
                               const std::vector<Eigen::Vector3f>& q2V,
                               const std::vector<P2P2DC>& CP2P,
                               std::vector<P2L2DC>& CP2L,
                               const F2FParams& params,
                               std::shared_ptr<spdlog::logger> logger)
{
    const DOF& dof = params.dof;
    assert(Jtr.size() == dof.all());
    assert(JtJ.rows() == dof.all());
    assert(JtJ.cols() == dof.all());
    
    const int w = inputRGB.cols;
    const int h = inputRGB.rows;

    if (params.verbose_)
        logger->info("	Computing Vert-wise Position and its Gradient...");
    
    computeVertexWiseGradPosition2D(data.pV, data.dpV, fd, rtc, rtf, camera.intrinsic_, dof);
    
    std::vector<Eigen::Vector2f> p_p2l;
    std::vector<Eigen::Vector3f> q_p2p;
    std::vector<Eigen::Vector3f> q_p2l;
    std::vector<Eigen::Vector2f> n_p2l;
    std::vector<int> idx_p2p;
    std::vector<int> idx_p2l;

    P2P2DC::getIndexList(CP2P, idx_p2p);
    P2L2DC::getIndexList(CP2L, idx_p2l);
    
    for(int idx : idx_p2l)
        p_p2l.push_back(data.pV[idx]);
    
    // P2P remains same for the whole iterations. no need to execute here...
    P2P2DC::updateConstraints(CP2P, q2V, q_p2p);
    P2L2DC::updateConstraints(CP2L, q2V, p_p2l, q_p2l, n_p2l);
    
    if (params.verbose_)
        logger->info("	Rendering Multi-pass for Jacobian Computation...");
    
    if (params.w_pix_ != 0.f)
        renderer.render(w, h, camera, fd, data.renderTarget);
    
    if (params.verbose_)
        logger->info("	Computing Pixel-wise Color Jacobian...");
    
    if (params.w_pix_ != 0.f)
        F2FRenderer::computeJacobianColor(Jtr, JtJ, fd, data.pV, data.dpV, data.nV, data.dnV,
                                          fd.SH, data.renderTarget, inputRGB, dIx, dIy, dof, params.w_pix_);
    
    if (q2V.size() != 0){
        // TODO: add mouth close contraints
        
        if (params.verbose_)
            logger->info("	Computing Landmark Jacobian...");
        
        // compute landmark jacobian
        computeJacobianPoint2Point2D(Jtr, JtJ, data.pV, data.dpV, q_p2p, params.w_p2p_/(float)idx_p2p.size(), params.robust_, idx_p2p);
        computeJacobianPoint2Line2D(Jtr, JtJ, data.pV, data.dpV, q_p2l, n_p2l, params.w_p2l_/(float)idx_p2p.size(), params.robust_, idx_p2l);
    }
}

static void computeRegularizerJacobian(Eigen::VectorXf& Jtr,
                                       Eigen::MatrixXf& JtJ,
                                       const Eigen::VectorXf& X,
                                       const FaceData& fd,
                                       const F2FParams& params,
                                       std::shared_ptr<spdlog::logger> logger)
{
    const DOF& dof = params.dof;

    if (params.verbose_)
        logger->info("	Computing Regularization Jacobian...");
    
    if (dof.ID)
    {
        // TODO: head constraints
        computeJacobianSymmetry(Jtr, JtJ, fd, dof, params.w_sym_, params.sym_with_exp_);
    }
    
    int cur_pos = 0;
    const Eigen::VectorXf& sigma_id = fd.model_->sigmaID();
    const Eigen::VectorXf& sigma_ex = fd.model_->sigmaEX();
    const Eigen::VectorXf& sigma_cl = fd.model_->sigmaCL();
    computeJacobianPCAReg(Jtr, JtJ, X, sigma_id, 0, dof.ID, params.w_reg_pca_id_); cur_pos += dof.ID;
    computeJacobianPCAReg(Jtr, JtJ, X, sigma_ex, cur_pos, dof.EX, params.w_reg_pca_ex_); cur_pos += dof.EX;
    computeJacobianPCAReg(Jtr, JtJ, X, sigma_cl, cur_pos, dof.AL, params.w_reg_pca_cl_);
    
    for(int j = 0; j < Jtr.size(); ++j)
    {
        JtJ(j,j) += 1.e-5;
    }
}

// renderTarget contains
// positions,normals,colors,vIndices,vBarycentric,texCoords
// compute jacobian for color consistency term
// | renderTarget(i,j)(x) - inputRGB(i,j) |
void F2FGaussNewton(FaceData& fd,
                    Camera& camera,
                    F2FRenderer& renderer,
                    const cv::Mat_<cv::Vec4f>& inputRGB,
                    const std::vector<P2P2DC>& CP2P,
                    std::vector<P2L2DC>& CP2L,
                    const std::vector<Eigen::Vector3f>& q2V,
                    unsigned int level,
                    const F2FParams& params,
                    std::shared_ptr<spdlog::logger> logger)
{
    MTR_SCOPE("GaussNewton", "GaussNewtonMultiView");    
    const DOF& dof = params.dof;
    const int n_vert = fd.pts_.size()/3;
    
    Eigen::VectorXf X(dof.all());
    Eigen::VectorXf dX(dof.all());
    Eigen::Vector6f rtf, rtc;
    cv::Mat dIx, dIy;

    F2FData data(n_vert, dof);
    Eigen::MatrixXf JtJ = Eigen::MatrixXf::Zero(dof.all(), dof.all());
    Eigen::VectorXf Jtr = Eigen::VectorXf::Zero(dof.all());
    
    clock_t tm0, tm1;
    tm0 = clock();
    
    // compute image gradient
    if (params.verbose_)
        logger->info("	Computing Image Gradient...");

    MTR_BEGIN("GaussNewton", "cv::Scharr");
    cv::Mat tmp;
    cv::cvtColor(inputRGB, tmp, CV_BGRA2BGR);
    cv::Scharr(tmp, dIx, CV_32F, 1, 0);
    cv::Scharr(tmp, dIy, CV_32F, 0, 1);
    MTR_END("GaussNewton", "cv::Scharr");
    
    tm1 = clock(); if (params.verbose_) logger->info(" img grad: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    tm1 = clock(); if (params.verbose_) logger->info(" setFaceVec: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    if (params.verbose_)
        logger->info("	First Color Evaluation + Contour Line Search...");
    
    tm1 = clock(); if (params.verbose_)logger->info(" renderFace: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    Eigen::Ref<Eigen::VectorXf> Xf = X.segment(0,dof.face());
    Eigen::Ref<Eigen::VectorXf> Xc = X.segment(dof.face(),dof.camera());
    setFaceVector(Xf, rtf, fd, dof);
    setCameraVector(Xc, rtc, camera, dof);
    
    tm1 = clock(); if (params.verbose_) logger->info(" data/cucg init: ", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    
    for (int i = 0; i < params.maxIter_[level]; ++i)
    {
        JtJ.setZero();
        Jtr.setZero();
        
        // render face
        if (params.verbose_) logger->info("	Computing Vert-wise Normal and its Gradient...");;
        
        if (params.w_pix_ != 0.f)
            computeVertexWiseGradNormal(data.nV, data.dnV, fd, dof);
        
        computeF2FJacobian(Jtr, JtJ, data, renderer, fd, camera, rtf, rtc, inputRGB, dIx, dIy, q2V, CP2P, CP2L, params, logger);
        
        tm1 = clock(); if (params.verbose_) logger->info(" t5: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
        
        computeRegularizerJacobian(Jtr, JtJ, X, fd, params, logger);

        // update x
        if (params.verbose_) logger->info("	Solving Gauss-Newton Step...");
        
        dX = JtJ.ldlt().solve(Jtr);
        X -= dX;
        
        if (params.verbose_) logger->info("	Updating Face...");
        
        loadFaceVector(X.segment(0,dof.face()), rtf, fd, dof);
        loadCameraVector(X.segment(dof.face(), dof.camera()), rtc, camera, dof);

        fd.updateAll();
        
        if (params.verbose_) logger->info("	Error Evaluation...");
        
        //logger->info("iter: {} E = {} (Eprev-Ecur) = {} |dX| = {} ", i, ErrCur, ErrPrev - ErrCur, dX.norm());
        
        if (dX.norm() < params.gn_thresh_) break;
    }
}

void F2FHierarchicalGaussNewton(FaceData& fd,
                                Camera& camera,
                                F2FRenderer& renderer,
                                const cv::Mat_<cv::Vec4f>& inputRGB,
                                const std::vector<P2P2DC>& C_P2P,
                                std::vector<P2L2DC>& C_P2L,
                                const std::vector<Eigen::Vector3f>& q2V,
                                const F2FParams& params,
                                std::shared_ptr<spdlog::logger> logger )
{
    MTR_SCOPE("GaussNewton", "HierarchicalGaussNewtonMultiView");
    if (params.maxIter_.size() == 0) return;
    
    const int hieLev = params.maxIter_.size();
    // build image hierarchy 
    std::vector< cv::Mat_<cv::Vec4f> > inputHieRGB(hieLev);
    const unsigned int width = inputRGB.cols;
    const unsigned int height = inputRGB.rows;
    
    MTR_BEGIN("GaussNewton", "cv::GaussianBlur");
    inputHieRGB[0] = inputRGB.clone();
    cv::resize(inputHieRGB[0], inputHieRGB[0], cv::Size(width, height));
    cv::GaussianBlur(inputHieRGB[0], inputHieRGB[0], cv::Size(params.smoothLev_ * 2 + 1, params.smoothLev_ * 2 + 1), 0.0f);
    
    MTR_END("GaussNewton", "cv::GaussianBlur");
    MTR_BEGIN("GaussNewton", "cv::pyrDown");
    for (int i = 1; i < hieLev; ++i)
    {
        cv::pyrDown(inputHieRGB[i - 1], inputHieRGB[i]);
    }
    MTR_END("GaussNewton", "cv::pyrDown");
    for (int i = hieLev - 1; i >= 0; --i)
    {
        F2FGaussNewton(fd, camera, renderer, inputHieRGB[i], C_P2P, C_P2L, q2V, i, params, logger);
    }
}

