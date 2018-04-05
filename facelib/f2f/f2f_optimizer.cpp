
#include "f2f_optimizer.h"

#include <minitrace.h>

#include <utility/str_utils.h>

struct ErrF2F
{
    float p2p = 0.0;
    float p2l = 0.0;
    float pix = 0.0;
    float sym = 0.0;
    float pca_id = 0.0;
    float pca_ex = 0.0;
    float pca_cl = 0.0;
    
    friend std::ostream& operator<<(std::ostream& os, const ErrF2F& err)
    {
        os << "Total: " << err.p2p + err.p2l + err.pix + err.sym + err.pca_id + err.pca_ex + err.pca_cl;
        os << " P2P: " << err.p2p << " P2L: " << err.p2l << " Pix: " << err.pix;
        os << " Sym: " << err.sym << " PCAID: " << err.pca_id << " PCAEX: " << err.pca_ex << " PCACL: " << err.pca_cl;
        return os;
    }
    
    float total(){
        return p2p + p2l + pix + sym + pca_id + pca_ex + pca_cl;
    }
};

bool F2FParams::loadParamFromTxt(std::string file)
{
    std::ifstream fin(file);
    if(!fin.is_open()){
        std::cout << "Warning: failed parsing f2f params from " << file << std::endl;
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
            maxIter_ = string2arrayi(val);
        }
        if(label.find("onetime") != std::string::npos){
            onetime_run_ = bool(std::stoi(val));
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
        if(label.find("sym_with_exp") != std::string::npos){
            sym_with_exp_ = bool(std::stoi(val));
        }
        if(label.find("enable_seg") != std::string::npos){
            enable_seg_ = bool(std::stoi(val));
        }
        if(label.find("smoothLev") != std::string::npos){
            smoothLev_ = std::stoi(val);
        }
        if(label.find("gn_thresh") != std::string::npos){
            gn_thresh_ = std::stof(val);
        }
        if(label.find("mclose_thresh") != std::string::npos){
            mclose_thresh_ = std::stof(val);
        }
        if(label.find("w_pix") != std::string::npos){
            w_pix_ = std::stof(val);
        }
        if(label.find("w_reg_pca_id") != std::string::npos){
            w_reg_pca_id_ = std::stof(val);
        }
        if(label.find("w_reg_pca_ex") != std::string::npos){
            w_reg_pca_ex_ = std::stof(val);
        }
        if(label.find("w_reg_pca_cl") != std::string::npos){
            w_reg_pca_cl_ = std::stof(val);
        }
        if(label.find("w_p2p") != std::string::npos){
            w_p2p_ = std::stof(val);
        }
        if(label.find("w_p2l") != std::string::npos){
            w_p2l_ = std::stof(val);
        }
        if(label.find("w_p3d") != std::string::npos){
            w_p3d_ = std::stof(val);
        }
        if(label.find("w_sym") != std::string::npos){
            w_sym_ = std::stof(val);
        }
    }
    
    return true;
}

bool F2FParams::saveParamToTxt(std::string file)
{
    std::ofstream fout(file);
    if(!fout.is_open()){
        std::cout << "Warning: failed writing f2f params to " << file << std::endl;
        return false;
    }

    fout << "DOF: " << dof.ID << " " << dof.EX << " " << dof.AL << " " << dof.fROT << " ";
    fout << dof.fTR << " " << dof.cROT << " " << dof.cTR << " " << dof.CAM << " " << dof.SH << std::endl;
    fout << "maxIter:";
    for(int i : maxIter_)
    {
        fout << " " << i;
    }
    fout << std::endl;
    fout << "onetime: " << onetime_run_ << std::endl;
    fout << "run: " << run_ << std::endl;
    fout << "verbose: " << verbose_ << std::endl;
    fout << "robust: " << robust_ << std::endl;
    fout << "sym_with_exp: " << sym_with_exp_ << std::endl;
    fout << "enable_seg: " << enable_seg_ << std::endl;
    fout << "smoothLev: " << smoothLev_ << std::endl;
    fout << "gn_thresh: " << gn_thresh_ << std::endl;
    fout << "mclose_thresh: " << mclose_thresh_ << std::endl;
    fout << "w_pix: " << w_pix_ << std::endl;
    fout << "w_reg_pca_id: " << w_reg_pca_id_ << std::endl;
    fout << "w_reg_pca_ex: " << w_reg_pca_ex_ << std::endl;
    fout << "w_reg_pca_cl: " << w_reg_pca_cl_ << std::endl;
    fout << "w_p2p: " << w_p2p_ << std::endl;
    fout << "w_p2l: " << w_p2l_ << std::endl;
    fout << "w_p2d: " << w_p3d_ << std::endl;
    fout << "w_sym: " << w_sym_ << std::endl;
   
    fout.close();
    
    return true;
}

#ifdef WITH_IMGUI
void F2FParams::updateIMGUI()
{
    if (ImGui::CollapsingHeader("F2F Parameters")){        
        if (ImGui::Button("Load"))
            loadParamFromTxt("f2f.ini");
        if (ImGui::Button("Save"))
            saveParamToTxt("f2f.ini");
        if (ImGui::Button("OneTimeRun"))
            onetime_run_ = true;

        ImGui::Checkbox("Run", &run_);
        ImGui::Checkbox("verbose", &verbose_);
        
        ImGui::Checkbox("enable seg", &enable_seg_);
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
    pV(nv), dpV(nv, Eigen::Matrix2Xf::Zero(2, dof.pos())),
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
                               ErrF2F& err,
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
    
    Eigen::Ref<Eigen::VectorXf> Jtr_pos = Jtr.segment(dof.AL, dof.pos());
    Eigen::Ref<Eigen::MatrixXf> JtJ_pos = JtJ.block(dof.AL, dof.AL, dof.pos(), dof.pos());
    
    if (params.verbose_)
        logger->info("	Rendering Multi-pass for Jacobian Computation...");
    
    if (params.w_pix_ != 0.f)
        renderer.render(w, h, camera, fd, data.renderTarget);
    
    if (params.verbose_)
        logger->info("	Computing Pixel-wise Color Jacobian...");
    
    if (params.w_pix_ != 0.f)
        err.pix += F2FRenderer::computeJacobianColor(Jtr, JtJ, fd, data.pV, data.dpV, data.nV, data.dnV,
                                          fd.SH, data.renderTarget, inputRGB, dIx, dIy, dof, params.w_pix_);
    
    if (q2V.size() != 0){
        // TODO: add mouth close contraints
        
        if (params.verbose_)
            logger->info("	Computing Landmark Jacobian...");
        
        // compute landmark jacobian
        err.p2p += computeJacobianPoint2Point2D(Jtr_pos, JtJ_pos, data.pV, data.dpV, q_p2p, params.w_p2p_/(float)idx_p2p.size(), params.robust_, idx_p2p);
        err.p2l += computeJacobianPoint2Line2D(Jtr_pos, JtJ_pos, data.pV, data.dpV, q_p2l, n_p2l, params.w_p2l_/(float)idx_p2p.size(), params.robust_, idx_p2l);
    }
}

static void computeRegularizerJacobian(Eigen::VectorXf& Jtr,
                                       Eigen::MatrixXf& JtJ,
                                       const Eigen::VectorXf& X,
                                       const FaceData& fd,
                                       const F2FParams& params,
                                       ErrF2F& err,
                                       std::shared_ptr<spdlog::logger> logger)
{
    const DOF& dof = params.dof;

    if (params.verbose_)
        logger->info("	Computing Regularization Jacobian...");
    
    if (dof.ID)
    {
        // TODO: head constraints
        
        const int sym_dof = (params.sym_with_exp_ ? dof.ID + dof.EX : dof.ID);
        Eigen::Ref<Eigen::VectorXf> Jtr_shape = Jtr.segment(dof.AL, sym_dof);
        Eigen::Ref<Eigen::MatrixXf> JtJ_shape = JtJ.block(dof.AL, dof.AL, sym_dof, sym_dof);
        
        err.sym += computeJacobianSymmetry(Jtr_shape, JtJ_shape, fd, dof, params.w_sym_, params.sym_with_exp_);
    }
    
    int cur_pos = 0;
    const Eigen::VectorXf& sigma_id = fd.model_->sigmaID();
    const Eigen::VectorXf& sigma_ex = fd.model_->sigmaEX();
    const Eigen::VectorXf& sigma_cl = fd.model_->sigmaCL();
    err.pca_cl += computeJacobianPCAReg(Jtr, JtJ, X, sigma_cl, cur_pos, dof.AL, params.w_reg_pca_cl_); cur_pos += dof.AL;
    err.pca_id += computeJacobianPCAReg(Jtr, JtJ, X, sigma_id, cur_pos, dof.ID, params.w_reg_pca_id_); cur_pos += dof.ID;
    err.pca_ex += computeJacobianPCAReg(Jtr, JtJ, X, sigma_ex, cur_pos, dof.EX, params.w_reg_pca_ex_); cur_pos += dof.EX;
    
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
    
    float err_cur, err_prev = 1.e10;
    for (int i = 0; i < params.maxIter_[level]; ++i)
    {
        JtJ.setZero();
        Jtr.setZero();
        
        ErrF2F err;
        
        // render face
        if (params.verbose_) logger->info("	Computing Vert-wise Normal and its Gradient...");;
        
        if (params.w_pix_ != 0.f)
            computeVertexWiseGradNormal(data.nV, data.dnV, fd, dof);
        
        computeF2FJacobian(Jtr, JtJ, data, renderer, fd, camera, rtf, rtc, inputRGB, dIx, dIy, q2V, CP2P, CP2L, params, err, logger);
        
        tm1 = clock(); if (params.verbose_) logger->info(" t5: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
        
        computeRegularizerJacobian(Jtr, JtJ, X, fd, params, err, logger);

        // update x
        if (params.verbose_) logger->info("	Solving Gauss-Newton Step...");
        
        dX = JtJ.ldlt().solve(Jtr);
        X -= dX;
        
        if (params.verbose_) logger->info("	Updating Face...");
        
        loadFaceVector(X.segment(0,dof.face()), rtf, fd, dof);
        loadCameraVector(X.segment(dof.face(), dof.camera()), rtc, camera, dof);

        fd.updateAll();
        
        if (params.verbose_) logger->info("	Error Evaluation...");
        
        std::cout << "iter " << i << " " << err << " |dX|:" << dX.norm() << std::endl;
        
        err_cur = err.total();
        //logger->info("iter: {} E = {} (Eprev-Ecur) = {} |dX| = {} ", i, ErrCur, ErrPrev - ErrCur, dX.norm());
        
        if (dX.norm() < params.gn_thresh_ || err_cur > err_prev) break;
        err_prev = err_cur;
    }
}

//void F2FGaussNewton(std::vector<FaceData>& fd,
//                    Camera& camera,
//                    F2FRenderer& renderer,
//                    const std::vector<cv::Mat_<cv::Vec4f>>& inputRGB,
//                    const std::vector<P2P2DC>& CP2P,
//                    std::vector<P2L2DC>& CP2L,
//                    const std::vector<std::vector<Eigen::Vector3f>>& q2V,
//                    unsigned int level,
//                    const F2FParams& params,
//                    std::shared_ptr<spdlog::logger> logger)
//{
//    MTR_SCOPE("GaussNewton", "GaussNewtonMultiView");
//    const DOF& dof = params.dof;
//    const int n_vert = fd.pts_.size()/3;
//    
//    Eigen::VectorXf X(dof.all());
//    Eigen::VectorXf dX(dof.all());
//    Eigen::Vector6f rtf, rtc;
//    cv::Mat dIx, dIy;
//    
//    F2FData data(n_vert, dof);
//    Eigen::MatrixXf JtJ = Eigen::MatrixXf::Zero(dof.all(), dof.all());
//    Eigen::VectorXf Jtr = Eigen::VectorXf::Zero(dof.all());
//    
//    clock_t tm0, tm1;
//    tm0 = clock();
//    
//    // compute image gradient
//    if (params.verbose_)
//        logger->info("	Computing Image Gradient...");
//    
//    MTR_BEGIN("GaussNewton", "cv::Scharr");
//    cv::Mat tmp;
//    cv::cvtColor(inputRGB, tmp, CV_BGRA2BGR);
//    cv::Scharr(tmp, dIx, CV_32F, 1, 0);
//    cv::Scharr(tmp, dIy, CV_32F, 0, 1);
//    MTR_END("GaussNewton", "cv::Scharr");
//    
//    tm1 = clock(); if (params.verbose_) logger->info(" img grad: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
//    
//    tm1 = clock(); if (params.verbose_) logger->info(" setFaceVec: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
//    
//    if (params.verbose_)
//        logger->info("	First Color Evaluation + Contour Line Search...");
//    
//    tm1 = clock(); if (params.verbose_)logger->info(" renderFace: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
//    
//    Eigen::Ref<Eigen::VectorXf> Xf = X.segment(0,dof.face());
//    Eigen::Ref<Eigen::VectorXf> Xc = X.segment(dof.face(),dof.camera());
//    setFaceVector(Xf, rtf, fd, dof);
//    setCameraVector(Xc, rtc, camera, dof);
//    
//    tm1 = clock(); if (params.verbose_) logger->info(" data/cucg init: ", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
//    
//    for (int i = 0; i < params.maxIter_[level]; ++i)
//    {
//        JtJ.setZero();
//        Jtr.setZero();
//        
//        // render face
//        if (params.verbose_) logger->info("	Computing Vert-wise Normal and its Gradient...");;
//        
//        if (params.w_pix_ != 0.f)
//            computeVertexWiseGradNormal(data.nV, data.dnV, fd, dof);
//        
//        computeF2FJacobian(Jtr, JtJ, data, renderer, fd, camera, rtf, rtc, inputRGB, dIx, dIy, q2V, CP2P, CP2L, params, logger);
//        
//        tm1 = clock(); if (params.verbose_) logger->info(" t5: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
//        
//        computeRegularizerJacobian(Jtr, JtJ, X, fd, params, logger);
//        
//        // update x
//        if (params.verbose_) logger->info("	Solving Gauss-Newton Step...");
//        
//        dX = JtJ.ldlt().solve(Jtr);
//        X -= dX;
//        
//        if (params.verbose_) logger->info("	Updating Face...");
//        
//        loadFaceVector(X.segment(0,dof.face()), rtf, fd, dof);
//        loadCameraVector(X.segment(dof.face(), dof.camera()), rtc, camera, dof);
//        
//        fd.updateAll();
//        
//        if (params.verbose_) logger->info("	Error Evaluation...");
//        
//        std::cout << "iter " << i << " |dX|:" << dX.norm() << std::endl;
//        
//        //logger->info("iter: {} E = {} (Eprev-Ecur) = {} |dX| = {} ", i, ErrCur, ErrPrev - ErrCur, dX.norm());
//        
//        if (dX.norm() < params.gn_thresh_) break;
//    }
//}

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
    
    MTR_BEGIN("GaussNewton", "cv::GaussianBlur");
    inputHieRGB[0] = inputRGB.clone();
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

