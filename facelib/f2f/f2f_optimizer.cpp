
#include "f2f_optimizer.h"

#include <minitrace.h>
#include <sstream>

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

void createImagePyramidMFMV(const MFMVCaptureData& in, std::vector<MFMVCaptureData>& out, int hieLev, int smoothLev)
{
    out.resize(hieLev);
    out[0].frames_.resize(in.frames_.size());
    for(int i = 0; i < in.frames_.size(); ++i)
    {
        out[0].frames_[i].val_.resize(in.frames_[i].val_.size());
        for(int j = 0; j < in.frames_[i].val_.size(); ++j)
        {
            out[0].frames_[i].val_[j].q2V_ = in.frames_[i].val_[j].q2V_;
            out[0].frames_[i].val_[j].q3V_ = in.frames_[i].val_[j].q3V_;
            auto& img = out[0].frames_[i].val_[j].img_;
            
            cv::Mat tmp;
            cv::cvtColor(in.frames_[i].val_[j].img_, tmp, CV_BGR2RGBA);
            tmp.convertTo(img, CV_32F);
            img *= 1.f / 255.f;
            cv::GaussianBlur(img, img, cv::Size(smoothLev * 2 + 1, smoothLev * 2 + 1), 0.0f);
        }
    }
    
    for(int h = 1; h < hieLev; ++h)
    {
        out[h].frames_.resize(in.frames_.size());
        for(int i = 0; i < in.frames_.size(); ++i)
        {
            out[h].frames_[i].val_.resize(in.frames_[i].val_.size());
            for(int j = 0; j < in.frames_[i].val_.size(); ++j)
            {
                out[h].frames_[i].val_[j].q2V_ = in.frames_[i].val_[j].q2V_;
                out[h].frames_[i].val_[j].q3V_ = in.frames_[i].val_[j].q3V_;
                auto& img = out[h].frames_[i].val_[j].img_;
                cv::pyrDown(out[h-1].frames_[i].val_[j].img_, img);
            }
        }
    }
}

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
        logger->info("  Computing Vert-wise Position and its Gradient...");
    
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
        logger->info("  Rendering Multi-pass for Jacobian Computation...");
    
    if (params.w_pix_ != 0.f)
        renderer.render(w, h, camera, fd, data.renderTarget);
    
    if (params.verbose_)
        logger->info("  Computing Pixel-wise Color Jacobian...");
    
    if (params.w_pix_ != 0.f)
        err.pix += F2FRenderer::computeJacobianColor(Jtr, JtJ, fd, data.pV, data.dpV, data.nV, data.dnV,
                                          fd.SH, data.renderTarget, inputRGB, dIx, dIy, dof, params.w_pix_);
    
    if (q2V.size() != 0){
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
                                       int start_cl,
                                       int start_id,
                                       int start_ex,
                                       ErrF2F& err,
                                       std::shared_ptr<spdlog::logger> logger)
{
    const DOF& dof = params.dof;

    if (params.verbose_)
        logger->info("	Computing Regularization Jacobian...");
    
    if (dof.ID)
    {
        const int sym_dof = (params.sym_with_exp_ ? dof.ID + dof.EX : dof.ID);
        Eigen::Ref<Eigen::VectorXf> Jtr_shape = Jtr.segment(dof.AL, sym_dof);
        Eigen::Ref<Eigen::MatrixXf> JtJ_shape = JtJ.block(dof.AL, dof.AL, sym_dof, sym_dof);
        
        err.sym += computeJacobianSymmetry(Jtr_shape, JtJ_shape, fd, dof, params.w_sym_, params.sym_with_exp_);
    }
    
    int cur_pos = 0;
    const Eigen::VectorXf& sigma_id = fd.model_->sigmaID();
    const Eigen::VectorXf& sigma_ex = fd.model_->sigmaEX();
    const Eigen::VectorXf& sigma_cl = fd.model_->sigmaCL();
    err.pca_cl += computeJacobianPCAReg(Jtr, JtJ, X, sigma_cl, start_cl, dof.AL, params.w_reg_pca_cl_);
    err.pca_id += computeJacobianPCAReg(Jtr, JtJ, X, sigma_id, start_id, dof.ID, params.w_reg_pca_id_);
    err.pca_ex += computeJacobianPCAReg(Jtr, JtJ, X, sigma_ex, start_ex, dof.EX, params.w_reg_pca_ex_);
}

// renderTarget contains
// positions,normals,colors,vIndices,vBarycentric,texCoords
// compute jacobian for color consistency term
// | renderTarget(i,j)(x) - inputRGB(i,j) |
void F2FGaussNewton(std::vector<FaceData>& fd,
                    std::vector<Camera>& cameras,
                    F2FRenderer& renderer,
                    const MFMVCaptureData& data,
                    const std::vector<P2P2DC>& CP2P,
                    std::vector<P2L2DC>& CP2L,
                    unsigned int level,
                    const F2FParams& params,
                    std::shared_ptr<spdlog::logger> logger)
{
    std::cout << "Start Optimizing at level " << level << std::endl;
    MTR_SCOPE("GaussNewton", "GaussNewtonMultiView");
    assert(fd.size() == data.frames_.size());
    const DOF& dof = params.dof;
    const int n_vert = fd[0].pts_.size()/3;
    
    const int n_frame = data.frames_.size();
    const int n_camera = cameras.size();
    const int dof_all = dof.tvar()*n_frame+dof.ftinv()+dof.camera()*n_camera;
    const int dof_tinv = dof.ftinv()+dof.camera()*n_camera;
    std::vector<Eigen::Vector6f> rtf(n_frame);
    std::vector<Eigen::Vector6f> rtc(n_camera);
    Eigen::VectorXf X(dof_all);
    Eigen::VectorXf dX(dof_all);

    F2FData f2fdata(n_vert, dof);
    clock_t tm0, tm1;
    tm0 = clock();
    {
        Eigen::Ref<Eigen::VectorXf> Xfinvar = X.segment(0,dof.ftinv());
        setFaceVector(Xfinvar, rtf[0], fd[0], dof);
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
    tm1 = clock(); if (params.verbose_) logger->info(" setFaceVec: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;

    // ID, camera parameters, (EX, RT) * #frames
    Eigen::MatrixXf JtJ = Eigen::MatrixXf::Zero(dof_all, dof_all);
    Eigen::VectorXf Jtr = Eigen::VectorXf::Zero(dof_all);
    
    Eigen::MatrixXf JtJe = Eigen::MatrixXf::Zero(dof.all(), dof.all());
    Eigen::VectorXf Jtre = Eigen::VectorXf::Zero(dof.all());


    
    // compute image gradient
    MTR_BEGIN("GaussNewton", "cv::Scharr");
    std::vector<std::vector<cv::Mat>> dIx(n_frame), dIy(n_frame);
    for(int i = 0; i < n_frame; ++i)
    {
        dIx[i].resize(n_camera);
        dIy[i].resize(n_camera);
        for(int j = 0; j < n_camera; ++j)
        {
            cv::Mat tmp;
            cv::cvtColor(data.frames_[i].val_[j].img_, tmp, CV_BGRA2BGR);
            cv::Scharr(tmp, dIx[i][j], CV_32F, 1, 0);
            cv::Scharr(tmp, dIy[i][j], CV_32F, 0, 1);
        }
    }
    tm1 = clock(); if (params.verbose_) logger->info(" img grad: {}", (float)(tm1 - tm0) / (float)CLOCKS_PER_SEC); tm0 = tm1;
    MTR_END("GaussNewton", "cv::Scharr");
    
    
    float err_cur, err_prev = 1.e10;
    for (int i = 0; i < params.maxIter_[level]; ++i)
    {
        JtJ.setZero();
        Jtr.setZero();
        
        ErrF2F err;
        // render face
        if (params.verbose_) logger->info("  Computing Vert-wise Normal and its Gradient...");;

        for(int j = 0; j < n_frame; ++j)
        {
            if (params.w_pix_ != 0.f)
                computeVertexWiseGradNormal(f2fdata.nV, f2fdata.dnV, fd[j], dof);
            for(int k = 0; k < n_camera; ++k)
            {
                auto& img = data.frames_[j].val_[k].img_;
                auto& p = data.frames_[j].val_[k].q2V_;
                computeF2FJacobian(Jtre, JtJe, f2fdata, renderer, fd[j], cameras[k], rtf[j], rtc[k], img, dIx[j][k], dIy[j][k], p, CP2P, CP2L, params, err, logger);
                
                computeRegularizerJacobian(Jtre, JtJe, X, fd[j], params, 0, dof.ID, dof_tinv+j*dof.tvar(), err, logger);
                
                // inserting JtJ block diagonal part
                JtJ.block(0,0,dof.ftinv(),dof.ftinv()) += JtJe.block(0,0,dof.ftinv(),dof.ftinv());
                JtJ.block(dof.ftinv()+k*dof.camera(),dof.ftinv()+k*dof.camera(),dof.camera(),dof.camera()) += JtJe.block(dof.face(),dof.face(),dof.camera(),dof.camera());
                JtJ.block(dof_tinv+j*dof.tvar(),dof_tinv+j*dof.tvar(),dof.tvar(),dof.tvar()) += JtJe.block(dof.ftinv(),dof.ftinv(),dof.tvar(),dof.tvar());
                
                // inserting JtJ off diagonal part
                JtJ.block(0,dof.ftinv()+k*dof.camera(),dof.ftinv(),dof.camera()) += JtJe.block(0,dof.face(),dof.ftinv(),dof.camera());
                JtJ.block(dof.ftinv()+k*dof.camera(),0,dof.camera(),dof.ftinv()) += JtJe.block(dof.face(),0,dof.camera(),dof.ftinv());
                JtJ.block(0,dof_tinv+j*dof.tvar(),dof.ftinv(),dof.tvar()) += JtJe.block(0,dof.ftinv(),dof.ftinv(),dof.tvar());
                JtJ.block(dof_tinv+j*dof.tvar(),0,dof.tvar(),dof.ftinv()) += JtJe.block(dof.ftinv(),0,dof.tvar(),dof.ftinv());
                JtJ.block(dof.ftinv()+k*dof.camera(),dof_tinv+j*dof.tvar(),dof.camera(),dof.tvar()) += JtJe.block(dof.face(),dof.ftinv(),dof.camera(),dof.tvar());
                JtJ.block(dof_tinv+j*dof.tvar(),dof.ftinv()+k*dof.camera(),dof.tvar(),dof.camera()) += JtJe.block(dof.ftinv(),dof.face(),dof.tvar(),dof.camera());
                
                // inserting Jtr
                Jtr.segment(0, dof.ftinv()) += Jtre.segment(0, dof.ftinv());
                Jtr.segment(dof.ftinv()+k*dof.camera(), dof.camera()) += Jtre.segment(dof.face(), dof.camera());
                Jtr.segment(dof_tinv+j*dof.tvar(), dof.tvar()) += Jtre.segment(dof.ftinv(), dof.tvar());
            }
        }
        
        for(int j = 0; j < Jtr.size(); ++j)
        {
            JtJ(j,j) += 1.e-5;
        }
        
        // update x
        if (params.verbose_) logger->info("  Solving Gauss-Newton Step...");
        Eigen::SparseMatrix<float> JtJsp = JtJ.sparseView();
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> ldlt;
        factorizeLDLTSolver(JtJsp, ldlt);
        dX = ldlt.solve(Jtr);
        X -= dX;
        
        if (params.verbose_) logger->info("  Updating Face...");
        {
            Eigen::Ref<Eigen::VectorXf> Xfinvar = X.segment(0,dof.ftinv());
            loadFaceVector(Xfinvar, rtf[0], fd[0], dof);
            for(int j = 0; j < n_camera; ++j)
            {
                Eigen::Ref<Eigen::VectorXf> Xc = X.segment(dof.ftinv()+j*dof.camera(),dof.camera());
                loadCameraVector(Xc, rtc[j], cameras[j], dof);
            }
            for(int j = 0; j < n_frame; ++j)
            {
                Eigen::Ref<Eigen::VectorXf> Xf = X.segment(dof_tinv+j*dof.tvar(),dof.tvar());
                loadFaceVector(Xf, rtf[j], fd[j], dof);
                fd[j].updateAll();
            }
        }

        if (params.verbose_){
            std::stringstream ss;
            ss << i << " iter " << err << " |dX|:" << dX.norm();
            logger->info(ss.str());
        }
        else
            std::cout << i << " iter " << err << " |dX|:" << dX.norm() << std::endl;
        
        err_cur = err.total();
        if (dX.norm() < params.gn_thresh_ || err_cur > err_prev) break;
        err_prev = err_cur;
    }
}

void F2FHierarchicalGaussNewton(std::vector<FaceData>& fd,
                                std::vector<Camera>& cameras,
                                F2FRenderer& renderer,
                                const MFMVCaptureData& data,
                                const std::vector<P2P2DC>& C_P2P,
                                std::vector<P2L2DC>& C_P2L,
                                const F2FParams& params,
                                std::shared_ptr<spdlog::logger> logger)
{
    MTR_SCOPE("GaussNewton", "HierarchicalGaussNewtonMultiView");
    if (params.maxIter_.size() == 0) return;
    
    const int hieLev = params.maxIter_.size();
    
    std::vector<MFMVCaptureData> dataHie;
    createImagePyramidMFMV(data, dataHie, hieLev, params.smoothLev_);

    MTR_END("GaussNewton", "cv::pyrDown");
    for (int i = hieLev - 1; i >= 0; --i)
    {
        F2FGaussNewton(fd, cameras, renderer, dataHie[i], C_P2P, C_P2L, i, params, logger);
    }
}

void F2FOptimizer::init(std::string data_dir, FaceModelPtr fm)
{
    fm_ = fm;
    param_.dof.ID = std::min(param_.dof.ID, fm_->n_id());
    param_.dof.EX = std::min(param_.dof.EX, fm_->n_exp());
    param_.dof.AL = std::min(param_.dof.AL, fm_->n_clr());

    // OpenGL cannot share context across different threads...
    // TODO: multi-thread use only
    {
        glfwWindowHint(GLFW_VISIBLE, false);
        glfwWindowHint(GLFW_FOCUSED, false);
        auto window = Window(1, 1, 1, "F2F Window");
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
    }
    
    renderer_.init(data_dir, data_dir + "shaders", fm_);
}

void F2FOptimizer::solve(FaceResult &result)
{
    // param check (especially dof)
    {
        param_.dof.ID = std::max(0,std::min((int)result.fd[0].idCoeff.size(),param_.dof.ID));
        param_.dof.EX = std::max(0,std::min((int)result.fd[0].exCoeff.size(),param_.dof.EX));
        param_.dof.AL = std::max(0,std::min((int)result.fd[0].alCoeff.size(),param_.dof.AL));
    }
    
    if((param_.run_ || param_.onetime_run_) &&
       result.fd[0].idCoeff.size() != 0 && result.fd[0].exCoeff.size() != 0 && result.fd[0].alCoeff.size() != 0){
        
        // update segmentation mask
        // TODO: support multi-view/multi-frame for segmentation update
        if(!result.cap_data[0][0].seg_.empty())
            renderer_.updateSegment(result.cap_data[0][0].seg_);

        F2FHierarchicalGaussNewton(result.fd, result.cameras, renderer_, result.cap_data, result.c_p2p, result.c_p2l, param_, logger_);
        
        result.processed_ = true;
        if(param_.onetime_run_) param_.onetime_run_ = false;
    }
}

#ifdef WITH_IMGUI
void F2FOptimizer::updateIMGUI()
{
    param_.updateIMGUI();
}
#endif

OptimizerHandle F2FOptimizer::Create(std::string name, bool run)
{
    auto opt = new F2FOptimizer(name);
    
    opt->param_.loadParamFromTxt("f2f.ini");
    opt->param_.run_ = run;
    
    return OptimizerHandle(opt);
}

