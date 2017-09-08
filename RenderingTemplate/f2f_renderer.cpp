#include "f2f_renderer.hpp"



void F2FRenderer::computeJacobianColor(Eigen::VectorXf& Jtr,
                                       Eigen::MatrixXf& JtJ,
                                       const Eigen::MatrixXf& w_al,
                                       const std::vector<cv::Mat_<cv::Vec3f>>& w_al_uv,
                                       const std::vector<Eigen::Vector2f>& pV,
                                       const std::vector<Eigen::MatrixX2f>& dpV,
                                       const std::vector<Eigen::Vector3f>& nV,
                                       const std::vector<Eigen::MatrixXf>& dnV,
                                       const Eigen::Vector3f* shCoeff,
                                       const std::vector< cv::Mat_<cv::Vec4f> >& renderTarget,
                                       const cv::Mat_<cv::Vec4f>& renderRGB,
                                       const cv::Mat_<cv::Vec4f>& inputRGB,
                                       const cv::Mat_<cv::Vec3f>& dIx,
                                       const cv::Mat_<cv::Vec3f>& dIy,
                                       const DOF& dof,
                                       const float w,
                                       bool tex_mode)
{
    // compute gradient for each pixel
    // since the interpolated normal is normalized again, compute jacobian based on normalized normal
    const float c1 = 0.429043f, c2 = 0.511664f, c3 = 0.743125f, c4 = 0.886227f, c5 = 0.247708f;
    
    float tex_w = (float)w_al_uv[0].cols;
    float tex_h = (float)w_al_uv[0].rows;
    
    Eigen::MatrixXf JtJc = Eigen::MatrixXf::Zero(dof.all(), dof.all());
    
    Eigen::MatrixXf Jtrc = Eigen::MatrixXf::Zero(dof.all(), 1);
    Eigen::Vector3f n;
    Eigen::Vector3f CI;
    Eigen::MatrixXf dn = Eigen::MatrixXf::Zero(dof.ID + dof.EX, 3);
    Eigen::MatrixXf dcdi = Eigen::MatrixXf::Zero(dof.all(), 3);
    Eigen::MatrixXf dp = Eigen::MatrixXf::Zero(dof.all(), 2);
    Eigen::MatrixXf di = Eigen::MatrixXf::Zero(dof.all(), 3);
    Eigen::MatrixXf a = Eigen::MatrixXf::Zero(3, dof.AL);
    Eigen::Matrix<float, 2, 3> dIxy;
    
    int idx0, idx1, idx2;
    int count = 0;
    
    std::vector<Eigen::MatrixX3f> dcdis;
    std::vector<Eigen::Vector3f> CIs;
    std::vector<Eigen::Vector2f> renderPixelPos;
    std::vector<Eigen::VectorXf> Jtrcs;
    
    //#pragma omp parallel for
    const unsigned int width = renderTarget[RT_NAMES::positions].cols;
    const unsigned int height = renderTarget[RT_NAMES::positions].rows;
    for (unsigned int j = 1; j < height - 1; ++j)
    {
        for (unsigned int i = 1; i < width - 1; ++i)
        {
            const auto& c = renderRGB(j, i);
            const auto& al = renderTarget[RT_NAMES::colors](j, i);
            // if it's face region
            // NOTE: we probably wanna consider facial segmentation later on.
            if (renderTarget[RT_NAMES::diffuse](j, i)[3] != 0.0){
                
                dcdi.setZero();
                
                // render target
                const auto& b = renderTarget[RT_NAMES::vBarycentric](j, i);
                const auto& tri_indf = renderTarget[RT_NAMES::vIndices](j, i);
                int tri_ind[3];
                tri_ind[0] = static_cast<int>(tri_indf[0] + 0.5f);
                tri_ind[1] = static_cast<int>(tri_indf[1] + 0.5f);
                tri_ind[2] = static_cast<int>(tri_indf[2] + 0.5f);
                
                idx0 = tri_ind[0];
                idx1 = tri_ind[1];
                idx2 = tri_ind[2];
                
                const Eigen::Vector3f& n0 = nV[idx0];
                const Eigen::Vector3f& n1 = nV[idx1];
                const Eigen::Vector3f& n2 = nV[idx2];
                
                const Eigen::MatrixXf& dn0 = dnV[idx0];
                const Eigen::MatrixXf& dn1 = dnV[idx1];
                const Eigen::MatrixXf& dn2 = dnV[idx2];
                
                n  = b[0]*n0 + b[1]*n1 + b[2]*n2;
                dn = b[0]*dn0 + b[1]*dn1 + b[2]*dn2;
                
                // normalized version of normal derivative
                float n_norm = n.norm();
                if (n_norm < EPSILON) n_norm = EPSILON;
                dn = (n_norm*dn + (1.0f / n_norm * (dn * n)) * n.transpose()) / (n_norm * n_norm);
                n /= n_norm;
                
                // for R, G, B
                if (dof.ID + dof.EX != 0){
                    dcdi.block(0, 0, dof.ID + dof.EX, 1) =
                    2.0f * al[0] * (c2 * shCoeff[3][0] + c1 * (shCoeff[8][0] * n[0] + shCoeff[4][0] * n[1] + shCoeff[7][0] * n[2])) * dn.col(0)
                    + 2.0f * al[0] * (c2 * shCoeff[1][0] + c1 * (shCoeff[4][0] * n[0] - shCoeff[8][0] * n[1] + shCoeff[5][0] * n[2])) * dn.col(1)
                    + 2.0f * al[0] * (c2 * shCoeff[2][0] + c1 * (shCoeff[7][0] * n[0] + shCoeff[5][0] * n[1]) + c3 * shCoeff[6][0] * n[2]) * dn.col(2);
                    dcdi.block(0, 1, dof.ID + dof.EX, 1) =
                    2.0f * al[1] * (c2 * shCoeff[3][1] + c1 * (shCoeff[8][1] * n[0] + shCoeff[4][1] * n[1] + shCoeff[7][1] * n[2])) * dn.col(0)
                    + 2.0f * al[1] * (c2 * shCoeff[1][1] + c1 * (shCoeff[4][1] * n[0] - shCoeff[8][1] * n[1] + shCoeff[5][1] * n[2])) * dn.col(1)
                    + 2.0f * al[1] * (c2 * shCoeff[2][1] + c1 * (shCoeff[7][1] * n[0] + shCoeff[5][1] * n[1]) + c3 * shCoeff[6][1] * n[2]) * dn.col(2);
                    dcdi.block(0, 2, dof.ID + dof.EX, 1) =
                    2.0f * al[2] * (c2 * shCoeff[3][2] + c1 * (shCoeff[8][2] * n[0] + shCoeff[4][2] * n[1] + shCoeff[7][2] * n[2])) * dn.col(0)
                    + 2.0f * al[2] * (c2 * shCoeff[1][2] + c1 * (shCoeff[4][2] * n[0] - shCoeff[8][2] * n[1] + shCoeff[5][2] * n[2])) * dn.col(1)
                    + 2.0f * al[2] * (c2 * shCoeff[2][2] + c1 * (shCoeff[7][2] * n[0] + shCoeff[5][2] * n[1]) + c3 * shCoeff[6][2] * n[2]) * dn.col(2);
                }
                
                // TODO: it can be pre-computed
                const cv::Vec3f& dIxij = dIx(j, i);
                const cv::Vec3f& dIyij = dIy(j, i);
                
                dIxy << dIxij(0), dIxij(1), dIxij(2),
                dIyij(0), dIyij(1), dIyij(2);
                
                // NOTE: need to divide by 32 to normalize image gradient
                dIxy *= 1.0f / 32.0f;
                
                const Eigen::MatrixXf& dp0 = dpV[idx0];
                const Eigen::MatrixXf& dp1 = dpV[idx1];
                const Eigen::MatrixXf& dp2 = dpV[idx2];
                
                dp = b[0] * dp0 + b[1] * dp1 + b[2] * dp2;
                
                di = dp * dIxy;
                dcdi.block(0, 0, dof.ID + dof.EX, 3) -= di.block(0, 0, dof.ID + dof.EX, 3);
                
                const auto& shade = renderTarget[RT_NAMES::shading](j, i);
                if(tex_mode){
                    const auto& uv = renderTarget[RT_NAMES::texCoords](j, i);
                    // convert UV [0,1] to image space
                    int tX = uv[0]*tex_w;
                    int tY = uv[1]*tex_h;
                    
                    for(int x = 0; x < dof.AL; ++x)
                    {
                        a(0, x) = w_al_uv[x](tY, tX)[0];
                        a(1, x) = w_al_uv[x](tY, tX)[1];
                        a(2, x) = w_al_uv[x](tY, tX)[2];
                    }
                }
                {
                    const Eigen::MatrixXf& a0 = w_al.block(tri_ind[0] * 3, 0, 3, dof.AL);
                    const Eigen::MatrixXf& a1 = w_al.block(tri_ind[1] * 3, 0, 3, dof.AL);
                    const Eigen::MatrixXf& a2 = w_al.block(tri_ind[2] * 3, 0, 3, dof.AL);
                    
                    a = b[0] * a0 + b[1] * a1 + b[2] * a2;
                }
                
                for (int x = 0; x < dof.AL; ++x)
                {
                    dcdi(dof.ID + dof.EX + x, 0) = a(0, x) * shade[0];
                    dcdi(dof.ID + dof.EX + x, 1) = a(1, x) * shade[1];
                    dcdi(dof.ID + dof.EX + x, 2) = a(2, x) * shade[2];
                }
                
                // TODO: treat camera derivative properly (for now, no need to optimize camera intrinsics though)
                dcdi.block(dof.ID + dof.EX + dof.AL, 0, dof.ROT + dof.TR + dof.CAM, 3) = -di.block(dof.ID + dof.EX + dof.AL, 0, dof.ROT + dof.TR + dof.CAM, 3);
                if (dof.SH == 27){
                    
                    for (int x = 0; x < 3; ++x)
                    {
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 0, x) = c4 * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 1, x) = 2.0f * c2 * n[1] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 2, x) = 2.0f * c2 * n[2] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 3, x) = 2.0f * c2 * n[0] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 4, x) = 2.0f * c1 * n[0] * n[1] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 5, x) = 2.0f * c1 * n[1] * n[2] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 6, x) = (c3 * n[2] * n[2] - c5)* al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 7, x) = 2.0f * c1 * n[2] * n[0] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 9 * x + 8, x) = c1 * (n[0] * n[0] - n[1] * n[1]) * al[x];
                    }
                }
                else if (dof.SH == 9)
                {
                    for (int x = 0; x < 3; ++x)
                    {
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 0, x) = c4 * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 1, x) = 2.0f * c2 * n[1] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 2, x) = 2.0f * c2 * n[2] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 3, x) = 2.0f * c2 * n[0] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 4, x) = 2.0f * c1 * n[0] * n[1] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 5, x) = 2.0f * c1 * n[1] * n[2] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 6, x) = (c3 * n[2] * n[2] - c5)* al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 7, x) = 2.0f * c1 * n[2] * n[0] * al[x];
                        dcdi(dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM + 8, x) = c1 * (n[0] * n[0] - n[1] * n[1]) * al[x];
                    }
                }
                
                auto CminI = c - inputRGB(j, i);
                CI << CminI[0], CminI[1], CminI[2];
                float CI_norm = CI.norm();
                if (CI_norm < EPSILON) CI_norm = EPSILON;	
                
                Jtrc.noalias() += w / CI_norm * (dcdi * CI);
                
                const Eigen::MatrixXf dcdi_transpose = dcdi.transpose();
                JtJc.noalias() += w / CI_norm * (dcdi * dcdi_transpose);
                
                count++;
            }
        }
    }
    
    if (count != 0){
        JtJc *= 1.0f / (float)count;
        Jtrc *= 1.0f / (float)count;
    }
    
    JtJ += JtJc;
    Jtr += Jtrc;
}

void F2FRenderParams::init(GLProgram& prog)
{
    prog.createUniform("u_enable_texture", DataType::UINT);
    prog.createUniform("u_enable_mask", DataType::UINT);
    prog.createUniform("u_enable_seg", DataType::UINT);
    
    prog.createUniform("u_cull_offset", DataType::FLOAT);
}

void F2FRenderParams::update(GLProgram& prog)
{
    prog.setUniformData("u_enable_texture", enable_tex);
    prog.setUniformData("u_enable_mask", enable_mask);
    prog.setUniformData("u_enable_seg", enable_seg);
    
    prog.setUniformData("u_cull_offset", cull_offset);
}

void F2FRenderer::init(std::string data_dir, Camera& camera, FaceModel& model)
{
    prog_ = GLProgram(data_dir + "shaders/face2face.vert",
                      data_dir + "shaders/face2face.geom",
                      data_dir + "shaders/face2face.frag",
                      DrawMode::TRIANGLES_IDX);
    
    param_.init(prog_);
    prog_.createUniform("u_SHCoeffs", DataType::VECTOR3);
    fb_ = Framebuffer::Create(camera.width_, camera.height_, 8);
    camera.intializeUniforms(prog_, true, false);
    mesh_.init_with_idx(prog_, model.pts_, model.clr_, model.nml_, model.uvs_, model.tri_pts_, model.tri_uv_);
}

void F2FRenderer::render(int w, int h, Camera& camera, FaceParams& fParam, FaceModel& model, std::vector<cv::Mat_<cv::Vec4f>>& out)
{
    // render parameters update
    param_.update(prog_);
    
    // spherical harmonics update
    std::vector<glm::vec3> sh;
    for(int i = 0; i < 9; ++i)
    {
        sh.push_back(glm::vec3(fParam.SH[i][0],fParam.SH[i][1],fParam.SH[i][2]));
    }
    prog_.setUniformData("u_SHCoeffs", sh);
    
    // camera parameters update
    camera.updateUniforms(prog_, fParam.RT, true, false);
    
    // update mesh attributes
    mesh_.update_with_idx(prog_, model.pts_, model.clr_, model.nml_);
    
    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, w, h);
    clearBuffer(COLOR::COLOR_ALPHA);
    
    // draw mesh
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    prog_.draw();
    
    // unbinding framebuffer
    fb_->Unbind();
    
    fb_->RetrieveFBO(w, h, out);
}
