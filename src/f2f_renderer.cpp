#include "f2f_renderer.hpp"



float F2FRenderer::computeJacobianColor(Eigen::VectorXf& Jtr,
                                        Eigen::MatrixXf& JtJ,
                                        const Eigen::MatrixXf& w_al,
                                        const std::vector<Eigen::Vector2f>& pV,
                                        const std::vector<Eigen::MatrixX2f>& dpV,
                                        const std::vector<Eigen::Vector3f>& nV,
                                        const std::vector<Eigen::MatrixXf>& dnV,
                                        const Eigen::Matrix3Xf& sh,
                                        const std::vector< cv::Mat_<cv::Vec4f> >& renderTarget,
                                        const cv::Mat_<cv::Vec4f>& inputRGB,
                                        const cv::Mat_<cv::Vec3f>& dIx,
                                        const cv::Mat_<cv::Vec3f>& dIy,
                                        const DOF& dof,
                                        const float w)
{
    // compute gradient for each pixel
    // since the interpolated normal is normalized again, compute jacobian based on normalized normal
    const float c1 = 0.429043f, c2 = 0.511664f, c3 = 0.743125f, c4 = 0.886227f, c5 = 0.247708f;
    
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
    float error = 0.0;
    int cur_pos;
    for (unsigned int j = 1; j < height - 1; ++j)
    {
        for (unsigned int i = 1; i < width - 1; ++i)
        {
            const auto& c = renderTarget[RT_NAMES::diffuse](j, i);
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
                    for(int k = 0; k < 3; ++k){
                        dcdi.block(0, k, dof.ID + dof.EX, 1) =
                        2.0f * al[k] * (c2 * sh(k,3) + c1 * (sh(k,8) * n[0] + sh(k,4) * n[1] + sh(k,7) * n[2])) * dn.col(0)
                        + 2.0f * al[k] * (c2 * sh(k,1) + c1 * (sh(k,4) * n[0] - sh(k,8) * n[1] + sh(k,5) * n[2])) * dn.col(1)
                        + 2.0f * al[k] * (c2 * sh(k,2) + c1 * (sh(k,7) * n[0] + sh(k,5) * n[1]) + c3 * sh(k,6) * n[2]) * dn.col(2);
                    }
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

                const Eigen::MatrixXf& a0 = w_al.block(tri_ind[0] * 3, 0, 3, dof.AL);
                const Eigen::MatrixXf& a1 = w_al.block(tri_ind[1] * 3, 0, 3, dof.AL);
                const Eigen::MatrixXf& a2 = w_al.block(tri_ind[2] * 3, 0, 3, dof.AL);
                
                a = b[0] * a0 + b[1] * a1 + b[2] * a2;
                
                for (int x = 0; x < dof.AL; ++x)
                {
                    dcdi(dof.ID + dof.EX + x, 0) = a(0, x) * shade[0];
                    dcdi(dof.ID + dof.EX + x, 1) = a(1, x) * shade[1];
                    dcdi(dof.ID + dof.EX + x, 2) = a(2, x) * shade[2];
                }
                
                // TODO: treat camera derivative properly (for now, no need to optimize camera intrinsics though)
                cur_pos = dof.ID + dof.EX + dof.AL;
                dcdi.block(cur_pos, 0, dof.ROT + dof.TR + dof.CAM, 3) = -di.block(cur_pos, 0, dof.ROT + dof.TR + dof.CAM, 3);
                if (dof.SH == 27){
                    cur_pos = dof.ID + dof.EX + dof.AL + dof.ROT + dof.TR + dof.CAM;
                    for (int x = 0; x < 3; ++x)
                    {
                        dcdi(cur_pos + 9 * x + 0, x) = c4 * al[x];
                        dcdi(cur_pos + 9 * x + 1, x) = 2.0f * c2 * n[1] * al[x];
                        dcdi(cur_pos + 9 * x + 2, x) = 2.0f * c2 * n[2] * al[x];
                        dcdi(cur_pos + 9 * x + 3, x) = 2.0f * c2 * n[0] * al[x];
                        dcdi(cur_pos + 9 * x + 4, x) = 2.0f * c1 * n[0] * n[1] * al[x];
                        dcdi(cur_pos + 9 * x + 5, x) = 2.0f * c1 * n[1] * n[2] * al[x];
                        dcdi(cur_pos + 9 * x + 6, x) = (c3 * n[2] * n[2] - c5)* al[x];
                        dcdi(cur_pos + 9 * x + 7, x) = 2.0f * c1 * n[2] * n[0] * al[x];
                        dcdi(cur_pos + 9 * x + 8, x) = c1 * (n[0] * n[0] - n[1] * n[1]) * al[x];
                    }
                }
                else if (dof.SH == 9)
                {
                    for (int x = 0; x < 3; ++x)
                    {
                        dcdi(cur_pos + 0, x) = c4 * al[x];
                        dcdi(cur_pos + 1, x) = 2.0f * c2 * n[1] * al[x];
                        dcdi(cur_pos + 2, x) = 2.0f * c2 * n[2] * al[x];
                        dcdi(cur_pos + 3, x) = 2.0f * c2 * n[0] * al[x];
                        dcdi(cur_pos + 4, x) = 2.0f * c1 * n[0] * n[1] * al[x];
                        dcdi(cur_pos + 5, x) = 2.0f * c1 * n[1] * n[2] * al[x];
                        dcdi(cur_pos + 6, x) = (c3 * n[2] * n[2] - c5)* al[x];
                        dcdi(cur_pos + 7, x) = 2.0f * c1 * n[2] * n[0] * al[x];
                        dcdi(cur_pos + 8, x) = c1 * (n[0] * n[0] - n[1] * n[1]) * al[x];
                    }
                }
                
                auto CminI = c - inputRGB(j, i);
                CI << CminI[0], CminI[1], CminI[2];
                float CI_norm = CI.norm();
                if (CI_norm < EPSILON) CI_norm = EPSILON;	
                
                Jtrc.noalias() += w / CI_norm * (dcdi * CI);
                
                const Eigen::MatrixXf dcdi_transpose = dcdi.transpose();
                JtJc.noalias() += w / CI_norm * (dcdi * dcdi_transpose);
                
                error += w * CI_norm;
                
                count++;
            }
        }
    }
    
    if (count != 0){
        JtJc *= 1.0f / (float)count;
        Jtrc *= 1.0f / (float)count;
        error *= 1.0f / (float)count;
    }
    
    JtJ += JtJc;
    Jtr += Jtrc;
    
    return error;
}

void F2FRenderParams::init(GLProgram& prog, bool _preview)
{
    prog.createUniform("u_enable_texture", DataType::UINT);
    prog.createUniform("u_enable_mask", DataType::UINT);
    prog.createUniform("u_enable_seg", DataType::UINT);
    
    prog.createUniform("u_cull_offset", DataType::FLOAT);
    
    preview = _preview;
    if(preview){
        prog.createUniform("u_tex_mode", DataType::UINT);
        prog.createUniform("u_inv_diffuse", DataType::UINT);
    }
}

void F2FRenderParams::update(GLProgram& prog)
{
    prog.setUniformData("u_enable_texture", (uint)enable_tex);
    prog.setUniformData("u_enable_mask", (uint)enable_mask);
    prog.setUniformData("u_enable_seg", (uint)enable_seg);
    
    prog.setUniformData("u_cull_offset", cull_offset);
    
    if(preview){
        prog.setUniformData("u_tex_mode", (uint)tex_mode);
        prog.setUniformData("u_inv_diffuse", (uint)enable_inv_diffuse);
    }
}

#ifdef WITH_IMGUI
void F2FRenderParams::updateIMGUI()
{
    if (ImGui::CollapsingHeader("F2F Rendering Parameters")){
        ImGui::Checkbox("mask", &enable_mask);
        ImGui::Checkbox("seg", &enable_seg);
        ImGui::Checkbox("texture", &enable_tex);
        ImGui::SliderFloat("cullOffset", &cull_offset, -1.0, 0.0);
        const char* listbox_items[] = { "positions", "normals", "albedo", "texCoords", "diffuse", "shading", "vBarycentric", "vIndices"};
        ImGui::ListBox("RenderTarget", &location, listbox_items, 8);
    }
}
#endif

void F2FRenderer::init(std::string data_dir, FaceModel& model)
{
    programs_["f2f"] = GLProgram(data_dir + "shaders/face2face.vert",
                                 data_dir + "shaders/face2face.geom",
                                 data_dir + "shaders/face2face.frag",
                                 DrawMode::TRIANGLES_IDX);
    programs_["plane"] = GLProgram(data_dir + "shaders/full_texture_bgr.vert",
                                   data_dir + "shaders/full_texture_bgr.frag",
                                   DrawMode::TRIANGLES);
    
    auto& prog_f2f = programs_["f2f"];
    auto& prog_pl = programs_["plane"];
    
    param_.init(prog_f2f);
    prog_f2f.createUniform("u_SHCoeffs", DataType::VECTOR3);
    prog_f2f.createTexture("u_sample_mask", data_dir + "data/f2f_mask.png");
    fb_ = Framebuffer::Create(1, 1, RT_NAMES::count); // will be resized based on frame size
    
    Camera::initializeUniforms(prog_f2f, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update_tri(model.tri_pts_);
    mesh_.init(prog_f2f, AT_POSITION | AT_NORMAL | AT_COLOR | AT_UV | AT_TRI);
    
    mesh_.update_uv(model.uvs_, model.tri_uv_, model.tri_pts_);
    mesh_.update(prog_f2f, AT_UV);
    
    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_->color(RT_NAMES::diffuse), fb_->width(), fb_->height());
}

void F2FRenderer::render(const Camera& camera, const FaceParams& fParam)
{
    if(camera.width_ != fb_->width() || camera.height_ != fb_->height())
        fb_->Resize(camera.width_, camera.height_, RT_NAMES::count);
    
    // render parameters update
    auto& prog_f2f = programs_["f2f"];
    auto& prog_pl = programs_["plane"];
    
    param_.update(prog_f2f);
    
    // spherical harmonics update
    std::vector<glm::vec3> sh;
    for(int i = 0; i < 9; ++i)
    {
        sh.push_back(glm::vec3(fParam.SH(0,i),fParam.SH(1,i),fParam.SH(2,i)));
    }
    prog_f2f.setUniformData("u_SHCoeffs", sh);
    
    // camera parameters update
    camera.updateUniforms(prog_f2f, fParam.RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    // update mesh attributes
    mesh_.update_position(fParam.pts_);
    mesh_.update_color(fParam.clr_);
    mesh_.update_normal(fParam.nml_);
    
    mesh_.update(prog_f2f, AT_POSITION | AT_COLOR | AT_NORMAL | AT_TRI);
    
    fb_->Bind();
    
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    // draw mesh
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    prog_f2f.draw();
    
    fb_->Unbind();
    
    prog_pl.updateTexture("u_texture", fb_->color((uint)param_.location));
    GLFWwindow* window = glfwGetCurrentContext();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
}

void F2FRenderer::render(int w, int h, const Camera& camera, const FaceParams& fParam, std::vector<cv::Mat_<cv::Vec4f>>& out)
{
    auto& prog_f2f = programs_["f2f"];
    
    // render parameters update
    param_.update(prog_f2f);
    
    // spherical harmonics update
    std::vector<glm::vec3> sh;
    for(int i = 0; i < 9; ++i)
    {
        sh.push_back(glm::vec3(fParam.SH(0,i),fParam.SH(1,i),fParam.SH(2,i)));
    }
    prog_f2f.setUniformData("u_SHCoeffs", sh);
    
    // camera parameters update
    camera.updateUniforms(prog_f2f, fParam.RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    // update mesh attributes
    mesh_.update_position(fParam.pts_);
    mesh_.update_color(fParam.clr_);
    mesh_.update_normal(fParam.nml_);
    
    mesh_.update(prog_f2f, AT_POSITION | AT_COLOR | AT_NORMAL);
    
    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, w, h);
    clearBuffer(COLOR::COLOR_ALPHA);
    
    // draw mesh
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    prog_f2f.draw();
    
    // unbinding framebuffer
    fb_->Unbind();
    
    fb_->RetrieveFBO(w, h, out);
}
