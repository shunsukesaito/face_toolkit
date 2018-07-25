#include "f2f_renderer.h"

#include <gflags/gflags.h>
DEFINE_uint32(f2f_render_location, 4, "f2f rendering location (defalut: diffuse)");
DEFINE_string(f2f_seg_path, "", "segmentation file for f2f (default: none)");
DEFINE_string(f2f_tex_path, "", "texture file for f2f (default: none)");

static const float c1 = 0.429043f, c2 = 0.511664f, c3 = 0.743125f, c4 = 0.886227f, c5 = 0.247708f;

static Eigen::MatrixXf computedIdn(int ch,
                                   const Eigen::Matrix3Xf& sh,
                                   const Eigen::Vector3f& n,
                                   const Eigen::Matrix3Xf& dn)
{
    return 2.0f * (c2 * sh(ch,3) + c1 * (sh(ch,8) * n[0] + sh(ch,4) * n[1] + sh(ch,7) * n[2])) * dn.row(0)
    + 2.0f * (c2 * sh(ch,1) + c1 * (sh(ch,4) * n[0] - sh(ch,8) * n[1] + sh(ch,5) * n[2])) * dn.row(1)
    + 2.0f * (c2 * sh(ch,2) + c1 * (sh(ch,7) * n[0] + sh(ch,5) * n[1]) + c3 * sh(ch,6) * n[2]) * dn.row(2);
}

static void computedIdsh(Eigen::Ref<Eigen::MatrixXf> dout,
                         const Eigen::Vector3f& n,
                         const cv::Vec4f& al,
                         int dof)
{
    if (dof == 27){
        for (int x = 0; x < 3; ++x)
        {
            dout(x, 9 * x + 0) = c4 * al[x];
            dout(x, 9 * x + 1) = 2.0f * c2 * n[1] * al[x];
            dout(x, 9 * x + 2) = 2.0f * c2 * n[2] * al[x];
            dout(x, 9 * x + 3) = 2.0f * c2 * n[0] * al[x];
            dout(x, 9 * x + 4) = 2.0f * c1 * n[0] * n[1] * al[x];
            dout(x, 9 * x + 5) = 2.0f * c1 * n[1] * n[2] * al[x];
            dout(x, 9 * x + 6) = (c3 * n[2] * n[2] - c5)* al[x];
            dout(x, 9 * x + 7) = 2.0f * c1 * n[2] * n[0] * al[x];
            dout(x, 9 * x + 8) = c1 * (n[0] * n[0] - n[1] * n[1]) * al[x];
        }
    }
    else if (dof == 9)
    {
        for (int x = 0; x < 3; ++x)
        {
            dout(x, 0) = c4 * al[x];
            dout(x, 1) = 2.0f * c2 * n[1] * al[x];
            dout(x, 2) = 2.0f * c2 * n[2] * al[x];
            dout(x, 3) = 2.0f * c2 * n[0] * al[x];
            dout(x, 4) = 2.0f * c1 * n[0] * n[1] * al[x];
            dout(x, 5) = 2.0f * c1 * n[1] * n[2] * al[x];
            dout(x, 6) = (c3 * n[2] * n[2] - c5)* al[x];
            dout(x, 7) = 2.0f * c1 * n[2] * n[0] * al[x];
            dout(x, 8) = c1 * (n[0] * n[0] - n[1] * n[1]) * al[x];
        }
    }
}

float F2FRenderer::computeJacobianColor(Eigen::VectorXf& Jtr,
                                        Eigen::MatrixXf& JtJ,
                                        const FaceData& fd,
                                        const std::vector<Eigen::Vector2f>& pV,
                                        const std::vector<Eigen::Matrix2Xf>& dpV,
                                        const std::vector<Eigen::Vector3f>& nV,
                                        const std::vector<Eigen::Matrix3Xf>& dnV,
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
    
    Eigen::MatrixXf JtJc = Eigen::MatrixXf::Zero(dof.all(), dof.all());
    Eigen::MatrixXf Jtrc = Eigen::MatrixXf::Zero(dof.all(), 1);
    Eigen::Vector3f n;
    Eigen::Vector3f CI;
    Eigen::MatrixXf dn = Eigen::MatrixXf::Zero(3, dof.ID + dof.EX);
    Eigen::MatrixXf dcdi = Eigen::MatrixXf::Zero(3, dof.all());
    Eigen::MatrixXf dp = Eigen::MatrixXf::Zero(2, dof.all());
    Eigen::MatrixXf a = Eigen::MatrixXf::Zero(3, dof.AL);
    Eigen::Matrix<float, 3, 2> dIxy;
    
    int idx0, idx1, idx2;
    int count = 0;
    
    //#pragma omp parallel for
    const unsigned int width = renderTarget[RT_NAMES::positions].cols;
    const unsigned int height = renderTarget[RT_NAMES::positions].rows;
    float error = 0.0;
    for (unsigned int j = 1; j < height - 1; ++j)
    {
        for (unsigned int i = 1; i < width - 1; ++i)
        {
            const auto& c = renderTarget[RT_NAMES::diffuse](j, i);
            const auto& al = renderTarget[RT_NAMES::colors](j, i);
            const auto& b = renderTarget[RT_NAMES::vBarycentric](j, i);
            const auto& tri_indf = renderTarget[RT_NAMES::vIndices](j, i);
  
            if (renderTarget[RT_NAMES::diffuse](j, i)[3] != 0.0){
                dcdi.setZero();
                Eigen::Ref<Eigen::MatrixXf> sh_block   = dcdi.block(0, dof.AL + dof.ID + dof.EX + dof.fROT + dof.fTR + dof.cROT + dof.cTR + dof.CAM, 3, dof.SH);
                Eigen::Ref<Eigen::MatrixXf> idex_block = dcdi.block(0, dof.AL, 3, dof.ID + dof.EX);
                Eigen::Ref<Eigen::MatrixXf> di_block   = dcdi.block(0, dof.AL, 3, dof.ID + dof.EX + dof.fROT + dof.fTR + dof.cROT + dof.cTR + dof.CAM);
                Eigen::Ref<Eigen::MatrixXf> al_block   = dcdi.block(0, 0, 3, dof.AL);
                
                idx0 = static_cast<int>(tri_indf[0] + 0.5f);
                idx1 = static_cast<int>(tri_indf[1] + 0.5f);
                idx2 = static_cast<int>(tri_indf[2] + 0.5f);
                
                n  = b[0]*nV[idx0] + b[1]*nV[idx1] + b[2]*nV[idx2];
                dn = b[0]*dnV[idx0] + b[1]*dnV[idx1] + b[2]*dnV[idx2];
                
                // normalized version of normal derivative
                float n_norm = n.norm();
                if (n_norm < EPSILON) n_norm = EPSILON;
                dn = (n_norm*dn + 1.0f / n_norm * n * n.transpose() * dn) / (n_norm * n_norm);
                n /= n_norm;
                
                const auto& shade = renderTarget[RT_NAMES::shading](j, i);
                
                const Eigen::MatrixXf& a0 = fd.dCL(idx0, dof.AL);
                const Eigen::MatrixXf& a1 = fd.dCL(idx1, dof.AL);
                const Eigen::MatrixXf& a2 = fd.dCL(idx2, dof.AL);
                
                a = b[0] * a0 + b[1] * a1 + b[2] * a2;
                
                const cv::Vec3f& dIxij = dIx(j, i);
                const cv::Vec3f& dIyij = dIy(j, i);
                
                dIxy << dIxij(0), dIyij(0),
                dIxij(1), dIyij(1),
                dIxij(2), dIyij(2);
                
                // NOTE: need to divide by 32 to normalize image gradient
                dIxy *= 1.0f / 32.0f;
                
                const Eigen::MatrixXf& dp0 = dpV[idx0];
                const Eigen::MatrixXf& dp1 = dpV[idx1];
                const Eigen::MatrixXf& dp2 = dpV[idx2];
                
                dp = b[0] * dp0 + b[1] * dp1 + b[2] * dp2;
                
                al_block.row(0) = shade[0] * a.block(0,0,1,dof.AL);
                al_block.row(1) = shade[1] * a.block(1,0,1,dof.AL);
                al_block.row(2) = shade[2] * a.block(2,0,1,dof.AL);
                
                di_block = - dIxy * dp;

                idex_block.row(0) += al[0] * computedIdn(0, sh, n, dn);
                idex_block.row(1) += al[1] * computedIdn(1, sh, n, dn);
                idex_block.row(2) += al[2] * computedIdn(2, sh, n, dn);
                
                computedIdsh(sh_block, n, al, dof.SH);
                
                auto CminI = c - inputRGB(j, i);
                CI << CminI[0], CminI[1], CminI[2];
                float CI_norm = CI.norm();
                if (CI_norm < EPSILON) CI_norm = EPSILON;	
                
                const Eigen::MatrixXf& dcdi_transpose = dcdi.transpose();
                Jtrc.noalias() += w / CI_norm * (dcdi_transpose * CI);
                JtJc.noalias() += w / CI_norm * (dcdi_transpose * dcdi);
                
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

void F2FRenderParams::init(GLProgram& prog)
{
    prog.createUniform("u_enable_texture", DataType::UINT);
    prog.createUniform("u_enable_mask", DataType::UINT);
    prog.createUniform("u_enable_seg", DataType::UINT);
    prog.createUniform("u_enable_cull", DataType::UINT);
    
    prog.createUniform("u_cull_offset", DataType::FLOAT);
    
    prog.createUniform("u_tex_mode", DataType::UINT);
    prog.createUniform("u_inv_diffuse", DataType::UINT);
}

void F2FRenderParams::update(GLProgram& prog)
{
    prog.setUniformData("u_enable_texture", (uint)enable_tex);
    prog.setUniformData("u_enable_mask", (uint)enable_mask);
    prog.setUniformData("u_enable_seg", (uint)enable_seg);
    prog.setUniformData("u_enable_cull", (uint)enable_cull);
    
    prog.setUniformData("u_cull_offset", cull_offset);
    
    prog.setUniformData("u_tex_mode", (uint)tex_mode);
    prog.setUniformData("u_inv_diffuse", (uint)enable_inv_diffuse);
}

#ifdef WITH_IMGUI
void F2FRenderParams::updateIMGUI()
{
    ImGui::Checkbox("mask", &enable_mask);
    ImGui::Checkbox("seg", &enable_seg);
    ImGui::Checkbox("cull", &enable_cull);
    ImGui::Checkbox("texture", &enable_tex);
    ImGui::Checkbox("tex view", &tex_mode);
    ImGui::Checkbox("inv diffuse", &enable_inv_diffuse);
    ImGui::SliderFloat("Transparency", &alpha, 0.0, 1.0);
    ImGui::SliderFloat("cullOffset", &cull_offset, -1.0, 0.0);
    const char* listbox_items[] = { "positions", "normals", "albedo", "texCoords", "diffuse", "shading", "vBarycentric", "vIndices"};
    ImGui::ListBox("RenderTarget", &location, listbox_items, 8);
}
#endif

void F2FRenderer::updateSegment(const cv::Mat& seg)
{
    auto& prog_f2f = programs_["f2f"];
    prog_f2f.updateTexture("u_sample_seg", seg);
}

void F2FRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr model)
{
    programs_["f2f"] = GLProgram(shader_dir, "face2face.vert", "face2face.geom", "face2face.frag", DrawMode::TRIANGLES_IDX);
    programs_["depth"] = GLProgram(shader_dir, "depthmap.vert", "depthmap.frag", DrawMode::TRIANGLES_IDX);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog_f2f = programs_["f2f"];
    auto& prog_depth = programs_["depth"];
    auto& prog_pl = programs_["plane"];
    
    param_.init(prog_f2f);
    prog_f2f.createUniform("u_SHCoeffs", DataType::VECTOR3);
    prog_f2f.createTexture("u_sample_mask", data_dir + "f2f_mask.png");
     if (!FLAGS_f2f_seg_path.empty())
         prog_f2f.createTexture("u_sample_seg", FLAGS_f2f_seg_path);
     else{
         cv::Mat_<cv::Vec3b> seg(100,100,cv::Vec3b(0,0,0));
         prog_f2f.createTexture("u_sample_seg", seg);
     }
     if (!FLAGS_f2f_tex_path.empty())
         prog_f2f.createTexture("u_sample_texture", FLAGS_f2f_tex_path);
     else{
         cv::Mat_<cv::Vec3b> tex(100,100,cv::Vec3b(0,0,0));
         prog_f2f.createTexture("u_sample_texture", tex);
     }
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);

    fb_ = Framebuffer::Create(1, 1, RT_NAMES::count); // will be resized based on frame size
    fb_depth_ = Framebuffer::Create(1, 1, 0);
    
    Camera::initializeUniforms(prog_f2f, U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW);
    Camera::initializeUniforms(prog_depth, U_CAMERA_MVP);
    
    mesh_.update_tri(model->tri_pts_);
    mesh_.init(prog_f2f, AT_POSITION | AT_NORMAL | AT_COLOR | AT_UV | AT_TRI);
    mesh_.init(prog_depth, AT_POSITION | AT_TRI);
    
    mesh_.update_position(model->meanShape());
    mesh_.update_uv(model->uvs_, model->tri_uv_, model->tri_pts_);
    mesh_.update(prog_f2f, AT_UV);
    
    plane_.init(prog_pl,0.01);
    prog_pl.createTexture("u_texture", fb_->color(RT_NAMES::diffuse), fb_->width(), fb_->height());
    prog_f2f.createTexture("u_sample_depth", fb_depth_->depth(), fb_depth_->width(), fb_depth_->height());
}

void F2FRenderer::render(const Camera& camera, const FaceData& fd)
{
    if(camera.width_ != fb_->width() || camera.height_ != fb_->height()){
        fb_->Resize(camera.width_, camera.height_, RT_NAMES::count);
        fb_depth_->Resize(camera.width_, camera.height_, 0);
    }
    
    // render parameters update
    auto& prog_f2f = programs_["f2f"];
    auto& prog_depth = programs_["depth"];
    auto& prog_pl = programs_["plane"];
    
    if(camera.weakPersp_)
        param_.enable_cull = false;
    param_.update(prog_f2f);
    
    prog_pl.setUniformData("u_alpha", param_.alpha);
    
    // spherical harmonics update
    std::vector<glm::vec3> sh;
    for(int i = 0; i < 9; ++i)
    {
        sh.push_back(glm::vec3(fd.SH(0,i),fd.SH(1,i),fd.SH(2,i)));
    }
    prog_f2f.setUniformData("u_SHCoeffs", sh);
    
    // camera parameters update
    camera.updateUniforms(prog_f2f, fd.RT, U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW);
    
    // update mesh attributes
    mesh_.update_position(fd.pts_);
    mesh_.update_color(fd.clr_);
    mesh_.update_normal(fd.nml_);
    
    mesh_.update(prog_f2f, AT_POSITION | AT_COLOR | AT_NORMAL);
    
    if(param_.tex_mode){
        camera.updateUniforms(prog_depth, fd.RT, U_CAMERA_MVP);
        mesh_.update(prog_depth, AT_POSITION);
        
        // NOTE: need to make sure the viewport size matches the framebuffer size
        fb_depth_->Bind();
        glViewport(0, 0, fb_depth_->width(), fb_depth_->height());
        clearBuffer(COLOR::COLOR_ALPHA);
        prog_depth.draw();
        fb_depth_->Unbind();
    }
    
    fb_->Bind();
    
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    // draw mesh
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    prog_f2f.draw(wire_);
    
    fb_->Unbind();
    
    prog_pl.updateTexture("u_texture", fb_->color((uint)param_.location));
    GLFWwindow* window = glfwGetCurrentContext();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
}

void F2FRenderer::render(int w, int h, const Camera& camera, const FaceData& fd, std::vector<cv::Mat_<cv::Vec4f>>& out)
{
    if(camera.width_ != fb_->width() || camera.height_ != fb_->height()){
        fb_->Resize(camera.width_, camera.height_, RT_NAMES::count);
        fb_depth_->Resize(camera.width_, camera.height_, 0);
    }
    
    auto& prog_f2f = programs_["f2f"];
    auto& prog_depth = programs_["depth"];
    
    // render parameters update
    param_.update(prog_f2f);
    
    // spherical harmonics update
    std::vector<glm::vec3> sh;
    for(int i = 0; i < 9; ++i)
    {
        sh.push_back(glm::vec3(fd.SH(0,i),fd.SH(1,i),fd.SH(2,i)));
    }
    prog_f2f.setUniformData("u_SHCoeffs", sh);
    
    // camera parameters update
    camera.updateUniforms(prog_f2f, fd.RT, U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW);
    
    // update mesh attributes
    mesh_.update_position(fd.pts_);
    mesh_.update_color(fd.clr_);
    mesh_.update_normal(fd.nml_);
    
    mesh_.update(prog_f2f, AT_POSITION | AT_COLOR | AT_NORMAL);
    
    if(param_.tex_mode){
        camera.updateUniforms(prog_depth, fd.RT, U_CAMERA_MVP);
        mesh_.update(prog_depth, AT_POSITION);
        
        // NOTE: need to make sure the viewport size matches the framebuffer size
        fb_depth_->Bind();
        glViewport(0, 0, fb_depth_->width(), fb_depth_->height());
        clearBuffer(COLOR::COLOR_ALPHA);
        prog_depth.draw();
        fb_depth_->Unbind();
    }
    
    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, w, h);
    clearBuffer(COLOR::COLOR_ALPHA);
    
    // draw mesh
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    prog_f2f.draw(wire_);
    
    // unbinding framebuffer
    fb_->Unbind();
    
    fb_->RetrieveFBO(w, h, out);
}

#ifdef FACE_TOOLKIT
void F2FRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    if(!result.cap_data[frame_id][cam_id].seg_.empty())
        updateSegment(result.cap_data[frame_id][cam_id].seg_);
    if(param_.enable_tex)
        programs_["f2f"].updateTexture("u_sample_texture", result.cap_data[frame_id][cam_id].img_);
    
    if(show_)
        render(result.cameras[cam_id], result.fd[frame_id]);
}
#endif

#ifdef WITH_IMGUI
void F2FRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        param_.updateIMGUI();
    }
}
#endif

RendererHandle F2FRenderer::Create(std::string name, bool show)
{
    auto renderer = new F2FRenderer(name, show);
    renderer->param_.location = FLAGS_f2f_render_location;
    
    return RendererHandle(renderer);
}
