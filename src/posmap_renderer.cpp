//
//  posmap_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "posmap_renderer.h"
#include "subdiv.h"

void PosMapRenderer::init(std::string data_dir,
                          FaceModelPtr model)
{
    programs_["main"] = GLProgram(data_dir + "shaders/subdiv.vert",
                                  data_dir + "shaders/subdiv.tc",
                                  data_dir + "shaders/subdiv.te",
                                  "",
                                  data_dir + "shaders/subdiv.frag",
                                  DrawMode::PATCHES);
    programs_["plane"] = GLProgram(data_dir + "shaders/full_texture_bgr.vert",
                                   data_dir + "shaders/full_texture_bgr.frag",
                                   DrawMode::TRIANGLES);
    
    auto& prog = programs_["main"];
    auto& prog_pl = programs_["plane"];
    
    fb_ = Framebuffer::Create(256, 256, 2); // will be resized based on frame size
    
    prog.createUniform("u_tessinner", DataType::FLOAT);
    prog.createUniform("u_tessouter", DataType::FLOAT);
    prog.createUniform("u_tessalpha", DataType::FLOAT);
    
    mesh_.init(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog, AT_UV);
    
    plane_.init(prog_pl,0.01);
    prog_pl.createTexture("u_texture", fb_->color(0), fb_->width(), fb_->height());
}

void PosMapRenderer::render(const FaceData& fd)
{
    // render parameters update
    auto& prog = programs_["main"];
    auto& prog_pl = programs_["plane"];
    
//    Eigen::VectorXf pts_dst;
//    Eigen::MatrixX2f uvs_dst;
//    Eigen::MatrixX3i triuv_dst;
//    Eigen::MatrixX3i tripts_dst;
//    Eigen::MatrixX3f nml_dst;
//    performSubdiv(fd.pts_, fd.uvs(), fd.triuv(), fd.tripts(), pts_dst, uvs_dst, triuv_dst, tripts_dst);
//    calcNormal(nml_dst, pts_dst, tripts_dst);
//
//    mesh_.update_uv(uvs_dst, triuv_dst);
//    mesh_.update(prog, AT_UV);
//    mesh_.update_position(pts_dst, tripts_dst);
//    mesh_.update_normal(nml_dst, tripts_dst);
//    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_UV);

    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_tessalpha", tessAlpha_);

    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw(wire_);
    fb_->Unbind();
    
    std::vector<cv::Mat_<cv::Vec4f>> out;
    fb_->RetrieveFBO(256, 256, out);
    
//    prog_pl.updateTexture("u_texture", fb_->color((uint)location_));
    GLFWwindow* window = glfwGetCurrentContext();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
//    glDisable(GL_CULL_FACE);
//    glEnable(GL_BLEND);
//    glEnable(GL_DEPTH_TEST);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    prog_pl.draw();
    
    cv::Mat_<cv::Vec3f> pos, dIx, dIy;
    cv::cvtColor(out[0], pos, cv::COLOR_BGRA2BGR);
    cv::Scharr(pos, dIx, CV_32F, 1, 0);
    cv::Scharr(pos, dIy, CV_32F, 0, 1);
    
    cv::Mat_<cv::Vec3f> nml(pos.size());
    for(int y=0; y<pos.rows; ++y)
    {
        for(int x=0; x<pos.cols; ++x)
        {
            auto& a = dIx(y,x);
            auto& b = dIy(y,x);
            
            auto c = b.cross(a);
            c = c / cv::norm(c);
            nml(y,x)[2] = c[0];
            nml(y,x)[1] = c[1];
            nml(y,x)[0] = c[2];
        }
    }
    
    cv::imshow("normal", nml);
    cv::waitKey(1);
    
}

void PosMapRenderer::render(int w, int h, const FaceData& fd, std::vector<cv::Mat_<cv::Vec4f>>& out)
{
    if(w != fb_->width() || h != fb_->height())
        fb_->Resize(w, h, 2);
    
    auto& prog = programs_["main"];
    
    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_tessalpha", tessAlpha_);
    
    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw(wire_);
    
    fb_->Unbind();
    
    fb_->RetrieveFBO(w, h, out);
}

#ifdef FACE_TOOLKIT
void PosMapRenderer::render(const FaceResult& result)
{
    if(show_)
        render(result.fd);
}
#endif

#ifdef WITH_IMGUI
void PosMapRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        const char* listbox_items[] = { "positions", "normals"};
        ImGui::ListBox("RenderTarget", &location_, listbox_items, 2);
        ImGui::InputInt("TessInner", &tessInner_);
        ImGui::InputInt("TessOuter", &tessOuter_);
        ImGui::SliderFloat("TessAlpha", &tessAlpha_, 0.0, 1.0);
    }
}
#endif

RendererHandle PosMapRenderer::Create(std::string name, bool show)
{
    auto renderer = new PosMapRenderer(name, show);
    
    return RendererHandle(renderer);
}
