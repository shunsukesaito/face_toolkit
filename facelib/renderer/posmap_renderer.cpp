//
//  posmap_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "posmap_renderer.h"

// constants
#include <gflags/gflags.h>
DEFINE_uint32(pm_size, 256, "size of geometry_image");
DEFINE_uint32(pm_tessin, 3, "number of tessellation in inner loop");
DEFINE_uint32(pm_tessout, 3, "number of tessellation in outer loop");

void PosMapRenderer::init(std::string data_dir,
                          FaceModelPtr model)
{
    programs_["main"] = GLProgram(data_dir + "shaders/subdiv.vert",
                                  data_dir + "shaders/subdiv.tc",
                                  data_dir + "shaders/subdiv.te",
                                  "",
                                  data_dir + "shaders/subdiv.frag",
                                  DrawMode::PATCHES);
    
    programs_["recon"] = GLProgram(data_dir + "shaders/posmap_recon.vert",
                                   data_dir + "shaders/posmap_recon.tc",
                                   data_dir + "shaders/posmap_recon.te",
                                   "",
                                   data_dir + "shaders/posmap_recon.frag",
                                   DrawMode::PATCHES);
    
    programs_["plane"] = GLProgram(data_dir + "shaders/full_texture_bgr.vert",
                                   data_dir + "shaders/full_texture_bgr.frag",
                                   DrawMode::TRIANGLES);
    
    auto& prog = programs_["main"];
    auto& prog_rec = programs_["recon"];
    auto& prog_pl = programs_["plane"];
    
    fb_ = Framebuffer::Create(FLAGS_pm_size, FLAGS_pm_size, 2); // will be resized based on frame size
    
    prog.createUniform("u_tessinner", DataType::FLOAT);
    prog.createUniform("u_tessouter", DataType::FLOAT);
    prog.createUniform("u_tessalpha", DataType::FLOAT);
    
    mesh_.init(prog, AT_POSITION | AT_NORMAL | AT_UV);
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog, AT_UV);
    
    prog_rec.createUniform("u_tessinner", DataType::FLOAT);
    prog_rec.createUniform("u_tessouter", DataType::FLOAT);
    
    mesh_.init(prog_rec, AT_UV);
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog_rec, AT_UV);
    Camera::initializeUniforms(prog_rec, U_CAMERA_MVP | U_CAMERA_MV);
    
    prog_rec.createTexture("u_sample_pos", fb_->color(0), fb_->width(), fb_->height());
    prog_rec.createTexture("u_sample_normal", fb_->color(1), fb_->width(), fb_->height());
    
    plane_.init(prog_pl,0.01);
    prog_pl.createTexture("u_texture", fb_->color(0), fb_->width(), fb_->height());
    
    tessInner_ = FLAGS_pm_tessin;
    tessOuter_ = FLAGS_pm_tessout;
}

void PosMapRenderer::render(const FaceData& fd)
{
    // render parameters update
    auto& prog = programs_["main"];
    auto& prog_pl = programs_["plane"];

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
    fb_->RetrieveFBO(FLAGS_pm_size, FLAGS_pm_size, out);
    
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
    
    std::vector<cv::Mat_<float>> xyz(3);
    cv::split(pos, xyz);
    double xmin,xmax,ymin,ymax,zmin,zmax;
    cv::minMaxLoc(xyz[0], &xmin, &xmax);
    cv::minMaxLoc(xyz[1], &ymin, &ymax);
    cv::minMaxLoc(xyz[2], &zmin, &zmax);
    
    xmin = -11;
    xmax = 11;
    ymin = -18;
    ymax = 18;
    zmin = -18;
    zmax = 8;
    
    for(int y=0; y<pos.rows; ++y)
    {
        for(int x=0; x<pos.cols; ++x)
        {
            auto& a = dIx(y,x);
            auto& b = dIy(y,x);
            
            auto c = b.cross(a);
            c = c / cv::norm(c);
            nml(y,x)[2] = 0.5*(c[0]+1.0);
            nml(y,x)[1] = 0.5*(c[1]+1.0);
            nml(y,x)[0] = 0.5*(c[2]+1.0);
            
            pos(y,x)[0] = (pos(y,x)[0]-xmin)/(xmax-xmin);
            pos(y,x)[1] = (pos(y,x)[1]-ymin)/(ymax-ymin);
            pos(y,x)[2] = (pos(y,x)[2]-zmin)/(zmax-zmin);
        }
    }
    
    cv::imshow("normal", nml);
    cv::imshow("pos", pos);
    cv::cvtColor(pos, pos, CV_RGB2BGR);
    cv::imwrite("pos.png",255.0*pos);
    cv::waitKey(1);
    
}

void PosMapRenderer::render(const Camera& camera, const FaceData& fd)
{
    // render parameters update
    auto& prog = programs_["main"];
    auto& prog_rec = programs_["recon"];
    
    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_UV);
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_tessalpha", tessAlpha_);
    
    prog_rec.setUniformData("u_tessinner", (float)tessInner_);
    prog_rec.setUniformData("u_tessouter", (float)tessOuter_);    
    camera.updateUniforms(prog_rec, fd.RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    // binding framebuffer
    fb_->Bind();
    
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw(wire_);
    fb_->Unbind();
    
    GLFWwindow* window = glfwGetCurrentContext();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog_rec.draw(wire_);
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
        render(result.camera, result.fd);
        //render(result.fd);
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
