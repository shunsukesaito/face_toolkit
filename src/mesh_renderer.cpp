//
//  mesh_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "mesh_renderer.h"

void MeshRenderer::init(std::string data_dir,
                        const Eigen::MatrixX3i& tri)
{
    programs_["mesh"] = GLProgram(data_dir + "shaders/mesh.vert",
                                  data_dir + "shaders/mesh.tc",
                                  data_dir + "shaders/mesh.te",
                                  "",
                                  data_dir + "shaders/mesh.frag",
                                  DrawMode::PATCHES_IDX);
//    programs_["mesh"] = GLProgram(data_dir + "shaders/mesh.vert",
//                                  data_dir + "shaders/mesh.frag",
//                                  DrawMode::TRIANGLES_IDX);
    
    auto& prog = programs_["mesh"];
    
    prog.createUniform("u_tessinner", DataType::FLOAT);
    prog.createUniform("u_tessouter", DataType::FLOAT);
    prog.createUniform("u_tessalpha", DataType::FLOAT);
    
    Camera::initializeUniforms(prog, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update_tri(tri);
    mesh_.init(prog, AT_POSITION | AT_NORMAL | AT_TRI);
}

void MeshRenderer::render(const Camera& camera,
                          const Eigen::VectorXf& pts,
                          const Eigen::MatrixX3f& nml)
{
    auto& prog = programs_["mesh"];
    
    camera.updateUniforms(prog, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update_position(pts);
    mesh_.update_normal(nml);
    mesh_.update(prog, AT_POSITION | AT_NORMAL);
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_tessalpha", tessAlpha_);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    prog.draw(wire_);
}

void MeshRenderer::render(const Camera& camera,
                          const Eigen::Matrix4f& RT,
                          const Eigen::VectorXf& pts,
                          const Eigen::MatrixX3f& nml)
{
    auto& prog = programs_["mesh"];
    
    camera.updateUniforms(prog, RT, U_CAMERA_MVP | U_CAMERA_MV);
    
    mesh_.update_position(pts);
    mesh_.update_normal(nml);
    mesh_.update(prog, AT_POSITION | AT_NORMAL | AT_TRI);
    
    prog.setUniformData("u_tessinner", (float)tessInner_);
    prog.setUniformData("u_tessouter", (float)tessOuter_);
    prog.setUniformData("u_tessalpha", tessAlpha_);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw(wire_);
}

#ifdef FACE_TOOLKIT
void MeshRenderer::init(std::string data_dir, FaceModelPtr fm)
{
    init(data_dir,fm->tri_pts_);
}

void MeshRenderer::render(const FaceResult& result)
{
    if(show_)
        render(result.camera, result.fd.RT, result.fd.pts_, result.fd.nml_);
}
#endif

#ifdef WITH_IMGUI
void MeshRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::InputInt("TessInner", &tessInner_);
        ImGui::InputInt("TessOuter", &tessOuter_);
        ImGui::SliderFloat("TessAlpha", &tessAlpha_, 0.0, 1.0);
    }
}
#endif

RendererHandle MeshRenderer::Create(std::string name, bool show)
{
    auto renderer = new MeshRenderer(name, show);
    
    return RendererHandle(renderer);
}
