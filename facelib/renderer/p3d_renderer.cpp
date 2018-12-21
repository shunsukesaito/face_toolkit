//
//  p3d_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "p3d_renderer.h"

void P3DRenderer::init(std::string shader_dir)
{
    programs_["p3d"] = GLProgram(shader_dir, "point3d.vert", "point3d.frag", DrawMode::POINTS);
    auto& prog = programs_["p3d"];
    
    Camera::initializeUniforms(prog, U_CAMERA_MVP);
    
    p3d_.init(prog, AT_POSITION | AT_COLOR);
}

void P3DRenderer::render(const Camera& camera,
                         const std::vector<Eigen::Vector3f>& pts,
                         const Eigen::Matrix4f& RT,
                         const Eigen::Vector4f& color)
{
    if(!show_)
        return;
    
    auto& prog = programs_["p3d"];
    
    camera.updateUniforms(prog, RT, U_CAMERA_MVP);

    p3d_.update_position(pts);
    p3d_.update_color(color);
    p3d_.update(prog, AT_POSITION | AT_COLOR);
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    prog.draw();
}

#ifdef FACE_TOOLKIT
void P3DRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr fm)
{
    init(shader_dir);
}

void P3DRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    render(result.cameras[cam_id], getP3DFromP2PC(result.fd[frame_id].pts_,  result.c_p2p), result.fd[frame_id].RT());
    render(result.cameras[cam_id], getP3DFromP2LC(result.fd[frame_id].pts_, result.c_p2l), result.fd[frame_id].RT());
}
#endif

#ifdef WITH_IMGUI
void P3DRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
    }
}
#endif

RendererHandle P3DRenderer::Create(std::string name, bool show)
{
    auto renderer = new P3DRenderer(name, show);
    
    return RendererHandle(renderer);
}
