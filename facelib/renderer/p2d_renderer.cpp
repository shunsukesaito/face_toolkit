//
//  p2d_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "p2d_renderer.h"

void P2DRenderer::init(std::string shader_dir)
{
    programs_["p2d"] = GLProgram(shader_dir, "point2d.vert", "point2d.frag", DrawMode::POINTS);
    auto& prog = programs_["p2d"];
    
    p2d_.init(prog);
}

void P2DRenderer::render(int w, int h, const std::vector<Eigen::Vector2f>& pts)
{
    auto& prog = programs_["p2d"];
    
    p2d_.update(prog, w, h, pts, Eigen::Vector4f(0,1,0,1));
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    prog.draw();
}

void P2DRenderer::render(int w, int h, const std::vector<Eigen::Vector3f>& pts)
{
    auto& prog = programs_["p2d"];
    
    p2d_.update(prog, w, h, pts);
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    prog.draw();
}

#ifdef FACE_TOOLKIT
void P2DRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr fm)
{
    init(shader_dir);
}

void P2DRenderer::render(const FaceResult& result)
{
    if(show_)
        render(result.camera.width_, result.camera.height_, result.p2d);
}
#endif

#ifdef WITH_IMGUI
void P2DRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
    }
}
#endif

RendererHandle P2DRenderer::Create(std::string name, bool show)
{
    auto renderer = new P2DRenderer(name, show);
    
    return RendererHandle(renderer);
}
