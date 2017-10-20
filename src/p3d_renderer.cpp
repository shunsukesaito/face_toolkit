//
//  p3d_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "p3d_renderer.h"

void P3DRenderer::init(std::string data_dir)
{
    programs_["p3d"] = GLProgram(data_dir + "shaders/point3d.vert",
                                  data_dir + "shaders/point3d.frag",
                                  DrawMode::POINTS);
    auto& prog = programs_["p3d"];
    
    Camera::initializeUniforms(prog, U_CAMERA_MVP);
    
    p3d_.init(prog, AT_POSITION | AT_COLOR);
}

void P3DRenderer::render(const Camera& camera,
                         const std::vector<Eigen::Vector3f>& pts)
{
    auto& prog = programs_["p3d"];
    
    camera.updateUniforms(prog, U_CAMERA_MVP);
    
    p3d_.update_position(pts);
    p3d_.update_color(Eigen::Vector4f(0,0,1,1));
    p3d_.update(prog, AT_POSITION | AT_COLOR);
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    prog.draw();
}

void P3DRenderer::render(const Camera& camera,
                         const Eigen::Matrix4f& RT,
                         const std::vector<Eigen::Vector3f>& pts,
                         const Eigen::Vector4f& color)
{
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
