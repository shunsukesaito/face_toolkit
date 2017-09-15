//
//  p3d_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "p3d_renderer.hpp"

void P3DRenderer::init(std::string data_dir,
                       const Camera& camera,
                       const std::vector<Eigen::Vector3f>& pts)
{
    programs_["p3d"] = GLProgram(data_dir + "shaders/point3d.vert",
                                  data_dir + "shaders/point3d.frag",
                                  DrawMode::POINTS);
    auto& prog = programs_["p3d"];
    
    camera.intializeUniforms(prog, false, false);
    camera.updateUniforms(prog, false, false);
    
    p3d_.init(prog, pts, Eigen::Vector4f(0,0,1,1));
}

void P3DRenderer::render(const Camera& camera,
                         const std::vector<Eigen::Vector3f>& pts)
{
    auto& prog = programs_["p3d"];
    
    camera.updateUniforms(prog, false, false);
    
    p3d_.update(prog, pts);
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    prog.draw();
}

void P3DRenderer::render(const Camera& camera,
                         const Eigen::Matrix4f& RT,
                         const std::vector<Eigen::Vector3f>& pts)
{
    auto& prog = programs_["p3d"];
    
    camera.updateUniforms(prog, RT, false, false);
    
    p3d_.update(prog, pts, Eigen::Vector4f(0,0,1,1));
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    prog.draw();
}
