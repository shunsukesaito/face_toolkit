//
//  p2d_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "p2d_renderer.hpp"

void P2DRenderer::init(std::string data_dir)
{
    programs_["p2d"] = GLProgram(data_dir + "shaders/point2d.vert",
                                 data_dir + "shaders/point2d.frag",
                                 DrawMode::POINTS);
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
