//
//  mesh_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "mesh_renderer.hpp"

void MeshRenderer::init(std::string data_dir,
                        const Camera& camera,
                        const Eigen::VectorXf& pts,
                        const Eigen::MatrixX3f& nml,
                        const Eigen::MatrixX3i& tri_pts)
{
    programs_["mesh"] = GLProgram(data_dir + "shaders/mesh.vert",
                                  data_dir + "shaders/mesh.frag",
                                  DrawMode::TRIANGLES_IDX);
    
    auto& prog = programs_["mesh"];
    
    camera.intializeUniforms(prog, true, false);
    camera.updateUniforms(prog, true, false);
    
    mesh_.init_with_idx(prog, pts, nml, tri_pts);
}

void MeshRenderer::render(const Camera& camera,
                          const Eigen::VectorXf& pts,
                          const Eigen::MatrixX3f& nml)
{
    auto& prog = programs_["mesh"];
    
    camera.updateUniforms(prog, true, false);
    
    mesh_.update_with_idx(prog, pts, nml);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    prog.draw();
}

void MeshRenderer::render(const Camera& camera,
                          const Eigen::Matrix4f& RT,
                          const Eigen::VectorXf& pts,
                          const Eigen::MatrixX3f& nml)
{
    auto& prog = programs_["mesh"];
    
    camera.updateUniforms(prog, RT, true, false);
    
    mesh_.update_with_idx(prog, pts, nml);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    prog.draw();
}
