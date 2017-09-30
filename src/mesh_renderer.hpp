//
//  mesh_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef mesh_renderer_hpp
#define mesh_renderer_hpp

#include "EigenHelper.h"
#include "camera.hpp"
#include "gl_core.hpp"
#include "gl_mesh.hpp"

struct MeshRenderer
{
    std::unordered_map<std::string, GLProgram> programs_;
    glMesh mesh_;
    
    void init(std::string data_dir,
              const Eigen::VectorXf& pts,
              const Eigen::MatrixX3f& nml,
              const Eigen::MatrixX3i& tri_pts);
    
    void render(const Camera& camera,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml);
    
    void render(const Camera& camera,
                const Eigen::Matrix4f& RT,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml);
};

#endif /* mesh_renderer_hpp */
