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
    
    // sphere rendering
    glSphere ball_;
    
    void init(std::string data_dir,
              const Eigen::MatrixX3i& tri);
    
    void render(const Camera& camera,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml,
                bool draw_sphere = false);
    
    void render(const Camera& camera,
                const Eigen::Matrix4f& RT,
                const Eigen::VectorXf& pts,
                const Eigen::MatrixX3f& nml,
                bool draw_sphere = false);
};

#endif /* mesh_renderer_hpp */
