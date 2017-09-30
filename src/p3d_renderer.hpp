//
//  p3d_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef p3d_renderer_hpp
#define p3d_renderer_hpp

#include "EigenHelper.h"
#include "camera.hpp"
#include "gl_core.hpp"
#include "gl_mesh.hpp"

struct P3DRenderer
{
    std::unordered_map<std::string, GLProgram> programs_;
    glMesh p3d_;
    
    void init(std::string data_dir,
              const std::vector<Eigen::Vector3f>& pts);
    
    void render(const Camera& camera,
                const std::vector<Eigen::Vector3f>& pts);
    
    void render(const Camera& camera,
                const Eigen::Matrix4f& RT,
                const std::vector<Eigen::Vector3f>& pts);
};

#endif /* p3d_renderer_hpp */
