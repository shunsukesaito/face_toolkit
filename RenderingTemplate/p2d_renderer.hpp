//
//  p2d_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/14/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef p2d_renderer_hpp
#define p2d_renderer_hpp

#include "EigenHelper.h"
#include "gl_core.hpp"
#include "gl_mesh.hpp"

struct P2DRenderer
{
    std::unordered_map<std::string, GLProgram> programs_;
    glPoint2D p2d_;
    
    void init(std::string data_dir);
    
    void render(int w, int h, const std::vector<Eigen::Vector2f>& pts);
};

#endif /* p2d_renderer_hpp */
