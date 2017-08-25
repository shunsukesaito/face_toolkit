//
//  gl_mesh.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef gl_mesh_hpp
#define gl_mesh_hpp

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gl_core.hpp"
#include "gl_utils.h"

struct Mesh
{
    std::vector<glm::vec2> uvs_;
    std::vector<glm::vec3> pts_;
    std::vector<glm::vec3> nml_;
    
    void init(GLProgram&, const Eigen::VectorXf&, const Eigen::MatrixX3f&, const Eigen::Matrix2f&, const Eigen::MatrixX3i&, const Eigen::MatrixX3i&);
    void update(GLProgram&,const Eigen::VectorXf&, const Eigen::MatrixX3f&, const Eigen::MatrixX3i&);
};

#endif /* gl_mesh_hpp */
