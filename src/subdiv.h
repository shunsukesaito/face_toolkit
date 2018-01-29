//
//  subdiv.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 1/27/18.
//  Copyright © 2018 Shunsuke Saito. All rights reserved.
//

#ifndef subdiv_h
#define subdiv_h

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/primvarRefiner.h>

#include <stdio.h>

#include "gl_utils.h"

struct Vertex {
    
    // Minimal required interface ----------------------
    Vertex() { }
    
    Vertex(Vertex const & src) {
        _position[0] = src._position[0];
        _position[1] = src._position[1];
        _position[2] = src._position[2];
    }
    
    void Clear( void * =0 ) {
        _position[0]=_position[1]=_position[2]=0.0f;
    }
    
    void AddWithWeight(Vertex const & src, float weight) {
        _position[0]+=weight*src._position[0];
        _position[1]+=weight*src._position[1];
        _position[2]+=weight*src._position[2];
    }
    
    // Public interface ------------------------------------
    void SetPosition(float x, float y, float z) {
        _position[0]=x;
        _position[1]=y;
        _position[2]=z;
    }
    
    const float * GetPosition() const {
        return _position;
    }
    
private:
    float _position[3];
};

struct FVarVertexUV {
    
    // Minimal required interface ----------------------
    void Clear() {
        u=v=0.0f;
    }
    
    void AddWithWeight(FVarVertexUV const & src, float weight) {
        u += weight * src.u;
        v += weight * src.v;
    }
    
    // Basic 'uv' layout channel
    float u,v;
};

void performSubdiv(const Eigen::VectorXf& pts_src,
                   const Eigen::MatrixX2f& uvs_src,
                   const Eigen::MatrixX3i& triuv_src,
                   const Eigen::MatrixX3i& tripts_src,
                   Eigen::VectorXf& pts_dst,
                   Eigen::MatrixX2f& uvs_dst,
                   Eigen::MatrixX3i& triuv_dst,
                   Eigen::MatrixX3i& tripts_dst);

#endif /* subdiv_hpp */
