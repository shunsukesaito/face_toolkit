/*
 MIT License
 
 Copyright (c) 2018 Shunsuke Saito
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gl_core.h"
#include "gl_utils.h"

// flags for attributes
static const int AT_POSITION = 0x0001;
static const int AT_NORMAL   = 0x0002;
static const int AT_COLOR    = 0x0004;
static const int AT_UV       = 0x0008;
static const int AT_TAN      = 0x0010;
static const int AT_TRI      = 0x0020;


struct glPlane
{
    std::vector<glm::vec2> uvs_;
    std::vector<glm::vec3> pts_;
    
    void init(GLProgram&, float z = 1.0);
};

struct glPoint2D
{
    std::vector<glm::vec2> pts_;
    std::vector<glm::vec4> clr_;
    
    void init(GLProgram& prog);
    
    void update(GLProgram& prog,
                int width,
                int height,
                const std::vector<Eigen::Vector2f>& pts,
                const Eigen::Vector4f& clr);
    
    void update(GLProgram& program,
                int width,
                int height,
                const std::vector<Eigen::Vector3f>& pts);
};

struct glSphere
{
    glSphere(){ generateSphere(10.0,24,48,true);}
    
    void generateSphere(float radius, unsigned int rings, unsigned int sectors, bool with_idx);
    void init(GLProgram& prog, int flag);
    void update(GLProgram& prog, int flag);
    
    std::vector<glm::vec2> uvs_;
    std::vector<glm::vec3> nml_;
    std::vector<glm::vec3> pts_;
    std::vector<glm::vec4> clr_;
    
    std::vector<unsigned int> tri_;
};

struct glMesh
{
    std::vector<glm::vec2> uvs_;
    std::vector<glm::vec3> pts_;
    std::vector<glm::vec3> nml_;
    std::vector<glm::vec4> clr_;
    std::vector<glm::vec3> tan_;
    std::vector<glm::vec3> btan_;
    
    std::vector<unsigned int> tri_;
    
    void init(GLProgram& prog, int flag);
    void update(GLProgram& prog, int flag);
    
    void update_position(const Eigen::VectorXf& pts,
                         const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_position(const std::vector<Eigen::Vector3f>& pts,
                         const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_normal(const Eigen::VectorXf& nml,
                       const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());

    void update_normal(const Eigen::MatrixX3f& nml,
                       const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_color(const Eigen::VectorXf& clr,
                      const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_color(const Eigen::Vector4f& clr,
                      const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());
    
    void update_tangent(const Eigen::MatrixX3f& tan,
                        const Eigen::MatrixX3f& btan,
                        const Eigen::MatrixX3i& tri = Eigen::MatrixX3i());

    void update_uv(const Eigen::MatrixX2f& uvs,
                   const Eigen::MatrixX3i& tri_uv,
                   const Eigen::MatrixX3i& tri_pts = Eigen::MatrixX3i());
    
    void update_tri(const Eigen::MatrixX3i& tri);
    void update_tri(const std::vector<unsigned int>& tri);
};
