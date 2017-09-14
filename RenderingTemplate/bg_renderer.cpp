//
//  bg_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "bg_renderer.hpp"

void BGRenderer::init(std::string data_dir,
                      std::string img_path)
{
    programs_["bg"] = GLProgram(data_dir + "shaders/full_texture_bgr.vert",
                                data_dir + "shaders/full_texture_bgr.frag",
                                DrawMode::TRIANGLES);

    plane_.init(programs_["bg"],0.99999);
    programs_["bg"].createTexture("u_texture", img_path);
    
    programs_["bg"].createUniform("u_tex_mode", DataType::UINT);
    programs_["bg"].setUniformData("u_tex_mode", (uint)1);
}

void BGRenderer::init(std::string data_dir,
                      const cv::Mat& img)
{
    programs_["bg"] = GLProgram(data_dir + "shaders/full_texture_bgr.vert",
                                data_dir + "shaders/full_texture_bgr.frag",
                                DrawMode::TRIANGLES);
    
    plane_.init(programs_["bg"],0.99999);
    programs_["bg"].createTexture("u_texture", img);
    
    programs_["bg"].createUniform("u_tex_mode", DataType::UINT);
    programs_["bg"].setUniformData("u_tex_mode", (uint)1);
}

void BGRenderer::render(const cv::Mat& img, bool mirror)
{
    if(!img.empty()){
        programs_["bg"].updateTexture("u_texture", img);
        if(mirror)
            programs_["bg"].setUniformData("u_tex_mode", (uint)2);
    }
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    programs_["bg"].draw();
}
