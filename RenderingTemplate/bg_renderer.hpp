//
//  bg_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef bg_renderer_hpp
#define bg_renderer_hpp

#include "gl_core.hpp"
#include "gl_mesh.hpp"

struct BGRenderer
{
    std::unordered_map<std::string, GLProgram> programs_;
    glPlane plane_;
    int width_, height_;
    
    void init(std::string data_dir,
              std::string img_path);
    
    void init(std::string data_dir,
              const cv::Mat& img);

    void render(const cv::Mat& img = cv::Mat(), bool mirror = false);
};


#endif /* bg_renderer_hpp */
