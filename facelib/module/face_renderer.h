//
//  face_renderer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <map>
#include <chrono>

#include <gl_utility/gl_core.h>
#include <gl_utility/framebuffer.h>
#include <gl_utility/gl_mesh.h>
#include <gl_utility/camera.h>
#include <gl_utility/gl_utils.h>

#include <renderer/base_renderer.h>

#include "face_module.h"

struct FaceRenderer;
typedef std::shared_ptr<FaceRenderer> FaceRendererPtr;

struct FaceRenderer {
    std::string data_dir_;
    FaceModelPtr face_model_;
    
    std::map<std::string, RendererHandle> renderer_;
    
    FaceRenderer() {}
    ~FaceRenderer() {}
    
    void init(FaceModelPtr fm, std::string data_dir = "./");
    void addRenderer(std::string name, RendererHandle renderer);
    
    void draw(const FaceResult& result, int cam_id = 0, int frame_id = 0);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};
