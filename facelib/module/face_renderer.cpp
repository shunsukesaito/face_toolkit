//
//  face_renderer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "face_renderer.h"

#include <utility/obj_loader.h>

void FaceRenderer::init(FaceModelPtr fm, std::string data_dir)
{
    data_dir_ = data_dir;
    face_model_ = fm;
    
    for(auto&& r : renderer_)
        r.second->init(data_dir_, data_dir_ + "shaders", face_model_);
}

void FaceRenderer::addRenderer(std::string name, RendererHandle renderer)
{
    if(renderer_.find(name) != renderer_.end()){
        throw std::runtime_error("Attempted to create renderer with duplicate name " + name);
    }
    renderer_[name] = renderer;
}

void FaceRenderer::draw(const FaceResult& result, int cam_id, int frame_id)
{
    for(auto&& r : renderer_)
        r.second->render(result, cam_id, frame_id);
}

#ifdef WITH_IMGUI
void FaceRenderer::updateIMGUI()
{
    for(auto&& r : renderer_)
        r.second->updateIMGUI();
}
#endif

