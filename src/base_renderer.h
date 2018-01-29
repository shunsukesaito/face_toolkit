//
//  base_renderer.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 12/4/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef base_renderer_h
#define base_renderer_h

#include "EigenHelper.h"
#include "camera.h"
#include "gl_core.h"
#include "gl_mesh.h"

#ifdef FACE_TOOLKIT
#include "face_model.h"
#include "face_result.h"
#endif

#ifdef WITH_IMGUI
#include "imgui.h"
#endif

struct BaseRenderer;
typedef std::shared_ptr<BaseRenderer> RendererHandle;

struct BaseRenderer
{
    std::unordered_map<std::string, GLProgram> programs_;
    std::string name_;
    bool show_ = false;
    bool wire_ = false;
    
    BaseRenderer() : name_(""), show_(false), wire_(false){}
    BaseRenderer(std::string name, bool show, bool wire = false) : name_(name), show_(show), wire_(wire){}
    
#ifdef FACE_TOOLKIT
    virtual void init(std::string data_dir, FaceModelPtr fm){ throw std::runtime_error( "Error: Base class is called..."); }
    virtual void render(const FaceResult& result){ throw std::runtime_error( "Error: Base class is called..."); }
#endif
    
#ifdef WITH_IMGUI
    virtual inline void updateIMGUI(){ throw std::runtime_error( "Error: Base class is called..."); }
#endif
};

#endif /* base_renderer_hpp */
