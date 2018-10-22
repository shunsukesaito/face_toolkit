#pragma once

#include <utility/EigenHelper.h>
#include <gl_utility/camera.h>
#include <shape_model/face_model.h>

#include "face_result.h"

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

struct BaseOptimizer;
typedef std::shared_ptr<BaseOptimizer> OptimizerHandle;

struct BaseOptimizer
{
    std::string name_ = "";
    
    BaseOptimizer() : name_(""){}
    BaseOptimizer(std::string name) : name_(name){}
    
    virtual void init(std::string data_dir, FaceModelPtr fm)
    { throw std::runtime_error( "BaseOptimizer::solve() base class is called..."); }
    
    virtual void solve(FaceResult& result)
    { throw std::runtime_error( "BaseOptimizer::solve() - base class is called..."); }
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI(){ throw std::runtime_error( "BaseOptimizer::updateIMGUI() base class is called..."); }
#endif
};
