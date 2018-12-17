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

#include <utility/EigenHelper.h>
#include <gl_utility/camera.h>
#include <shape_model/face_model.h>

#include "face_result.h"

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

struct BaseOptimizer;
typedef std::shared_ptr<BaseOptimizer> OptimizerHandle;

struct BaseOptParams;
typedef std::shared_ptr<BaseOptParams> OptParamHandle;

struct BaseOptParams
{
    
};

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
