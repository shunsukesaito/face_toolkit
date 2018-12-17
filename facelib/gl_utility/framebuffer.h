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

#include <memory>
#include <vector>

#include <stdexcept>
#include <iostream>

#include <GL/glew.h>
#include <opencv2/opencv.hpp>

#include "gl_utils.h"

class Framebuffer;
typedef std::shared_ptr<Framebuffer> FramebufferPtr;

// Describes a framebuffer object
class Framebuffer
{
    Framebuffer(unsigned int width, unsigned int height,int color_size,int tex_filter);
    Framebuffer(const Framebuffer &);
    const Framebuffer &operator =(const Framebuffer &);
    
public:
    ~Framebuffer();
    
    static FramebufferPtr Create(unsigned int width, unsigned int height, int color_size, int tex_filter=GL_NEAREST);
    void AttachColorTexture();
    void ApplyBuffers();
    
    void Bind() const;
    void Unbind() const;
    void Resize(unsigned int width, unsigned int height, int color_size);
    
    void RetrieveFBO(std::vector<cv::Mat>& mat);
    void RetrieveFBO(std::vector<cv::Mat_<cv::Vec4f>>& mat);
    void RetrieveFBO(cv::Mat& mat, int attachID);
    
    void RetrieveFBO(int w, int h, std::vector<cv::Mat>& mat);
    void RetrieveFBO(int w, int h, std::vector<cv::Mat_<cv::Vec4f>>& mat);
    void RetrieveFBO(int w, int h, cv::Mat& mat, int attachID);
    
    GLuint color(unsigned int idx) const { return colors_[idx]; }
    GLuint depth() const { return depth_; }
    
    unsigned int width() const {return width_;}
    unsigned int height() const {return height_;}
    
    // low-level handle
    unsigned int handle() const { return handle_; }
    
private:
    void init(unsigned int width, unsigned int height, int color_size);
    void deinit();
    
    GLuint depth_;
    std::vector<GLuint> colors_;
    GLint tex_filter_;
    
    unsigned int width_;
    unsigned int height_;
    
    GLuint handle_;
};
