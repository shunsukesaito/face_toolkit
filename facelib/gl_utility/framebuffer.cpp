//
//  framebuffer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "framebuffer.h"

// Constructor requires at least one of the textures (color or depth) to be valid.
Framebuffer::Framebuffer(unsigned int width, unsigned int height, int color_size, int tex_filter)
: width_(width), height_(height), handle_(-1), depth_(-1), tex_filter_(tex_filter)
{
    init(width, height, color_size);
}

Framebuffer::~Framebuffer()
{
    Unbind();
    glDeleteFramebuffers(1, &handle_);
    
    for(int i = 0; i < colors_.size(); ++i)
        glDeleteTextures(1, &colors_[i]);
    glDeleteTextures(1, &depth_);
}

FramebufferPtr Framebuffer::Create(unsigned int width, unsigned int height, int color_size, int tex_filter)
{
    return FramebufferPtr(new Framebuffer(width, height, color_size, tex_filter));
}

void Framebuffer::AttachColorTexture()
{
    Bind();
    
    GLuint color;
    
    unsigned int colorIdx = static_cast<unsigned int>(colors_.size());
    
    glGenTextures(1, &color);
    glBindTexture(GL_TEXTURE_2D, color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width_, height_, 0, GL_RGBA, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, tex_filter_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, tex_filter_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
    
    colors_.push_back(color);
    
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + colorIdx, GL_TEXTURE_2D, color, 0);
    
    glReadBuffer(GL_COLOR_ATTACHMENT0 + colorIdx);
    
    Unbind();
}

void Framebuffer::RetrieveFBO(int w, int h, std::vector<cv::Mat>& mat)
{
    if(mat.size() != colors_.size())
        mat.resize(colors_.size());
    
    for(int i = 0; i < colors_.size(); ++i)
    {
        RetrieveFBO(w, h, mat[i], i);
    }
}

void Framebuffer::RetrieveFBO(int w, int h, std::vector<cv::Mat_<cv::Vec4f>>& mat)
{
    if(mat.size() != colors_.size())
        mat.resize(colors_.size());
    
    for(int i = 0; i < colors_.size(); ++i)
    {
        RetrieveFBO(w, h, mat[i], i);
    }
}

void Framebuffer::RetrieveFBO(std::vector<cv::Mat>& mat)
{
    if(mat.size() != colors_.size())
        mat.resize(colors_.size());
    
    for(int i = 0; i < colors_.size(); ++i)
    {
        RetrieveFBO(mat[i], i);
    }
}

void Framebuffer::RetrieveFBO(std::vector<cv::Mat_<cv::Vec4f>>& mat)
{
    if(mat.size() != colors_.size())
        mat.resize(colors_.size());
    
    for(int i = 0; i < colors_.size(); ++i)
    {
        RetrieveFBO(mat[i], i);
    }
}

void Framebuffer::RetrieveFBO(int w, int h, cv::Mat& mat, int attachID)
{
    assert(w <= width_ && h <= height_);
    if(mat.rows != h || mat.cols != w)
        mat.create(h, w, mat.type());
    
    Bind();
    
    CHECK_GL_ERROR();
    
    glViewport(0, 0, w, h);
    
    CHECK_GL_ERROR();
    
    glReadBuffer(GL_COLOR_ATTACHMENT0 + attachID);
    
    CHECK_GL_ERROR();
    
    // This is needed because RGB buffer is not 32-bit aligned
    
    //use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_PACK_ALIGNMENT, (mat.step & 3) ? 1 : 4);
    
    //set length of one complete row in destination data (doesn't need to equal img.cols)
    glPixelStorei(GL_PACK_ROW_LENGTH, mat.step/mat.elemSize());
    
    // Warning: This is a bit adhoc.
    GLint rgbmode;
    if (mat.channels() == 4)
    {
        rgbmode = GL_RGBA;
    }
    else if (mat.channels() == 3)
    {
        rgbmode = GL_BGR;
    }
    else if (mat.channels() == 1)
    {
        rgbmode = GL_DEPTH_COMPONENT;
    }
    else
    {
        throw std::runtime_error("Error, texture formap unknown.");
    }
    
    GLint type;
    if ((mat.type() & CV_MAT_DEPTH_MASK) == CV_8U)
    {
        type = GL_UNSIGNED_BYTE;
    }
    else if ((mat.type() & CV_MAT_DEPTH_MASK) == CV_32F)
    {
        type = GL_FLOAT;
    }
    else
    {
        throw std::runtime_error("Error, texture type unknown.");
    }
    
    glReadPixels(0, 0, w, h, rgbmode, type, mat.data );
    CHECK_GL_ERROR();
    cv::flip(mat, mat, 0);
    
    Unbind();
}

void Framebuffer::RetrieveFBO(cv::Mat& mat, int attachID)
{
    RetrieveFBO(width_, height_, mat, attachID);
}

void Framebuffer::ApplyBuffers()
{
    Bind();
    if(colors_.size() == 0)
    {
        glDrawBuffers(0,NULL);
        glReadBuffer(GL_NONE);
    }
    else
    {
        GLenum* attachments = new GLenum[colors_.size()];
        for(int i = 0; i < colors_.size(); ++i)
            attachments[i] = GL_COLOR_ATTACHMENT0 + i;
        
        glDrawBuffers((GLsizei)colors_.size(), attachments);
        
        GLenum ret = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if(ret != GL_FRAMEBUFFER_COMPLETE)
        {
            printf("GL_FRAMEBUFFER NOT COMPLETE! Status: 0x%x\n",ret);
            throw std::runtime_error("GL_FRAMEBUFFER NOT COMPLETE!");

        }
        delete[] attachments;
    }
    Unbind();
}

void Framebuffer::Bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, handle_);
}

void Framebuffer::Unbind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::init(unsigned int width, unsigned int height, int color_size)
{
    if(handle_ == -1)
        glGenFramebuffers(1, &handle_);
    
    width_ = width;
    height_ = height;
    
    Bind();
    // add color attachment and query resolution from any of the available textures    
    if(depth_ == -1)
        glGenTextures(1, &depth_);
    glBindTexture(GL_TEXTURE_2D, depth_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, tex_filter_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, tex_filter_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    CHECK_GL_ERROR();
    
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_, 0);
    //glFramebufferTextureEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, depth_, 0);
    //glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_, 0);
    CHECK_GL_ERROR();
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    GLenum ret = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(ret != GL_FRAMEBUFFER_COMPLETE)
    {
        printf("GL_FRAMEBUFFER NOT COMPLETE! Status: 0x%x\n",ret);
        throw std::runtime_error("GL_FRAMEBUFFER NOT COMPLETE!");
    }
    Unbind();
    
    for(int i = 0; i < colors_.size(); ++i)
        glDeleteTextures(1, &colors_[i]);
        
    colors_.clear();
    
    for(int i = 0; i < color_size; ++i)
        AttachColorTexture();
    
    ApplyBuffers();
    CHECK_GL_ERROR();
}

void Framebuffer::Resize(unsigned int width, unsigned int height, int color_size)
{
    Unbind();
    
    init(width, height, color_size);
}
