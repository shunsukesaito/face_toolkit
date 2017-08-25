//
//  framebuffer.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "framebuffer.hpp"

// Constructor requires at least one of the textures (color or depth) to be valid.
Framebuffer::Framebuffer(unsigned int width, unsigned int height, bool with_color)
: width_(width), height_(height), handle_(-1), depth_(-1)
{
    init(width, height, with_color);
}

Framebuffer::~Framebuffer()
{
    Unbind();
    glDeleteFramebuffers(1, &handle_);
    
    for(int i = 0; i < colors_.size(); ++i)
        glDeleteTextures(1, &colors_[i]);
    glDeleteTextures(1, &depth_);
}

FramebufferPtr Framebuffer::Create(unsigned int width, unsigned int height, bool with_color)
{
    return FramebufferPtr(new Framebuffer(width, height,with_color));
}

void Framebuffer::AttachColorTexture()
{
    Bind();
    
    GLuint color;
    
    unsigned int colorIdx = static_cast<unsigned int>(colors_.size());
    
    glGenTextures(1, &color);
    glBindTexture(GL_TEXTURE_2D, color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width_, height_, 0, GL_RGBA, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    
    colors_.push_back(color);
    
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + colorIdx, GL_TEXTURE_2D, color, 0);
    
    glReadBuffer(GL_COLOR_ATTACHMENT0 + colorIdx);
    
    Unbind();
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

void Framebuffer::init(unsigned int width, unsigned int height, bool with_color)
{
    if(handle_ == -1)
        glGenFramebuffers(1, &handle_);
    std::cout << "Frame Buffer " << handle_ << std::endl;
    
    width_ = width;
    height_ = height;
        Bind();
    
    // add color attachment and query resolution from any of the available textures    
    if(depth_ == -1)
        glGenTextures(1, &depth_);
    glBindTexture(GL_TEXTURE_2D, depth_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width_, height_, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    CHECK_GL_ERROR();
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_, 0);
    CHECK_GL_ERROR();
    glDrawBuffer(NULL);
    GLenum ret = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(ret != GL_FRAMEBUFFER_COMPLETE)
    {
        printf("GL_FRAMEBUFFER NOT COMPLETE! Status: 0x%x\n",ret);
    }
    Unbind();
    
    if(with_color){
        
        for(int i = 0; i < colors_.size(); ++i)
            glDeleteTextures(1, &colors_[i]);
        
        colors_.clear();
        
        AttachColorTexture();
    }
    
    ApplyBuffers();
    CHECK_GL_ERROR();
}

void Framebuffer::Resize(unsigned int width, unsigned int height, bool with_color)
{
    Unbind();
    
    init(width, height, with_color);
}
