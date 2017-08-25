//
//  framebuffer.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef framebuffer_hpp
#define framebuffer_hpp

#include <memory>
#include <vector>

#include <stdexcept>
#include <iostream>

#include <GL/glew.h>

#include "gl_utils.h"

class Framebuffer;
typedef std::shared_ptr<Framebuffer> FramebufferPtr;

// Describes a framebuffer object
class Framebuffer
{
    Framebuffer(unsigned int width, unsigned int height,bool with_color);
    Framebuffer(const Framebuffer &);
    const Framebuffer &operator =(const Framebuffer &);
    
public:
    ~Framebuffer();
    
    static FramebufferPtr Create(unsigned int width, unsigned int height, bool with_color);
    void AttachColorTexture();
    void ApplyBuffers();
    
    void Bind() const;
    void Unbind() const;
    void Resize(unsigned int width, unsigned int height, bool with_color);
    
    GLuint color(unsigned int idx) const { return colors_[idx]; }
    GLuint depth() const { return depth_; }
    
    unsigned int width() const {return width_;}
    unsigned int height() const {return height_;}
    
    // low-level handle
    unsigned int handle() const { return handle_; }
    
private:
    void init(unsigned int width, unsigned int height, bool with_color);
    void deinit();
    
    
    GLuint depth_;
    std::vector<GLuint> colors_;
    
    unsigned int width_;
    unsigned int height_;
    
    GLuint handle_;
};

#endif /* framebuffer_hpp */
