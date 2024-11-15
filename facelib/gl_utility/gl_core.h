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

#include <iostream>
#include <fstream>
#include <vector>

#include <GL/glew.h>

#if defined(__APPLE__)
#define GLFW_INCLUDE_GLCOREARB
#else
#define GL_GLEXT_PROTOTYPES
#endif

#include <GLFW/glfw3.h>

#include <unordered_map>

#define GLM_ENABLE_EXPERIMENTAL 
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <tinyexr.h>
#include <hdrloader.h>

#include "gl_utils.h"


enum class DataType { UINT, FLOAT, VECTOR4, VECTOR3, VECTOR2, MATRIX44, INDEX};
enum class DrawMode { TRIANGLES, TRIANGLES_IDX, POINTS, POINTS_IDX, PATCHES, PATCHES_IDX};
class GLProgram;

struct GLAttribute {
    
    GLAttribute();
    GLAttribute(GLProgram* parentProgram_, std::string name_, DataType type_, bool dynamic_);
    GLAttribute(GLProgram* parentProgram_);
    
    // member variables
    GLProgram* parentProgram;
    std::string name;
    DataType type;
    bool dynamic;
    GLuint location, VBO;
    long long dataSize = -1;
    
    void setData(const std::vector<glm::vec4>& data);
    void setData(const std::vector<glm::vec3>& data);
    void setData(const std::vector<glm::vec2>& data);    
    void setData(const std::vector<float>& data);
    void setData(const std::vector<unsigned int>& data);
};

struct GLUniform {
    
    GLUniform();
    GLUniform(GLProgram* parentProgram_, std::string name, DataType type_);
    
    // member variables
    GLProgram* parentProgram;
    std::string name;
    DataType type;
    GLuint location;
    bool hasBeenSet = false;
    
    void setData(const Eigen::Matrix4f& val);
    void setData(const glm::vec2& val);
    void setData(const glm::vec3& val);
    void setData(const glm::vec4& val);
    void setData(float val);
    void setData(uint val);
    
    void setData(const std::vector<Eigen::Matrix4f>& val);
    void setData(const std::vector<glm::vec3>& val);
    void setData(const std::vector<glm::vec4>& val);
    void setData(const std::vector<float>& val);
    void setData(const std::vector<uint>& val);
};

struct GLTexture {
    
    GLTexture();
    GLTexture(GLProgram* parentProgram_, std::string name_, std::string sourceFile);
    GLTexture(GLProgram* parentProgram_, std::string name_, const cv::Mat& img);
    GLTexture(GLProgram* parentProgram_, std::string name_, GLuint location_, int w, int h);
    
    static GLuint CreateTexture(const cv::Mat& img, bool use_mipmap = false);
    static GLuint CreateTexture(const HDRLoaderResult& img);
    static GLuint CreateTexture(const TinyExrImage& img);
    
    void UpdateTexture(const cv::Mat& img);
    
    // member variables
    GLProgram* parentProgram;
    std::string name;
    std::string sourceFile;
    GLuint location;
    int width, height;
    cv::Mat imageData;
};

class GLProgram {
    friend struct GLAttribute;
    friend struct GLUniform;
    friend struct GLTexture;
    
public:
    GLProgram() {}
    GLProgram(std::string root_dir,
              std::string vertShader,
              std::string tcShader,
              std::string teShader,
              std::string geomShader,
              std::string fragShader,
              DrawMode drawMode_);
    GLProgram(std::string root_dir,
              std::string vertShader,
              std::string geomShader,
              std::string fragShader,
              DrawMode drawMode_);
    GLProgram(std::string root_dir,
              std::string vertShader,
              std::string fragShader,
              DrawMode drawMode_);
    
    void draw(bool wire = false);
    
    void createElementIndex(const std::vector<unsigned int>& vals);
    void createAttribute(std::string attributeName, DataType type, bool dynamic);
    void createUniform(std::string uniformName, DataType type);
    void createTexture(std::string textureName, std::string sourceFile);
    void createTexture(std::string textureName, GLuint location, int w, int h);
    void createTexture(std::string textureName, const cv::Mat& img);
    void updateTexture(std::string textureName, std::string sourceFile, bool flip = false);
    void updateTexture(std::string textureName, GLuint location);
    void updateTexture(std::string textureName, const cv::Mat& img);
    void updateElementIndex(const std::vector<unsigned int>& vals);
    
    template <class T>
    void setAttributeData(std::string attributeName, const std::vector<T> &vals)
    {
        if(attributeMap.find(attributeName) == attributeMap.end()){
            throw std::runtime_error("Attempted to set attribute which does not exist: " + attributeName);
        }
        
        attributeMap[attributeName].setData(vals);
    }
    template <class T>
    void setUniformData(std::string uniformName, const T& val){
        if(uniformMap.find(uniformName) == uniformMap.end()){
            throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
        }
        uniformMap[uniformName].setData(val);
    }
    
    inline void setDrawMode(DrawMode mode){ drawMode = mode;}
private:
    
    DrawMode drawMode;
    
    std::unordered_map<std::string, GLAttribute> attributeMap;
    std::unordered_map<std::string, GLUniform> uniformMap;
    std::unordered_map<std::string, GLTexture> textureMap;
    
    GLuint vsHandle, tcsHandle, tesHandle, gsHandle, fsHandle, programHandle, vaoHandle;
    
    long long validateAttributeSizes();
    void validateUniformSet();
    void activateTextures();
};

class Drawable {
    
public:
    
    virtual ~Drawable();
    virtual void draw() = 0;
};
