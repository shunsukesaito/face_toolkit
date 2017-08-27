//
//  gl_core.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef gl_core_hpp
#define gl_core_hpp

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

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "gl_utils.h"

enum class DataType { UINT, FLOAT, VECTOR4, VECTOR3, VECTOR2, MATRIX44};
enum class DrawMode { TRIANGLES, POINTS };
class GLProgram;

struct GLAttribute {
    
    GLAttribute();
    GLAttribute(GLProgram* parentProgram_, std::string name_, DataType type_, bool dynamic_);
    
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
    void setData(const glm::vec3& val);
    void setData(float val);
    void setData(uint val);
};

struct GLTexture {
    
    GLTexture();
    GLTexture(GLProgram* parentProgram_, std::string name_, std::string sourceFile);
    GLTexture(GLProgram* parentProgram_, std::string name_, const cv::Mat& img);
    GLTexture(GLProgram* parentProgram_, std::string name_, GLuint location_, int w, int h);
    
    static GLuint CreateTexture(const cv::Mat& img);
    static void UpdateTexture(const cv::Mat& img, GLuint &_location);
    
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
    GLProgram(std::string vertShader, std::string geomShader, std::string fragShader, DrawMode drawMode_);
    GLProgram(std::string vertShader, std::string fragShader, DrawMode drawMode_);
    
    void draw(bool wire = false);
    
    void createAttribute(std::string attributeName, DataType type, bool dynamic);
    void createUniform(std::string uniformName, DataType type);
    void createTexture(std::string textureName, std::string sourceFile);
    void createTexture(std::string textureName, GLuint location, int w, int h);
    void updateTexture(std::string textureName, std::string sourceFile, bool flip = false);
    
    void setAttributeData(std::string attributeName, const std::vector<float> &vals);
    void setAttributeData(std::string attributeName, const std::vector<glm::vec4> &vals);
    void setAttributeData(std::string attributeName, const std::vector<glm::vec3> &vals);
    void setAttributeData(std::string attributeName, const std::vector<glm::vec2> &vals);
    void setUniformData(std::string uniformName, const Eigen::Matrix4f& val);
    void setUniformData(std::string uniformName, const glm::vec3& val);
    void setUniformData(std::string uniformName, float val);
    void setUniformData(std::string uniformName, uint val);
    
private:
    
    DrawMode drawMode;
    
    std::unordered_map<std::string, GLAttribute> attributeMap;
    std::unordered_map<std::string, GLUniform> uniformMap;
    std::unordered_map<std::string, GLTexture> textureMap;
    
    GLuint vertShaderHandle, geomShaderHandle, fragShaderHandle, programHandle, vaoHandle;
    
    long long validateAttributeSizes();
    void validateUniformSet();
    void activateTextures();
};

class Drawable {
    
public:
    
    virtual ~Drawable();
    virtual void draw() = 0;
};

#endif /* gl_core_hpp */
