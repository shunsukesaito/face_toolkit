//
//  gl_core.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "gl_core.hpp"

using std::cerr;
using std::cout;
using std::endl;

const char *fragment_depth_shader_code = "\
#version 330 core\n\
//in VertexData {\n\
vec4 normal;\n\
vec4 pos;\n\
vec2 proj_texcoord;\n\
vec2 texcoord;\n\
} VertexIn;\n\
void main()\n\
{\n\
}\n\
";

static char* textFileRead(const char* fileName)
{
    char* text;
    
    if (fileName != NULL) {
        FILE *file = fopen(fileName, "rt");
        
        if (file != NULL) {
            fseek(file, 0, SEEK_END);
            int count = ftell(file);
            rewind(file);
            
            if (count > 0) {
                text = (char*)malloc(sizeof(char) * (count + 1));
                count = fread(text, sizeof(char), count, file);
                text[count] = '\0';    //cap off the string with a terminal symbol, fixed by Cory
            }
            fclose(file);
        }
    }
    return text;
}

void printShaderInfoLog(GLuint shaderHandle)
{
    int logLen = 0;
    int chars = 0;
    GLint isCompiled = 0;
    char *log;
    
    glGetShaderiv(shaderHandle, GL_INFO_LOG_LENGTH, &logLen);
    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &isCompiled);
    
    if(isCompiled == GL_FALSE){
        log = (char *)malloc(logLen);
        glGetShaderInfoLog(shaderHandle, logLen, &chars, log);
        printf("Shader info log:\n%s\n", log);
        free(log);
        throw std::runtime_error("Shader compilation failed");
    }
}

void printProgramInfoLog(GLuint handle)
{
    int logLen = 0;
    int chars = 0;
    char *log;
    GLint validate = 0;
    
    glGetProgramiv(handle, GL_INFO_LOG_LENGTH, &logLen);
    glGetProgramiv(handle, GL_VALIDATE_STATUS, &validate);
    CHECK_GL_ERROR();
    if(validate == GL_FALSE){
        log = (char *)malloc(logLen);
        glGetProgramInfoLog(handle, logLen, &chars, log);
        printf("Program info log:\n%s\n", log);
        free(log);
    }
}

GLProgram::GLProgram(std::string vertShader, std::string fragShader, DrawMode drawMode_)
: GLProgram(vertShader, "", fragShader, drawMode_)
{
}


GLProgram::GLProgram(std::string vertShader, std::string geomShader, std::string fragShader, DrawMode drawMode_)
: drawMode(drawMode_)
{
    vertShaderHandle = glCreateShader(GL_VERTEX_SHADER);
    fragShaderHandle = glCreateShader(GL_FRAGMENT_SHADER);
    CHECK_GL_ERROR();
    
    const char *vertShaderTmp = textFileRead(vertShader.c_str());
    glShaderSource(vertShaderHandle, 1, &vertShaderTmp, nullptr);
    glCompileShader(vertShaderHandle);
    printShaderInfoLog(vertShaderHandle);
    
    if(!geomShader.empty()){
        geomShaderHandle = glCreateShader(GL_GEOMETRY_SHADER);
        const char *geomShaderTmp = textFileRead(geomShader.c_str());
        glShaderSource(geomShaderHandle, 1, &geomShaderTmp, nullptr);
        glCompileShader(geomShaderHandle);
        printShaderInfoLog(geomShaderHandle);
    }
    
    const char *fragShaderTmp = textFileRead(fragShader.c_str());
    glShaderSource(fragShaderHandle, 1, &fragShaderTmp, nullptr);
    glCompileShader(fragShaderHandle);
    printShaderInfoLog(fragShaderHandle);
    
    programHandle = glCreateProgram();
    glAttachShader(programHandle, vertShaderHandle);
    if(!geomShader.empty()){
        glAttachShader(programHandle, geomShaderHandle);
    }
    glAttachShader(programHandle, fragShaderHandle);

    // link the program
    glLinkProgram(programHandle);
    GLint linked;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &linked);
    if(!linked)
        printProgramInfoLog(programHandle);
    glUseProgram(programHandle);
    
    // create a VAO for use with all buffers
    // TODO: we could generalize this to share buffers
    glGenVertexArrays(1, &vaoHandle);
    glBindVertexArray(vaoHandle);
}

void GLProgram::draw(bool wire)
{
    validateUniformSet();
    long long dataLen = validateAttributeSizes();
    
    glUseProgram(programHandle);
    glBindVertexArray(vaoHandle);
    activateTextures();
    
    glPolygonMode(GL_FRONT_AND_BACK, (wire ? GL_LINE : GL_FILL));
    
    switch(drawMode)
    {
        case DrawMode::TRIANGLES:
            glDrawArrays(GL_TRIANGLES, 0, (GLsizei)dataLen);
            break;
        case DrawMode::TRIANGLES_IDX:
            glDrawElements(GL_TRIANGLES, attributeMap["index"].dataSize, GL_UNSIGNED_INT, NULL);
            break;
        case DrawMode::POINTS:
            glDrawArrays(GL_POINTS, 0, (GLsizei)dataLen);
            break;
    }
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void GLProgram::activateTextures()
{
    int texN = 0;
    for(auto&& entry : textureMap)
    {
        glActiveTexture(GL_TEXTURE0 + texN);
        glBindTexture(GL_TEXTURE_2D, entry.second.location);
        glUniform1i(glGetUniformLocation(programHandle, entry.second.name.c_str()), texN++);
    }
}

long long GLProgram::validateAttributeSizes()
{
    long long allSizes = -777;
    bool allGood = true;
    for(auto&& entry : attributeMap)
    {
        if(entry.first == "index") continue;
        if(entry.second.dataSize == -1){
            cerr << "GL ERROR: Attribute " << entry.second.name << " was never set." << endl;
            allGood = false;
        }
        else {
            if(allSizes == -777) {
                allSizes = entry.second.dataSize;
            }
            else if(allSizes != entry.second.dataSize) {
                cerr << "GL ERROR: Attribute " << entry.second.name << " has a size with disagrees with other attributes." << endl;
                allGood = false;
            }
        }
    }
    
    if(!allGood){
        cerr << endl << "Attribute data sizes:" << endl;
        for(auto&& entry : attributeMap) {
            cerr << "  -- " << entry.second.name << "  =  " << entry.second.dataSize << endl;
        }
        throw std::runtime_error("Attribute validation failed");
    }
    
    return allSizes;
}

void GLProgram::validateUniformSet()
{
    bool allGood = true;
    
    for(auto&& entry : uniformMap)
    {
        if(!entry.second.hasBeenSet) {
            cerr << "GL ERROR: Uniform " << entry.second.name << " was never set." << endl;
            allGood = false;
        }
    }
    
    if(!allGood){
        throw std::runtime_error("Uniform validation failed");
    }
}

void GLProgram::createElementIndex(const std::vector<unsigned int>& vals)
{
    attributeMap["index"] = GLAttribute(this);
    attributeMap["index"].setData(vals);
}

void GLProgram::createUniform(std::string uniformName, DataType type)
{
    if(uniformMap.find(uniformName) != uniformMap.end()){
        throw std::runtime_error("Attempted to create uniform with duplicate name " + uniformName);
    }
    
    uniformMap[uniformName] = GLUniform(this, uniformName, type);
}

void GLProgram::createAttribute(std::string attributeName, DataType type, bool dynamic)
{
    if(attributeMap.find(attributeName) != attributeMap.end()){
        throw std::runtime_error("Attempted to create attribute with duplicate name " + attributeName);
    }
    
    attributeMap[attributeName] = GLAttribute(this, attributeName, type, dynamic);
}

void GLProgram::createTexture(std::string textureName, std::string sourceFile)
{
    if(textureMap.find(textureName) != textureMap.end()){
        throw std::runtime_error("Attempted to create texture with duplicate name " + textureName);
    }
    
    textureMap[textureName] = GLTexture(this, textureName, sourceFile);
}

void GLProgram::createTexture(std::string textureName, const cv::Mat& img)
{
    if(textureMap.find(textureName) != textureMap.end()){
        throw std::runtime_error("Attempted to create texture with duplicate name " + textureName);
    }
    
    textureMap[textureName] = GLTexture(this, textureName, img);
}

void GLProgram::createTexture(std::string textureName, GLuint location, int w, int h)
{
    if(textureMap.find(textureName) != textureMap.end()){
        throw std::runtime_error("Attempted to create texture with duplicate name " + textureName);
    }
    
    textureMap[textureName] = GLTexture(this, textureName, location, w, h);
}

void GLProgram::updateTexture(std::string textureName, std::string sourceFile, bool flip)
{
    if(textureMap.find(textureName) == textureMap.end()){
        throw std::runtime_error("Attempted to update texture which does not exist: " + textureName);
    }
    
    GLTexture& tex = textureMap[textureName];
    
    cv::Mat_<cv::Vec3b> img = cv::imread(sourceFile);
    if(img.size() != cv::Size(tex.width,tex.height))
       cv::resize(img, img, cv::Size(tex.width, tex.height));
    if(flip)
       cv::flip(img,img, -1);
    if(img.empty()){
        throw std::runtime_error("Attempted to load image which does not exist: " + textureName);
    }
    GLTexture::UpdateTexture(img, tex.location);
}

void GLProgram::updateTexture(std::string textureName, GLuint location)
{
    if(textureMap.find(textureName) == textureMap.end()){
        throw std::runtime_error("Attempted to update texture which does not exist: " + textureName);
    }
    
    GLTexture& tex = textureMap[textureName];
    
    tex.location = location;
}

void GLProgram::updateTexture(std::string textureName, const cv::Mat& img)
{
    if(textureMap.find(textureName) == textureMap.end()){
        throw std::runtime_error("Attempted to update texture which does not exist: " + textureName);
    }
    
    GLTexture& tex = textureMap[textureName];
    
    GLTexture::UpdateTexture(img, tex.location);
}

void GLProgram::setUniformData(std::string uniformName, const Eigen::Matrix4f &val)
{
    if(uniformMap.find(uniformName) == uniformMap.end()){
        throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
    }
    
    uniformMap[uniformName].setData(val);
}

void GLProgram::setUniformData(std::string uniformName, const glm::vec3 &val)
{
    if(uniformMap.find(uniformName) == uniformMap.end()){
        throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
    }
    
    uniformMap[uniformName].setData(val);
}

void GLProgram::setUniformData(std::string uniformName, float val)
{
    if(uniformMap.find(uniformName) == uniformMap.end()){
        throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
    }
    
    uniformMap[uniformName].setData(val);
}

void GLProgram::setUniformData(std::string uniformName, uint val)
{
    if(uniformMap.find(uniformName) == uniformMap.end()){
        throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
    }
    
    uniformMap[uniformName].setData(val);
}

void GLProgram::setUniformData(std::string uniformName, const std::vector<Eigen::Matrix4f> &val)
{
    if(uniformMap.find(uniformName) == uniformMap.end()){
        throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
    }
    
    uniformMap[uniformName].setData(val);
}

void GLProgram::setUniformData(std::string uniformName, const std::vector<glm::vec3> &val)
{
    if(uniformMap.find(uniformName) == uniformMap.end()){
        throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
    }
    
    uniformMap[uniformName].setData(val);
}

void GLProgram::setUniformData(std::string uniformName, const std::vector<float> &val)
{
    if(uniformMap.find(uniformName) == uniformMap.end()){
        throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
    }
    
    uniformMap[uniformName].setData(val);
}

void GLProgram::setUniformData(std::string uniformName, const std::vector<uint> &val)
{
    if(uniformMap.find(uniformName) == uniformMap.end()){
        throw std::runtime_error("Attempted to set uniform which does not exist: " + uniformName);
    }
    
    uniformMap[uniformName].setData(val);
}

void GLProgram::setAttributeData(std::string attributeName, const std::vector<float> &vals)
{
    if(attributeMap.find(attributeName) == attributeMap.end()){
        throw std::runtime_error("Attempted to set attribute which does not exist: " + attributeName);
    }
    
    attributeMap[attributeName].setData(vals);
}

void GLProgram::setAttributeData(std::string attributeName, const std::vector<glm::vec4> &vals)
{
    if(attributeMap.find(attributeName) == attributeMap.end()){
        throw std::runtime_error("Attempted to set attribute which does not exist: " + attributeName);
    }
    
    attributeMap[attributeName].setData(vals);
}

void GLProgram::setAttributeData(std::string attributeName, const std::vector<glm::vec3> &vals)
{
    if(attributeMap.find(attributeName) == attributeMap.end()){
        throw std::runtime_error("Attempted to set attribute which does not exist: " + attributeName);
    }
    
    attributeMap[attributeName].setData(vals);
}

void GLProgram::setAttributeData(std::string attributeName, const std::vector<glm::vec2> &vals)
{
    if(attributeMap.find(attributeName) == attributeMap.end()){
        throw std::runtime_error("Attempted to set attribute which does not exist: " + attributeName);
    }
    
    attributeMap[attributeName].setData(vals);
}

// === GLAttribute class functions
GLAttribute::GLAttribute() {}

GLAttribute::GLAttribute(GLProgram* parentProgram_, std::string name_, DataType type_, bool dynamic_)
: parentProgram(parentProgram_), name(name_), type(type_), dynamic(dynamic_)
{
    glUseProgram(parentProgram->programHandle);
    
    location = glGetAttribLocation(parentProgram->programHandle, name.c_str());
    if(location == -1){
        cerr << "ERROR: Attribute " << name << " does not appear in this shader program." << endl;
        throw std::runtime_error("Tried to set attribute with invalid name");
    }
    
    glBindVertexArray(parentProgram->vaoHandle);
    glEnableVertexAttribArray(location);
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    
    switch (type) {
        case DataType::FLOAT:
            glVertexAttribPointer(location, 1, GL_FLOAT, 0, 0, 0);
            break;
        case DataType::VECTOR2:
            glVertexAttribPointer(location, 2, GL_FLOAT, 0, 0, 0);
            break;
        case DataType::VECTOR3:
            glVertexAttribPointer(location, 3, GL_FLOAT, 0, 0, 0);
            break;
        case DataType::VECTOR4:
            glVertexAttribPointer(location, 4, GL_FLOAT, 0, 0, 0);
            break;
        case DataType::MATRIX44:
            throw std::runtime_error("Unsupported attribute");
            break;
    }
}

GLAttribute::GLAttribute(GLProgram* parentProgram_)
: parentProgram(parentProgram_), type(DataType::INDEX), dynamic(false)
{
    glUseProgram(parentProgram->programHandle);
    
    glBindVertexArray(parentProgram->vaoHandle);
    glGenBuffers(1, &VBO);
}

void GLAttribute::setData(const std::vector<glm::vec4> &data)
{
    if(type != DataType::VECTOR4){
        throw std::runtime_error("Attempted to set attribute of type " + std::to_string(static_cast<int>(type)) + " with Vector4 data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glBindVertexArray(parentProgram->vaoHandle);
    
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    if(dynamic)
        glBufferData(GL_ARRAY_BUFFER, 4*sizeof(float)*data.size(), &data[0], GL_DYNAMIC_DRAW);
    else
        glBufferData(GL_ARRAY_BUFFER, 4*sizeof(float)*data.size(), &data[0], GL_STATIC_DRAW);
    
    dataSize = data.size();
}

void GLAttribute::setData(const std::vector<glm::vec3> &data)
{
    if(type != DataType::VECTOR3){
        throw std::runtime_error("Attempted to set attribute of type " + std::to_string(static_cast<int>(type)) + " with Vector3 data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glBindVertexArray(parentProgram->vaoHandle);
    
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    if(dynamic)
        glBufferData(GL_ARRAY_BUFFER, 3*sizeof(float)*data.size(), &data[0], GL_DYNAMIC_DRAW);
    else
        glBufferData(GL_ARRAY_BUFFER, 3*sizeof(float)*data.size(), &data[0], GL_STATIC_DRAW);
    
    dataSize = data.size();
}

void GLAttribute::setData(const std::vector<glm::vec2> &data)
{
    if(type != DataType::VECTOR2){
        throw std::runtime_error("Attempted to set attribute of type " + std::to_string(static_cast<int>(type)) + " with Vector2 data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glBindVertexArray(parentProgram->vaoHandle);
    
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    if(dynamic)
        glBufferData(GL_ARRAY_BUFFER, 2*sizeof(float)*data.size(), &data[0], GL_DYNAMIC_DRAW);
    else
        glBufferData(GL_ARRAY_BUFFER, 2*sizeof(float)*data.size(), &data[0], GL_STATIC_DRAW);
    
    dataSize = data.size();
}

void GLAttribute::setData(const std::vector<float> &data)
{
    if(type != DataType::FLOAT){
        throw std::runtime_error("Attempted to set attribute of type " + std::to_string(static_cast<int>(type)) + " with scalar data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glBindVertexArray(parentProgram->vaoHandle);
    
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    if(dynamic)
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*data.size(), &data[0], GL_DYNAMIC_DRAW);
    else
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*data.size(), &data[0], GL_STATIC_DRAW);
    
    dataSize = data.size();
}

void GLAttribute::setData(const std::vector<unsigned int> &data)
{
    if(type != DataType::INDEX){
        throw std::runtime_error("Attempted to set attribute of type " + std::to_string(static_cast<int>(type)) + " with scalar data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glBindVertexArray(parentProgram->vaoHandle);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*data.size(), &data[0], GL_STATIC_DRAW);
    CHECK_GL_ERROR();
    
    dataSize = data.size();
}


// === GLUniform class functions
GLUniform::GLUniform() {}

GLUniform::GLUniform(GLProgram* parentProgram_, std::string name_, DataType type_)
: parentProgram(parentProgram_), name(name_), type(type_)
{
    glUseProgram(parentProgram->programHandle);

    location = glGetUniformLocation(parentProgram->programHandle, name.c_str());
    if(location == -1){
        cerr << "ERROR: Uniform " << name << " does not appear in this shader program." << endl;
        throw std::runtime_error("Tried to create uniform with invalid name");
    }
}

void GLUniform::setData(const std::vector<Eigen::Matrix4f> &val)
{
    if(type != DataType::MATRIX44){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with Eigen::Matrix4f data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniformMatrix4fv(location, val.size(), GL_FALSE, (float*)val[0].data());

    hasBeenSet = true;
}

void GLUniform::setData(const std::vector<glm::vec3> &val)
{
    if(type != DataType::VECTOR3){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with Vector3 data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniform3fv(location, val.size(), (float*)&val[0][0]);
    
    hasBeenSet = true;
}

void GLUniform::setData(const std::vector<float> &val)
{
    if(type != DataType::FLOAT){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with float data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniform1fv(location, val.size(), (float*)&val[0]);
    
    hasBeenSet = true;
}

void GLUniform::setData(const std::vector<uint> &val)
{
    if(type != DataType::UINT){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with uint data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniform1uiv(location, val.size(), (uint*)&val[0]);
    
    hasBeenSet = true;
}

void GLUniform::setData(const Eigen::Matrix4f &val)
{
    if(type != DataType::MATRIX44){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with Eigen::Matrix4f data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniformMatrix4fv(location, 1, GL_FALSE, val.data());
    
    hasBeenSet = true;
}

void GLUniform::setData(const glm::vec3 &val)
{
    if(type != DataType::VECTOR3){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with Vector3 data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniform3f(location, val[0], val[1], val[2]);
    
    hasBeenSet = true;
}

void GLUniform::setData(float val)
{
    if(type != DataType::FLOAT){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with float data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniform1f(location, val);
    
    hasBeenSet = true;
}

void GLUniform::setData(uint val)
{
    if(type != DataType::UINT){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with uint data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniform1ui(location, val);
    
    hasBeenSet = true;
}


// === GLTexture class functions
GLuint GLTexture::CreateTexture(const cv::Mat &img)
{
    GLuint location;
    glGenTextures(1, &location);
    
    CHECK_GL_ERROR();
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, location);
    
    CHECK_GL_ERROR();
    
    GLint mode1, mode2;
    switch (img.channels()) {
        case 4:
            mode1 = GL_RGBA;
            mode2 = GL_BGRA;
            break;
        case 3:
            mode1 = GL_RGB;
            mode2 = GL_BGR;
            break;
        case 1:
            mode1 = GL_RED;
            mode1 = GL_RED;
            break;
        default:
            cerr << "ERROR: Unsupported channels" << endl;
            break;
    }
    
    GLint type;
    if ((img.type() & CV_MAT_DEPTH_MASK) == CV_8U)
    {
        type = GL_UNSIGNED_BYTE;
    }
    else if ((img.type() & CV_MAT_DEPTH_MASK) == CV_32F)
    {
        type = GL_FLOAT;
    }
    else
    {
        cerr << "ERROR: Unsupported image type." << endl;
    }
    
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,  GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    CHECK_GL_ERROR();
    glTexImage2D(GL_TEXTURE_2D, 0, mode1, img.cols, img.rows, 0, mode2, type, img.ptr());
    
    CHECK_GL_ERROR();
    
    return location;
}

void GLTexture::UpdateTexture(const cv::Mat& img,GLuint &_location)
{
    if(_location < 1){
        _location = CreateTexture(img);
        return;
    }
    
    GLint mode;
    switch (img.channels())
    {
        case 4:
            mode = GL_BGRA;
            break;
        case 3:
            mode = GL_BGR;
            break;
        case 1:
            mode = GL_RED;
            break;
        default:
            cerr << "ERROR: Unsupported channels" << endl;
            break;
    }
    
    GLint type;
    if ((img.type() & CV_MAT_DEPTH_MASK) == CV_8U)
    {
        type = GL_UNSIGNED_BYTE;
    }
    else if ((img.type() & CV_MAT_DEPTH_MASK) == CV_32F)
    {
        type = GL_FLOAT;
    }
    else
    {
        cerr << "ERROR: Unsupported image type." << endl;
    }
    
    glBindTexture(GL_TEXTURE_2D, _location);
    CHECK_GL_ERROR();
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.cols, img.rows, mode, type, img.ptr());
    CHECK_GL_ERROR();
    glBindTexture(GL_TEXTURE_2D, 0);
    CHECK_GL_ERROR();
}

GLTexture::GLTexture() {}

GLTexture::GLTexture(GLProgram* parentProgram_, std::string name_, std::string sourceFile_)
: parentProgram(parentProgram_), name(name_), sourceFile(sourceFile_)
{
    glUseProgram(parentProgram->programHandle);
    
    glGenTextures(1, &location);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, location);
    
    imageData = cv::imread(sourceFile, CV_LOAD_IMAGE_COLOR);
    if(imageData.empty()){
        throw std::runtime_error("    Failed to load texture image from path: " + sourceFile);
    }
    cv::flip(imageData, imageData, 0);
    std::cout << "    Loading texture image from path: " << sourceFile << std::endl;
    // TODO: this should be something better to make sure texture size is exponent of 2
    cv::resize(imageData, imageData, cv::Size(1024,1024));
    width = imageData.cols; height = imageData.rows;
    cout << "    Loaded with width = " << width << "  height " << height << endl;
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, imageData.ptr());
    
    imageData.release();
    
    glUniform1i(glGetUniformLocation(parentProgram->programHandle, name.c_str()), 0);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    CHECK_GL_ERROR();
}

GLTexture::GLTexture(GLProgram* parentProgram_, std::string name_, const cv::Mat& img)
: parentProgram(parentProgram_), name(name_)
{
    glUseProgram(parentProgram->programHandle);

    width = img.cols;
    height = img.rows;
    
    location = CreateTexture(img);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, location);
    
    glUniform1i(glGetUniformLocation(parentProgram->programHandle, name.c_str()), 0);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    CHECK_GL_ERROR();
}

GLTexture::GLTexture(GLProgram* parentProgram_, std::string name_, GLuint location_, int w, int h)
: parentProgram(parentProgram_), name(name_)
{
    glUseProgram(parentProgram->programHandle);
    
    width = w;
    height = h;
    
    location = location_;

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, location);
    
    glUniform1i(glGetUniformLocation(parentProgram->programHandle, name.c_str()), 0);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    CHECK_GL_ERROR();
}

Drawable::~Drawable() {}







