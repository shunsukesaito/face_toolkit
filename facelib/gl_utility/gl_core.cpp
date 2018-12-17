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
#include "gl_core.h"

#include <utility/str_utils.h>

using std::cerr;
using std::cout;
using std::endl;

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

static std::string readShaderFile(std::string root_dir, std::string name, int &additionalLines)
{
    additionalLines = 0;
    std::string text;
    std::ifstream fin(root_dir + "/" + name);
    if(fin.is_open()){
        std::string line;
        while(std::getline(fin, line))
        {
            if (line.empty())
                continue;
            
            if (line.find("#include ") != std::string::npos){
                std::string inc_name = split_str(line, " ")[1];
                find_erase_all(inc_name, "\"");
                int throwaway;
                line = readShaderFile(root_dir, inc_name, throwaway);
                additionalLines += split_str(line, "\n").size() - 1;
            }
            text += line + "\n";
        }
    }
    else {
        std::cout << "Could not open file: " << root_dir + "/" + name << std::endl;
        exit(1);
    }
    return text;
}

void printShaderInfoLog(GLuint shaderHandle, std::string shaderName)
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
        printf("Shader (%s) info log:\n%s\n", shaderName.c_str(), log);
        free(log);
        throw std::runtime_error("Shader compilation failed");
    }
}

void printProgramInfoLog(GLuint handle, std::string progName)
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
        printf("Program (%s) info log:\n%s\n", progName.c_str(), log);
        free(log);
    }
}

GLProgram::GLProgram(std::string root_dir, std::string vertShader, std::string fragShader, DrawMode drawMode_)
: GLProgram(root_dir, vertShader, "", "", "", fragShader, drawMode_)
{
}


GLProgram::GLProgram(std::string root_dir, std::string vertShader, std::string geomShader, std::string fragShader, DrawMode drawMode_)
: GLProgram(root_dir, vertShader, "", "", geomShader, fragShader, drawMode_)
{
}

GLProgram::GLProgram(std::string root_dir, std::string vertShader, std::string tcShader, std::string teShader,std::string geomShader, std::string fragShader, DrawMode drawMode_)
: drawMode(drawMode_)
{
    vsHandle = glCreateShader(GL_VERTEX_SHADER);
    fsHandle = glCreateShader(GL_FRAGMENT_SHADER);
    CHECK_GL_ERROR();
    
    int n_line;
    std::string vertTmp = readShaderFile(root_dir, vertShader, n_line);
    const char *vertShaderTmp = vertTmp.c_str();
    glShaderSource(vsHandle, 1, &vertShaderTmp, nullptr);
    glCompileShader(vsHandle);
    printShaderInfoLog(vsHandle, vertShader);
    
    if(!tcShader.empty()){
        tcsHandle = glCreateShader(GL_TESS_CONTROL_SHADER);
        std::string tcTmp = readShaderFile(root_dir, tcShader, n_line);
        const char *tcShaderTmp = tcTmp.c_str();
        //const char *tcShaderTmp = textFileRead(tcShader.c_str());
        glShaderSource(tcsHandle, 1, &tcShaderTmp, nullptr);
        glCompileShader(tcsHandle);
        printShaderInfoLog(tcsHandle, tcShader);
    }
    
    if(!teShader.empty()){
        tesHandle = glCreateShader(GL_TESS_EVALUATION_SHADER);
        std::string teTmp = readShaderFile(root_dir, teShader, n_line);
        const char *teShaderTmp = teTmp.c_str();
        //const char *teShaderTmp = textFileRead(teShader.c_str());
        glShaderSource(tesHandle, 1, &teShaderTmp, nullptr);
        glCompileShader(tesHandle);
        printShaderInfoLog(tesHandle, teShader);
    }
    
    if(!geomShader.empty()){
        gsHandle = glCreateShader(GL_GEOMETRY_SHADER);
        std::string geoTmp = readShaderFile(root_dir, geomShader, n_line);
        const char *geomShaderTmp = geoTmp.c_str();
        //const char *geomShaderTmp = textFileRead(geomShader.c_str());
        glShaderSource(gsHandle, 1, &geomShaderTmp, nullptr);
        glCompileShader(gsHandle);
        printShaderInfoLog(gsHandle, geomShader);
    }
    
    std::string fragTmp = readShaderFile(root_dir, fragShader, n_line);
    const char *fragShaderTmp = fragTmp.c_str();
    glShaderSource(fsHandle, 1, &fragShaderTmp, nullptr);
    glCompileShader(fsHandle);
    printShaderInfoLog(fsHandle, fragShader);
    
    programHandle = glCreateProgram();
    glAttachShader(programHandle, vsHandle);
    if(!tcShader.empty()){
        glAttachShader(programHandle, tcsHandle);
    }
    if(!teShader.empty()){
        glAttachShader(programHandle, tesHandle);
    }
    if(!geomShader.empty()){
        glAttachShader(programHandle, gsHandle);
    }
    glAttachShader(programHandle, fsHandle);
    
    // link the program
    glLinkProgram(programHandle);
    GLint linked;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &linked);
    if(!linked)
        printProgramInfoLog(programHandle, vertShader);
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
        case DrawMode::POINTS_IDX:
            glDrawElements(GL_POINTS, attributeMap["index"].dataSize, GL_UNSIGNED_INT, NULL);
            break;
        case DrawMode::PATCHES:
            glPatchParameteri(GL_PATCH_VERTICES, 3);
            glDrawArrays(GL_PATCHES, 0, (GLsizei)dataLen);
            break;
        case DrawMode::PATCHES_IDX:
            glPatchParameteri(GL_PATCH_VERTICES, 3);
            glDrawElements(GL_PATCHES, attributeMap["index"].dataSize, GL_UNSIGNED_INT, NULL);
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

void GLProgram::createUniform(std::string uniformName, DataType type)
{
    if(uniformMap.find(uniformName) != uniformMap.end()){
        throw std::runtime_error("Attempted to create uniform with duplicate name " + uniformName);
    }
    
    uniformMap[uniformName] = GLUniform(this, uniformName, type);
}

void GLProgram::createElementIndex(const std::vector<unsigned int>& vals)
{
    attributeMap["index"] = GLAttribute(this);
    attributeMap["index"].setData(vals);
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
    tex.UpdateTexture(img);
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
    
    tex.UpdateTexture(img);
}

void GLProgram::updateElementIndex(const std::vector<unsigned int>& vals)
{
    if(attributeMap.find("index") == attributeMap.end()){
        throw std::runtime_error("Attempted to set attribute which does not exist: index");
    }

    attributeMap["index"].setData(vals);
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

void GLUniform::setData(const std::vector<glm::vec4> &val)
{
    if(type != DataType::VECTOR4){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with Vector4 data");
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

void GLUniform::setData(const glm::vec2 &val)
{
    if(type != DataType::VECTOR2){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with Eigen::Vector2f data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniform2f(location, val[0], val[1]);
    
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

void GLUniform::setData(const glm::vec4 &val)
{
    if(type != DataType::VECTOR4){
        throw std::runtime_error("Attempted to set uniform of type " + std::to_string(static_cast<int>(type)) + " with Vector4 data");
    }
    
    glUseProgram(parentProgram->programHandle);
    glUniform4f(location, val[0], val[1], val[2], val[3]);
    
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
    GLint type;
    if ((img.type() & CV_MAT_DEPTH_MASK) == CV_8U)
    {
        type = GL_UNSIGNED_BYTE;
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
                mode2 = GL_RED;
                break;
            default:
                cerr << "ERROR: Unsupported channels" << endl;
                break;
        }
    }
    else if ((img.type() & CV_MAT_DEPTH_MASK) == CV_32F)
    {
        type = GL_FLOAT;
        
        switch (img.channels()) {
            case 4:
                mode1 = GL_RGBA32F_ARB;
                mode2 = GL_BGRA;
                break;
            case 3:
                mode1 = GL_RGB32F_ARB;
                mode2 = GL_BGR;
                break;
            case 1:
                mode1 = GL_RED;
                mode2 = GL_RED;
                break;
            default:
                cerr << "ERROR: Unsupported channels" << endl;
                break;
        }
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

GLuint GLTexture::CreateTexture(const HDRLoaderResult& img)
{
    GLuint location;
    glGenTextures(1, &location);
    
    CHECK_GL_ERROR();
    
    glBindTexture(GL_TEXTURE_2D, location);
    
    CHECK_GL_ERROR();
    
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F_ARB, img.width, img.height, 0, GL_RGB, GL_FLOAT, &img.cols[0]);
    
    CHECK_GL_ERROR();
    
    return location;
}

GLuint GLTexture::CreateTexture(const TinyExrImage& img)
{
    GLuint location;
    glGenTextures(1, &location);
    
    CHECK_GL_ERROR();
    
    glBindTexture(GL_TEXTURE_2D, location);
    
    CHECK_GL_ERROR();

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    
    //[NOTE]: RGBA
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F_ARB, img.width, img.height, 0, GL_RGBA, GL_FLOAT, &img.buf[0]);
    
    CHECK_GL_ERROR();
    
    return location;
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
    CHECK_GL_ERROR();
}

void GLTexture::UpdateTexture(const cv::Mat& img)
{
    if(location < 1){
        location = CreateTexture(img);
        return;
    }
    if(width != img.cols || height != img.rows){
        glDeleteTextures(1, &location);
        location = CreateTexture(img);
        width = img.cols;
        height = img.rows;
        
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
    
    glBindTexture(GL_TEXTURE_2D, location);
    CHECK_GL_ERROR();
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.cols, img.rows, mode, type, img.ptr());
    CHECK_GL_ERROR();
    glBindTexture(GL_TEXTURE_2D, 0);
    CHECK_GL_ERROR();
}


Drawable::~Drawable() {}







