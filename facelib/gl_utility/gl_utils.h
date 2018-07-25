#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>

#define PI 3.14159265358979323846264

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/gtx/rotate_vector.hpp>


inline std::ofstream &operator<< (std::ofstream &out, const glm::vec3 &vec) {
    out << "<" << vec.x << "," << vec.y << "," << vec.z << ">";
    return out;
}

inline std::ofstream &operator<< (std::ofstream &out, const glm::vec2 &vec) {
    out << "<" << vec.x << "," << vec.y << ">";
    return out;
}

inline std::ofstream &operator<< (std::ofstream &out, const glm::mat4 &mat) {
    for(int i = 0; i < 4; ++i){
        out << mat[i][0] << "  " << mat[i][1] << "  " << mat[i][2] << "  " << mat[i][3] << std::endl;
    }
    return out;
}

inline std::ofstream &operator<< (std::ofstream &out, const glm::mat3 &mat) {
    for(int i = 0; i < 4; ++i){
        out << mat[i][0] << "  " << mat[i][1] << "  " << mat[i][2] << std::endl;
    }
    return out;
}

inline std::ofstream &operator<< (std::ofstream &out, const glm::mat2 &mat) {
    for(int i = 0; i < 4; ++i){
        out << mat[i][0] << "  " << mat[i][1] << std::endl;
    }
    return out;
}

inline float clamp(float val, double low, double high) {
    if(val < low) val = low;
    if(val > high) val = high;
    return val;
}

inline float regularAngle(float theta) {
    theta -= std::floor(theta / (2*PI)) * 2 * PI;
    return theta;
}

inline bool isnan(const glm::vec3& x){
    return std::isnan(x.x) || std::isnan(x.y) || std::isnan(x.z);
}
inline bool isfinate(const glm::vec3& x) {
    return std::isfinite(x.x) && std::isfinite(x.y) && std::isfinite(x.z);
}
inline bool isnan(const glm::vec2& x){
    return std::isnan(x.x) || std::isnan(x.y);
}
inline bool isfinate(const glm::vec2& x) {
    return std::isfinite(x.x) && std::isfinite(x.y);
}

inline bool endsWith(std::string const &str, std::string const &suffix) {
    if(str.length() >= suffix.length()) {
        return (0 == str.compare(str.length() - suffix.length(), suffix.length(), suffix));
    } else {
        return false;
    }
}

#define SEGFAULT() std::cout << (*(static_cast<int*>(0) + 3));

static std::string GetGLErrorString(GLenum& err)
{
    std::string error;
    
    switch(err) {
        case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
        case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
        case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
        case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
    }
    
    return error;
}

static void CheckGLError(const char *file, int line)
{
    GLenum error = glGetError();
    if(error != GL_NO_ERROR)
    {
        std::cout << "GL ERROR: " << GetGLErrorString(error) << " - " << file << ":" << line << std::endl;
    }
}

enum COLOR{
    COLOR_WHITE, COLOR_GREY, COLOR_GREEN, COLOR_ALPHA
};
inline void clearBuffer(COLOR mode) {
    switch(mode){
        case COLOR_WHITE:
            glClearColor(1.f, 1.f, 1.f, 1.f);
            break;
        case COLOR_GREY:
            glClearColor(0.5f, 0.5f, 0.5f, 1.f);
            break;
        case COLOR_GREEN:
            glClearColor(0.f, 1.f, 0.f, 1.f);
            break;
        case COLOR_ALPHA:
            glClearColor(0.f, 0.f, 0.f, 0.f);
            break;
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

#ifndef NDEBUG
#define CHECK_GL_ERROR() CheckGLError(__FILE__,__LINE__)
#else
#define CHECK_GL_ERROR()
#endif

typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;

static Mat4f PerspectiveFromVision(const Mat4f& K, unsigned int width, unsigned int height, float zNear, float zFar) {
    Mat4f flip = Mat4f::Identity();
    flip(1, 1) = -1.f;
    flip(1, 2) = static_cast<float>(height);
    
    Mat4f intrinsics = flip*K;
    const float L = 0.f;   // left
    const float R = static_cast<float>(width); // right
    const float T = 0.f;// top
    const float B = static_cast<float>(height);   // bottom
    
    // orthographic projection
    // https://msdn.microsoft.com/en-us/library/windows/desktop/bb205347(v=vs.85).aspx
    Mat4f ortho = Mat4f::Zero();
    ortho(0, 0) = 2.0f / (R - L); ortho(0, 3) = (R + L) / (R - L);
    ortho(1, 1) = -2.0f / (T - B); ortho(1, 3) = -(T + B) / (T - B);
    ortho(2, 2) = 2.0f / (zFar - zNear); ortho(2, 3) = (zNear + zFar) / (zNear - zFar);
    ortho(3, 3) = 1.f;

    // perspective projection
    Mat4f tp = Mat4f::Zero();
    tp(0, 0) = intrinsics(0, 0); tp(0, 1) = intrinsics(0, 1); tp(0, 2) = -(width - intrinsics(0, 2));
    tp(1, 1) = intrinsics(1, 1); tp(1, 2) = -(height - intrinsics(1, 2));
    tp(2, 2) = zNear + zFar; tp(2, 3) = - zNear * zFar;
    tp(3, 2) = 1.0f;
    return ortho * tp;
}

static Mat4f OrthogonalProjection(unsigned int width, unsigned int height, float zNear, float zFar) {
    const float L = 0.f;   // left
    const float R = static_cast<float>(width); // right
    const float T = 0.f;// top
    const float B = static_cast<float>(height);   // bottom
    
    // orthographic projection
    // https://msdn.microsoft.com/en-us/library/windows/desktop/bb205347(v=vs.85).aspx
    Mat4f ortho = Mat4f::Zero();
    ortho(0, 0) = 2.0f / (R - L); ortho(0, 3) = -(R + L) / (R - L);
    ortho(1, 1) = 2.0f / (T - B); ortho(1, 3) = -(T + B) / (T - B);
    ortho(2, 2) = 2.0f / (zFar - zNear); ortho(2, 3) = (zNear + zFar) / (zNear - zFar);
    ortho(3, 3) = 1.f;

    return ortho;
}
