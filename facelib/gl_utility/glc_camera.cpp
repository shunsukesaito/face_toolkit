//
//  glc_camera.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "glc_camera.h"

void createGLCFromSphere(float radius, unsigned int rings, unsigned int sectors, const Eigen::Vector3f& p,
                         float zN, float zF, int width, int height, std::vector<GLCCamera>& cameras)
{
    float const R = 1./(float)(rings-1);
    float const S = 1./(float)(sectors-1);
    int r, s;
    
    std::vector<glm::vec3> ptsv(rings * sectors);
    std::vector<glm::vec2> uvsv(rings * sectors);
    std::vector<unsigned int> tri;
    
    std::vector<glm::vec3>::iterator v = ptsv.begin();
    std::vector<glm::vec2>::iterator t = uvsv.begin();
    for(r = 1; r < rings-1; r++) for(s = 0; s < sectors; s++) {
        float const y = sin( -M_PI_2 + M_PI * r * R );
        float const x = cos(2.0*M_PI * s * S) * sin( M_PI * r * R );
        float const z = sin(2.0*M_PI * s * S) * sin( M_PI * r * R );
        
        float len = sqrt(x * x + y * y + z * z);
        (*t)[0] = (atan2(z, x) / M_PI + 1.0f) * 0.5f;
        (*t)[1] = acos(y / len) / M_PI;
        *t++;
        
        (*v)[0] = x * radius + p[0];
        (*v)[1] = y * radius + p[1];
        (*v)[2] = z * radius + p[2];
        *v++;
        
        int curRow = (r-1) * sectors;
        int nextRow = r * sectors;
        
        if(s != sectors - 1 && r != rings - 2){
            tri.push_back(curRow + s);
            tri.push_back(nextRow + s);
            tri.push_back(nextRow + (s+1));
            
            tri.push_back(curRow + s);
            tri.push_back(nextRow + (s+1));
            tri.push_back(curRow + (s+1));
        }
    }
    
    std::vector<glm::vec3> pts; pts.reserve(tri.size());
    std::vector<glm::vec2> uvs; uvs.reserve(tri.size());
    for(int i : tri)
    {
        pts.push_back(ptsv[i]);
        uvs.push_back(uvsv[i]);
    }
    for(int i = 0; i < uvs.size(); i+=3)
    {
        glm::vec2 t1 =uvs[i];
        glm::vec2 t2 =uvs[i+1];
        glm::vec2 t3 =uvs[i+2];
        // fix the mirrored uv
        if(t2[0]-t1[0]>0.25)
            uvs[i][0] = 1.0 - t1[0];
        else if(t1[0]-t2[0]>0.25)
            uvs[i+1][0] = 1.0 - t2[0];
        if(t3[0]-t2[0]>0.25)
            uvs[i+1][0] = 1.0 - t2[0];
        else if(t2[0]-t3[0]>0.25)
            uvs[i+2][0] = 1.0 - t3[0];
        if(t3[0]-t1[0]>0.25)
            uvs[i][0] = 1.0 - t1[0];
        else if(t1[0]-t3[0]>0.25)
            uvs[i+2][0] = 1.0 - t3[0];
        
        // fix polar cases
//        if(t1[1] > 0.99 || t1[1] < 0.01)
//            uvs[i][0] = 0.5;
//        if(t2[1] > 0.99 || t2[1] < 0.01)
//            uvs[i+1][0] = 0.5;
//        if(t3[1] > 0.99 || t3[1] < 0.01)
//            uvs[i+2][0] = 0.5;
    }
    
    cameras.clear();
    for(int i = 0; i < tri.size()/3; ++i)
    {
        cameras.push_back(GLCCamera(Eigen::Vector3f((float*)&pts[i*3+0][0]),
                                    Eigen::Vector3f((float*)&pts[i*3+1][0]),
                                    Eigen::Vector3f((float*)&pts[i*3+2][0]),
                                    Eigen::Vector2f((float*)&uvs[i*3+0][0]),
                                    Eigen::Vector2f((float*)&uvs[i*3+1][0]),
                                    Eigen::Vector2f((float*)&uvs[i*3+2][0]),
                                    p, p, p, zN, zF, width, height));
    }
    std::cout << cameras.size() << std::endl;
}

GLCCamera::GLCCamera(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3,
                     const Eigen::Vector2f& uv1, const Eigen::Vector2f& uv2, const Eigen::Vector2f& uv3,
                     const Eigen::Vector3f& q1, const Eigen::Vector3f& q2, const Eigen::Vector3f& q3,
                     float zNear, float zFar, int w, int h)
: p1_(p1), p2_(p2), p3_(p3), uv1_(uv1), uv2_(uv2), uv3_(uv3),
q1_(q1), q2_(q2), q3_(q3), zNear_(zNear), zFar_(zFar), width_(w), height_(h)
{
}

GLCCamera::GLCCamera(const GLCCamera& other)
{
    name_ = other.name_;
    
    p1_ = other.p1_;
    p2_ = other.p2_;
    p3_ = other.p3_;
    uv1_ = other.uv1_;
    uv2_ = other.uv2_;
    uv3_ = other.uv3_;
    q1_ = other.q1_;
    q2_ = other.q2_;
    q3_ = other.q3_;

    zNear_ = other.zNear_;
    zFar_ = other.zFar_;
    
    width_ = other.width_;
    height_ = other.height_;
}

void GLCCamera::initializeUniforms(GLProgram& program, int flag)
{
    program.createUniform("u_uv1", DataType::VECTOR2);
    program.createUniform("u_uv2", DataType::VECTOR2);
    program.createUniform("u_uv3", DataType::VECTOR2);

    program.createUniform("u_d1", DataType::VECTOR4);
    program.createUniform("u_d2", DataType::VECTOR4);
    program.createUniform("u_d3", DataType::VECTOR4);

//    program.createUniform("u_p1", DataType::VECTOR3);
//    program.createUniform("u_p2", DataType::VECTOR3);
//    program.createUniform("u_p3", DataType::VECTOR3);

    program.createUniform("u_T", DataType::MATRIX44);
    if(flag & U_GLCCAMERA_MV)
        program.createUniform("u_modelview", DataType::MATRIX44);
}

void GLCCamera::updateTransform(Eigen::Matrix4f& T, Eigen::Matrix2f& d1, Eigen::Matrix2f& d2, Eigen::Matrix2f& d3) const
{
    assert(zNear_ > 0);
    assert(zFar_ > zNear_);
    
    Eigen::Vector3f nx = (p1_-p2_).normalized();
    Eigen::Vector3f ny = (p3_-p2_).normalized();
    Eigen::Vector3f nz = (nx.cross(ny)).normalized();
    nx = (ny.cross(nz)).normalized();
    
    // scale factor
    Eigen::Vector3f s;
    float sx = 1.0f;
    float sy = 1.0f;
    float sz = 1.0f/(zFar_-zNear_);
    s << sx, sy, sz;
    
    // rotation
    Eigen::Matrix3f R;
    R << nx, ny, nz;
    
    // translation
    Eigen::Vector3f t;
    t = zNear_*nz+p1_;
  
    // compose them into 4x4 transform matrix
    T.setIdentity();
    T.block(0,0,3,3) = s.asDiagonal()*R.transpose();
    T.block(0,3,3,1) = -T.block(0,0,3,3)*t;
    
    // compute GLC generators (d1, d2, d3)
    Eigen::Vector4f tmp;
    tmp << p1_, 1.0;
    Eigen::Vector3f p1 = (T * tmp).segment(0,3);
    tmp << p2_, 1.0;
    Eigen::Vector3f p2 = (T * tmp).segment(0,3);
    tmp << p3_, 1.0;
    Eigen::Vector3f p3 = (T * tmp).segment(0,3);
    tmp << q1_, 1.0;
    Eigen::Vector3f q1 = (T * tmp).segment(0,3);
    tmp << q2_, 1.0;
    Eigen::Vector3f q2 = (T * tmp).segment(0,3);
    tmp << q3_, 1.0;
    Eigen::Vector3f q3 = (T * tmp).segment(0,3);
    
    float z = p1(2)-q1(2);
    d1 << (p1(0)-q1(0))/z, (p1(2)*q1(0)-p1(0)*q1(2))/z,
          (p1(1)-q1(1))/z, (p1(2)*q1(1)-p1(1)*q1(2))/z;

    z = p2(2)-q2(2);
    d2 << (p2(0)-q2(0))/z, (p2(2)*q2(0)-p2(0)*q2(2))/z,
          (p2(1)-q2(1))/z, (p2(2)*q2(1)-p2(1)*q2(2))/z;

    z = p3(2)-q3(2);
    d3 << (p3(0)-q3(0))/z, (p3(2)*q3(0)-p3(0)*q3(2))/z,
          (p3(1)-q3(1))/z, (p3(2)*q3(1)-p3(1)*q3(2))/z;
}

void GLCCamera::updateUniforms(GLProgram& program, int flag) const
{
    Eigen::Matrix4f T;
    Eigen::Matrix2f d1, d2, d3;
    updateTransform(T, d1, d2, d3);

    program.setUniformData("u_uv1", glm::vec2(uv1_[0],uv1_[1]));
    program.setUniformData("u_uv2", glm::vec2(uv2_[0],uv2_[1]));
    program.setUniformData("u_uv3", glm::vec2(uv3_[0],uv3_[1]));
    
    program.setUniformData("u_d1", glm::vec4(d1(0,0),d1(0,1),d1(1,0),d1(1,1)));
    program.setUniformData("u_d2", glm::vec4(d2(0,0),d2(0,1),d2(1,0),d2(1,1)));
    program.setUniformData("u_d3", glm::vec4(d3(0,0),d3(0,1),d3(1,0),d3(1,1)));

//    program.setUniformData("u_p1", glm::vec3(p1_[0],p1_[1],p1_[2]));
//    program.setUniformData("u_p2", glm::vec3(p2_[0],p2_[1],p2_[2]));
//    program.setUniformData("u_p3", glm::vec3(p3_[0],p3_[1],p3_[2]));

    program.setUniformData("u_T", T);
    
    if(flag & U_GLCCAMERA_MV)
        program.setUniformData("u_modelview", Eigen::Matrix4f::Identity());
}

void GLCCamera::updateUniforms(GLProgram& program, Eigen::Matrix4f RT, int flag) const
{
    Eigen::Matrix4f T;
    Eigen::Matrix2f d1, d2, d3;
    updateTransform(T, d1, d2, d3);

    program.setUniformData("u_uv1", glm::vec2(uv1_[0],uv1_[1]));
    program.setUniformData("u_uv2", glm::vec2(uv2_[0],uv2_[1]));
    program.setUniformData("u_uv3", glm::vec2(uv3_[0],uv3_[1]));

    program.setUniformData("u_d1", glm::vec4(d1(0,0),d1(0,1),d1(1,0),d1(1,1)));
    program.setUniformData("u_d2", glm::vec4(d2(0,0),d2(0,1),d2(1,0),d2(1,1)));
    program.setUniformData("u_d3", glm::vec4(d3(0,0),d3(0,1),d3(1,0),d3(1,1)));

//    program.setUniformData("u_p1", glm::vec3(p1_[0],p1_[1],p1_[2]));
//    program.setUniformData("u_p2", glm::vec3(p2_[0],p2_[1],p2_[2]));
//    program.setUniformData("u_p3", glm::vec3(p3_[0],p3_[1],p3_[2]));

    program.setUniformData("u_T", T);
    
    if(flag & U_GLCCAMERA_MV)
        program.setUniformData("u_modelview", RT);
}

GLCCamera GLCCamera::parseCameraParams(std::string filename)
{
    std::ifstream fin(filename);
    GLCCamera Camera;
    if(fin.is_open()){
        fin >> Camera.name_;
        
        for(int i = 0; i < 3; ++i)
            fin >> Camera.p1_[i];
        for(int i = 0; i < 3; ++i)
            fin >> Camera.p2_[i];
        for(int i = 0; i < 3; ++i)
            fin >> Camera.p3_[i];
        for(int i = 0; i < 3; ++i)
            fin >> Camera.q1_[i];
        for(int i = 0; i < 3; ++i)
            fin >> Camera.q2_[i];
        for(int i = 0; i < 3; ++i)
            fin >> Camera.q3_[i];

        fin >> Camera.zNear_;
        fin >> Camera.zFar_;
        
        fin >> Camera.width_;
        fin >> Camera.height_;
    }
    else{
        std::cout << "Warning: file does not exist. " << filename << std::endl;
    }

    return Camera;
}


#ifdef WITH_IMGUI
void GLCCamera::updateIMGUI()
{
    if (ImGui::CollapsingHeader("GLCCamera Parameters")){
        ImGui::InputFloat("zNear", &zNear_);
        ImGui::InputFloat("zFar", &zFar_);
        ImGui::InputFloat3("p1", &p1_[0]);
        ImGui::InputFloat3("p2", &p2_[0]);
        ImGui::InputFloat3("p3", &p3_[0]);
        ImGui::InputFloat3("q1", &q1_[0]);
        ImGui::InputFloat3("q2", &q2_[0]);
        ImGui::InputFloat3("q3", &q3_[0]);
    }
}
#endif
