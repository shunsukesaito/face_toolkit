//
//  camera.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include "camera.h"

// for sphere rendering
static const int sp_width_ = 400, sp_height_ = 400;
static const Eigen::Matrix4f sp_extrinsic_ = (Eigen::Matrix4f() << 1,0,0,0,0,-1,0,0,0,0,-1,50.0,0,0,0,1).finished();
static const Eigen::Matrix4f sp_intrinsic_ = (Eigen::Matrix4f() << 800,0,200,0,0,800,200,0,0,0,1,0,0,0,0,1).finished();


Camera::Camera(const Eigen::Matrix4f& RT, const Eigen::Matrix4f& K, int w, int h, float zN, float zF, bool c2w)
: intrinsic_(K), width_(w), height_(h), zNear_(zN), zFar_(zF)
{
    if(c2w){
        Eigen::Matrix3f Rt = RT.block<3,3>(0,0).transpose();
        extrinsic_.block(0, 3, 3, 1) = -Rt * RT.block(0, 3, 3, 1);
        extrinsic_.block(0, 0, 3, 3) = Rt;
        
        Eigen::Matrix4f flip; flip.setIdentity();
        flip.block(0, 0, 3, 3) = Eigen::Quaternion<float>(0, 1, 0, 0).toRotationMatrix();
        
        extrinsic_ = flip * extrinsic_;
    }
    else{
        extrinsic_ = RT;
    }
}

Camera::Camera(const Camera& other)
{
    name_ = other.name_;
    extrinsic_ = other.extrinsic_;
    intrinsic_ = other.intrinsic_;
    
    zNear_ = other.zNear_;
    zFar_ = other.zFar_;
    
    width_ = other.width_;
    height_ = other.height_;
    
    distCoeff_ = other.distCoeff_;
    
    weakPersp_ = other.weakPersp_;
}

void Camera::initializeUniforms(GLProgram& program, int flag)
{
    if(flag & U_CAMERA_MVP)
        program.createUniform("u_mvp", DataType::MATRIX44);
    if(flag & U_CAMERA_MV)
        program.createUniform("u_modelview", DataType::MATRIX44);
    if(flag & U_CAMERA_WORLD)
        program.createUniform("u_world", DataType::MATRIX44);
    if(flag & U_CAMERA_SHADOW)
        program.createUniform("u_shadow_mvp", DataType::MATRIX44);
    if(flag & U_CAMERA_POS)
        program.createUniform("u_camera_pos", DataType::VECTOR3);
}

void Camera::updateUniforms(GLProgram& program, int flag) const
{
    static const Eigen::Matrix4f biasMatrix =
    (Eigen::Matrix4f() << 0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0, 0.5, 0.5, 0, 0, 0, 1.0).finished();
    
    Eigen::Matrix4f MVP, MV;
    Eigen::Matrix4f perspective;
    if(weakPersp_){
        perspective = OrthogonalProjection(width_, height_, zNear_, zFar_);
    }
    else
        perspective = PerspectiveFromVision(intrinsic_, width_, height_, zNear_, zFar_);
    
    MV = extrinsic_;
    MVP = perspective * MV;
    
    Eigen::Matrix4f inv_extrinsic = extrinsic_.inverse();
    glm::vec3 pos(inv_extrinsic(0,3),inv_extrinsic(1,3),inv_extrinsic(2,3));
    
    Eigen::Matrix4f shadowMVP = biasMatrix * MVP;
    
    if(flag & U_CAMERA_MVP)
        program.setUniformData("u_mvp", MVP);
    if(flag & U_CAMERA_MV)
        program.setUniformData("u_modelview", MV);
    if(flag & U_CAMERA_WORLD)
        program.setUniformData("u_world", Eigen::Matrix4f::Identity());
    if(flag & U_CAMERA_SHADOW)
        program.setUniformData("u_shadow_mvp", shadowMVP);
    if(flag & U_CAMERA_POS)
        program.setUniformData("u_camera_pos", pos);
}

void Camera::updateUniforms(GLProgram& program, Eigen::Matrix4f RT, int flag) const
{
    static const Eigen::Matrix4f biasMatrix =
    (Eigen::Matrix4f() << 0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0, 0.5, 0.5, 0, 0, 0, 1.0).finished();

    Eigen::Matrix4f MVP, MV;
    Eigen::Matrix4f perspective;
    if(weakPersp_){
        perspective = OrthogonalProjection(width_, height_, zNear_, zFar_);
        RT.block(0,0,3,3) *= RT(2,3);
        RT(2,3) = 500.0;
        MV = RT;
    }
    else{
        perspective = PerspectiveFromVision(intrinsic_, width_, height_, zNear_, zFar_);
        MV = extrinsic_ * RT;
    }
    MVP = perspective * MV;
    
    Eigen::Matrix4f inv_extrinsic = extrinsic_.inverse();
    glm::vec3 pos(inv_extrinsic(0,3),inv_extrinsic(1,3),inv_extrinsic(2,3));
    
    Eigen::Matrix4f shadowMVP = biasMatrix * MVP;
    
    if(flag & U_CAMERA_MVP)
        program.setUniformData("u_mvp", MVP);
    if(flag & U_CAMERA_MV)
        program.setUniformData("u_modelview", MV);
    if(flag & U_CAMERA_WORLD)
        program.setUniformData("u_world", RT);
    if(flag & U_CAMERA_SHADOW)
        program.setUniformData("u_shadow_mvp", shadowMVP);
    if(flag & U_CAMERA_POS)
        program.setUniformData("u_camera_pos", pos);
}

void Camera::updateUniforms4Sphere(GLProgram& program, int flag) const
{
    static const Eigen::Matrix4f biasMatrix =
    (Eigen::Matrix4f() << 0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0, 0.5, 0.5, 0, 0, 0, 1.0).finished();
    
    Eigen::Matrix4f MVP;
    Eigen::Matrix4f perspective = PerspectiveFromVision(sp_intrinsic_, sp_width_, sp_height_, zNear_, zFar_);
    Eigen::Matrix4f MV = extrinsic_;
    MV.block<3,1>(0,3) = sp_extrinsic_.block<3,1>(0,3);
    MVP = perspective * MV;
    
    Eigen::Matrix4f inv_extrinsic = extrinsic_.inverse();
    glm::vec3 pos(inv_extrinsic(0,3),inv_extrinsic(1,3),inv_extrinsic(2,3));
    
    Eigen::Matrix4f shadowMVP = biasMatrix * MVP;
    
    if(flag & U_CAMERA_MVP)
        program.setUniformData("u_mvp", MVP);
    if(flag & U_CAMERA_MV)
        program.setUniformData("u_modelview", MV);
    if(flag & U_CAMERA_WORLD)
        program.setUniformData("u_world", Eigen::Matrix4f::Identity());
    if(flag & U_CAMERA_SHADOW)
        program.setUniformData("u_shadow_mvp", shadowMVP);
    if(flag & U_CAMERA_POS)
        program.setUniformData("u_camera_pos", pos);
}

Camera Camera::craeteFromFOV(int w, int h, int FOV)
{
    Camera camera;
    
    camera.width_ = w;
    camera.height_ = h;
    
    Eigen::Matrix3f I;
    float focal = 0.5*(float)h/tan((float)FOV/360.0*PI);
    I << focal, 0, 0.5 * (float)w,
         0, focal, 0.5 * (float)h,
         0, 0, 1;
    Eigen::Matrix4f RT;
    RT << 1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 30.0,
        0, 0, 0, 1;
    camera.intrinsic_.block(0,0,3,3) = I;
    camera.extrinsic_ = RT;
    
    return camera;
}

Camera Camera::parseCameraParams(std::string filename)
{
    std::ifstream fin(filename);
    Camera camera;
    if(fin.is_open()){
        fin >> camera.name_;
        
        camera.extrinsic_.setIdentity();
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
                fin >> camera.extrinsic_(i,j);
            }
        }
    
        camera.intrinsic_.setIdentity();
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                fin >> camera.intrinsic_(i,j);
            }
        }
        fin >> camera.width_;
        fin >> camera.height_;
        
        fin >> camera.zNear_;
        fin >> camera.zFar_;
        
        std::vector<float> distCoeff;
        while(1){
            float tmp;
            fin >> tmp;
            if(fin.good())
                distCoeff.push_back(tmp);
            else
                break;
        }
        camera.distCoeff_.resize(distCoeff.size());
        for(int i = 0; i < distCoeff.size(); ++i)
        {
            camera.distCoeff_[i] = distCoeff[i];
        }
    }
    else{
        std::cout << "Warning: file does not exist. " << filename << std::endl;
    }
    
    return camera;
}

Eigen::Matrix4f Camera::loadKFromTxt(std::string filename)
{
    std::ifstream fin(filename);
    Eigen::Matrix4f K = Eigen::Matrix4f::Identity();
    if(fin.is_open()){
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                fin >> K(i,j);
            }
        }
    }
    else{
        std::cout << "Warning: file does not exist. " << filename << std::endl;
    }
    
    return K;
}

Eigen::Matrix4f Camera::loadRTFromTxt(std::string filename)
{
    std::ifstream fin(filename);
    Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
    if(fin.is_open()){
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
                fin >> RT(i,j);
            }
        }
    }
    else{
        std::cout << "Warning: file does not exist. " << filename << std::endl;
    }
    
    return RT;
}


#ifdef WITH_IMGUI
void Camera::updateIMGUI()
{
    Eigen::Vector3f euler = Eigen::matToEulerAngle(extrinsic_.block<3,3>(0,0));
    if (ImGui::CollapsingHeader("Camera Parameters")){
        ImGui::InputFloat("zNear", &zNear_);
        ImGui::InputFloat("zFar", &zFar_);
        ImGui::InputFloat3("Rot", &euler(0), -1, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputFloat3("Tr", &extrinsic_(0,3));
        ImGui::InputFloat("fx", &intrinsic_(0,0));
        ImGui::InputFloat("fy", &intrinsic_(1,1));
        ImGui::InputFloat("px", &intrinsic_(0,2));
        ImGui::InputFloat("py", &intrinsic_(1,2));
        ImGui::InputInt("width", &width_);
        ImGui::InputInt("height", &height_);
    }
}
#endif
