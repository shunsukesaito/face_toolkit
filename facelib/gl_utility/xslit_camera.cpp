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
#include "xslit_camera.h"

void createCircleXSlits(float radius, const Eigen::Vector3f& q1, const Eigen::Vector3f& q2, int nCam,
                        int width, int height, std::vector<XSlitCamera>& cameras)
{
    cameras.clear();
    float unit = 2.0f*M_PI/(float)nCam;
    Eigen::Vector3f p1, p2;
    float h = 0.5f*(q1[1]+q2[1]);
    for(int i = 0; i < nCam; ++i)
    {
        p1 << radius*cos((float)i*unit), h, radius*sin((float)i*unit);
        p2 << radius*cos((float)(i+1)*unit), h, radius*sin((float)(i+1)*unit);
        cameras.push_back(XSlitCamera(p1,p2,q1,q2,0.01f,0.99f,width,height));
    }
}

XSlitCamera::XSlitCamera(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
                         const Eigen::Vector3f& q1, const Eigen::Vector3f& q2, float zNear, float zFar, int w, int h)
: p1_(p1), p2_(p2), q1_(q1), q2_(q2), zNear_(zNear), zFar_(zFar), width_(w), height_(h)
{
 }

XSlitCamera::XSlitCamera(const XSlitCamera& other)
{
    name_ = other.name_;
    
    p1_ = other.p1_;
    p2_ = other.p2_;
    q1_ = other.q1_;
    q2_ = other.q2_;
    
    zNear_ = other.zNear_;
    zFar_ = other.zFar_;
    
    width_ = other.width_;
    height_ = other.height_;
}

void XSlitCamera::initializeUniforms(GLProgram& program, int flag)
{
    program.createUniform("u_d1", DataType::VECTOR2);
    program.createUniform("u_d2", DataType::VECTOR2);
    program.createUniform("u_d3", DataType::VECTOR2);

    program.createUniform("u_T", DataType::MATRIX44);
    if(flag & U_XSCAMERA_MV)
        program.createUniform("u_modelview", DataType::MATRIX44);
}

void XSlitCamera::updateTransform(Eigen::Matrix4f& T, Eigen::Vector2f& d1, Eigen::Vector2f& d2, Eigen::Vector2f& d3) const
{
    assert(zNear_ > 0);
    assert(zFar_ > zNear_);
    
    float lp = (p1_-p2_).norm();
    float lq = (q1_-q2_).norm();
    Eigen::Vector3f nx = -(p1_-p2_).normalized();
    Eigen::Vector3f ny = -(q1_-q2_).normalized();
    Eigen::Vector3f nz = -(nx.cross(ny)).normalized();
    
    Eigen::Vector3f cp = 0.5f*(p1_+p2_);
    float z = ((q1_-cp).cross(q2_-cp)).norm()/lq;
    
    // scale factor
    Eigen::Vector3f s;
    float sx = 1.0f/((1.0f-zNear_)*lp);
    float sy = 1.0f/(zNear_*lq);
    float sz = 1.0f/((zFar_-zNear_)*z);
    s << sx, sy, sz;
    
    // rotation
    Eigen::Matrix3f R;
    R << nx, ny, nz;
    
    // translation
    Eigen::Vector3f t;
    t = zNear_*(q1_-p1_)+p1_;

    // compose them into 4x4 transform matrix
    T.setIdentity();
    T.block(0,0,3,3) = s.asDiagonal()*R.transpose();
    T.block(0,3,3,1) = -T.block(0,0,3,3)*t;
    
    // compute GLC generators (d1, d2, d3)
    Eigen::Vector4f pq;
    
    pq << q1_-p1_, 0.0;
    pq = T * pq;
    pq[2] = (pq[2] > 1.e-8f) ? pq[2] : 1.e-8f;
    d1 << pq[0]/pq[2], pq[1]/pq[2];
    
    pq << q1_-p2_, 0.0;
    pq = T * pq;
    pq[2] = (pq[2] > 1.e-8f) ? pq[2] : 1.e-8f;
    d2 << pq[0]/pq[2], pq[1]/pq[2];
    
    pq << q2_-p1_, 0.0;
    pq = T * pq;
    pq[2] = (pq[2] > 1.e-8f) ? pq[2] : 1.e-8f;
    d3 << pq[0]/pq[2], pq[1]/pq[2];
}

void XSlitCamera::updateUniforms(GLProgram& program, int flag) const
{
    Eigen::Matrix4f T;
    Eigen::Vector2f d1, d2, d3;
    updateTransform(T, d1, d2, d3);
    
    program.setUniformData("u_d1", glm::vec2(d1[0],d1[1]));
    program.setUniformData("u_d2", glm::vec2(d2[0],d2[1]));
    program.setUniformData("u_d3", glm::vec2(d3[0],d3[1]));
    
    program.setUniformData("u_T", T);
    
    if(flag & U_XSCAMERA_MV)
        program.setUniformData("u_modelview", Eigen::Matrix4f::Identity());
}

void XSlitCamera::updateUniforms(GLProgram& program, Eigen::Matrix4f RT, int flag) const
{
    Eigen::Matrix4f T;
    Eigen::Vector2f d1, d2, d3;
    updateTransform(T, d1, d2, d3);

    program.setUniformData("u_d1", glm::vec2(d1[0],d1[1]));
    program.setUniformData("u_d2", glm::vec2(d2[0],d2[1]));
    program.setUniformData("u_d3", glm::vec2(d3[0],d3[1]));

    program.setUniformData("u_T", T);
    
    if(flag & U_XSCAMERA_MV)
        program.setUniformData("u_modelview", RT);
}

XSlitCamera XSlitCamera::parseCameraParams(std::string filename)
{
    std::ifstream fin(filename);
    XSlitCamera Camera;
    if(fin.is_open()){
        fin >> Camera.name_;
        
        for(int i = 0; i < 3; ++i)
            fin >> Camera.p1_[i];
        for(int i = 0; i < 3; ++i)
            fin >> Camera.p2_[i];
        for(int i = 0; i < 3; ++i)
            fin >> Camera.q1_[i];
        for(int i = 0; i < 3; ++i)
            fin >> Camera.q2_[i];

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
void XSlitCamera::updateIMGUI()
{
    if (ImGui::CollapsingHeader("XSlitCamera Parameters")){
        ImGui::InputFloat("zNear", &zNear_);
        ImGui::InputFloat("zFar", &zFar_);
        ImGui::InputFloat3("p1", &p1_[0]);
        ImGui::InputFloat3("p2", &p2_[0]);
        ImGui::InputFloat3("q1", &q1_[0]);
        ImGui::InputFloat3("q2", &q2_[0]);
    }
}
#endif
