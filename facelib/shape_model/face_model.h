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

#include <unordered_map>
#include <array>
#include <mutex>
#include <fstream>
#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <utility/EigenHelper.h>
#include <utility/mesh_data.h>

#ifdef WITH_IMGUI
#include <imgui.h>
#endif

struct MeshData;

struct BaseFaceModel;
typedef std::shared_ptr<BaseFaceModel> FaceModelPtr;

struct FaceData;
typedef std::shared_ptr<FaceData> FaceDataPtr;

struct FaceParams
{
    Eigen::VectorXf idCoeff;
    Eigen::VectorXf exCoeff;
    Eigen::VectorXf alCoeff;
    
    Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
    Eigen::Matrix3Xf SH = Eigen::Matrix3Xf::Zero(3,9);
};

// all temporary data is stored in FaceData
struct FaceData : public MeshData
{
    void setFaceModel(FaceModelPtr model);
    
    FaceModelPtr model_ = NULL;
    
    Eigen::VectorXf idCoeff;
    Eigen::VectorXf exCoeff;
    Eigen::VectorXf alCoeff;
    
    // for IMGUI update check
    Eigen::VectorXf idCoeff_prev;
    Eigen::VectorXf exCoeff_prev;
    Eigen::VectorXf alCoeff_prev;
    
    Eigen::VectorXf neu_;  // current neutral
    
    Eigen::VectorXf d_ex_; // expression delta
    Eigen::VectorXf d_id_; // identity delta
    
    Eigen::MatrixXf w_ex_; // blendshape basis for bilinear model
    Eigen::MatrixXf w_id_; // identity basis for bilinear model

    std::vector<int> cont_idx_;
        
    bool opt_id_only_ = false; // it should be true only for id opt
    bool opt_ex_only_ = false; // it should be true only for ex opt
    
    FaceData(){}
    virtual ~FaceData(){}

    void updateParams(const FaceParams& fp);
    
    void updateIdentity();
    void updateExpression();
    void updateColor();
    
    void updateAll();
    void updateShape();

    void updateContour(const Eigen::Matrix4f& K, const Eigen::Matrix4f& RTc);

    void updateContourBV(const Eigen::Matrix4f& K, const Eigen::Matrix4f& RTc);
    void updateContourPIN(const Eigen::Matrix4f& K, const Eigen::Matrix4f& RTc);
    void updateContourFW(const Eigen::Matrix4f& K, const Eigen::Matrix4f& RTc);
    
    Eigen::Vector3f computeV(int vidx) const;
    Eigen::Ref<const Eigen::MatrixXf> dID(int vidx, int size) const;
    Eigen::Ref<const Eigen::MatrixXf> dEX(int vidx, int size) const;
    Eigen::Ref<const Eigen::MatrixXf> dCL(int vidx, int size) const;
    
    const Eigen::Matrix3Xf& dIDEdge(int fidx, int eidx) const;
    const Eigen::Matrix3Xf& dEXEdge(int fidx, int eidx) const;
    
    void dSym(int symidx, int axis, int nid, int nex, Eigen::Vector3f& v, Eigen::MatrixXf& dv) const;
    
    // for Cao's tracker
    void updateLandmarks(const cv::Mat_<float>& x, Eigen::Matrix4f& K, cv::Mat_<cv::Vec2f>& p2d) const ;
    void updateLandmarks(const cv::Mat_<float>& x, Eigen::Matrix4f& K, const cv::Mat_<float>& xid, cv::Mat_<cv::Vec2f>& p2d) const;

    void init();
    
#ifdef WITH_IMGUI
    virtual void updateIMGUI();
#endif
};

struct BaseFaceModel
{
    std::string fm_type_;
    
    // texture coordinates
    Eigen::MatrixX2f uvs_;
    
    // triangle list
    Eigen::MatrixX3i tri_pts_;
    Eigen::MatrixX3i tri_uv_;
    
    // base vertex positions
    Eigen::VectorXf mu_id_;
    
    // for rendering 
    std::unordered_map<std::string,unsigned> maps_;

    // contour candidates for fitting
    std::vector<std::vector<int>> cont_candi_;
    
    void loadContourList(std::string file);
    void loadMeanFromObj(const std::string& filename);
    
    bool has_id() const {return (this->n_id() != 0);}
    bool has_exp() const {return (this->n_exp() != 0);}
    bool has_clr() const {return (this->n_clr() != 0);}
    
    virtual void updateExpression(FaceData& data){ throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    virtual void updateIdentity(FaceData& data){ throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    virtual void updateColor(FaceData& data){ throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    
    virtual int n_id() const { return 0;}
    virtual int n_exp() const { return 0;}
    virtual int n_clr() const { return 0;}
    
    virtual int n_sym_pair(){ return 0;}
    
    virtual Eigen::Vector3f computeV(int vidx, const FaceData& data) const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    virtual Eigen::Ref<const Eigen::MatrixXf> dID(int vidx, int size, const FaceData& data) const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    virtual Eigen::Ref<const Eigen::MatrixXf> dEX(int vidx, int size, const FaceData& data) const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    virtual Eigen::Ref<const Eigen::MatrixXf> dCL(int vidx, int size, const FaceData& data) const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    
    virtual const Eigen::Matrix3Xf& dIDEdge(int fidx, int eidx, const FaceData& data) const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    virtual const Eigen::Matrix3Xf& dEXEdge(int fidx, int eidx, const FaceData& data) const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    
    virtual const Eigen::VectorXf& sigmaID() const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    virtual const Eigen::VectorXf& sigmaEX() const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    virtual const Eigen::VectorXf& sigmaCL() const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    
    virtual const Eigen::VectorXf& meanShape() const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
    
    virtual void dSym(int symidx, int axis, int nid, int nex, Eigen::Vector3f& v, Eigen::MatrixXf& dv, const FaceData& data)
    const { throw std::runtime_error( "Error: Base class (FaceModel) is called..."); }
};

struct LinearFaceModel : public BaseFaceModel
{
    // identity basis
    Eigen::VectorXf sigma_id_;
    Eigen::MatrixXf w_id_;
    
    // expression basis
    Eigen::VectorXf mu_ex_;
    Eigen::VectorXf sigma_ex_;
    Eigen::MatrixXf w_ex_;
    
    // albedo basis
    Eigen::VectorXf mu_cl_;
    Eigen::VectorXf sigma_cl_;
    Eigen::MatrixXf w_cl_;
    
    Eigen::MatrixX2i sym_list_;
    
    std::vector<std::array<Eigen::Matrix3Xf, 2>> id_edge_;
    std::vector<std::array<Eigen::Matrix3Xf, 2>> ex_edge_;
    
    void loadLightStageData(const std::string& data_dir);
    void loadDeepLSData(const std::string& data_dir);
    
    void saveBinaryModel(const std::string& file);
    void loadBinaryModel(const std::string& file);
    
    void loadModelFromObj(const std::string& obj_dir, int id_size, int ex_size);
    
    void loadOldBinaryModel(const std::string& modelfile, const std::string& meshfile);
    void loadMMBinaryModel(const std::string& modelfile);
    
    virtual void updateExpression(FaceData& data);
    virtual void updateIdentity(FaceData& data);
    virtual void updateColor(FaceData& data);
    
    inline virtual int n_id() const { return (int)sigma_id_.size();}
    inline virtual int n_exp() const { return (int)sigma_ex_.size();}
    inline virtual int n_clr() const { return (int)sigma_cl_.size();}
    
    inline virtual int n_sym_pair(){ return (int)sym_list_.rows();}
    
    virtual Eigen::Vector3f computeV(int vidx, const FaceData& data) const;
    virtual Eigen::Ref<const Eigen::MatrixXf> dID(int vidx, int size, const FaceData& data) const;
    virtual Eigen::Ref<const Eigen::MatrixXf> dEX(int vidx, int size, const FaceData& data) const;
    virtual Eigen::Ref<const Eigen::MatrixXf> dCL(int vidx, int size, const FaceData& data) const;
    
    virtual const Eigen::Matrix3Xf& dIDEdge(int fidx, int eidx, const FaceData& data) const;
    virtual const Eigen::Matrix3Xf& dEXEdge(int fidx, int eidx, const FaceData& data) const;
    
    inline virtual const Eigen::VectorXf& sigmaID() const { return sigma_id_; }
    inline virtual const Eigen::VectorXf& sigmaEX() const { return sigma_ex_; }
    inline virtual const Eigen::VectorXf& sigmaCL() const { return sigma_cl_; }
    
    inline virtual const Eigen::VectorXf& meanShape() const { return mu_id_; }
    
    virtual void dSym(int symidx, int axis, int nid, int nex, Eigen::Vector3f& v, Eigen::MatrixXf& dv, const FaceData& data) const;
    
    static FaceModelPtr LoadModel(const std::string& file, const std::string& fm_type);
    static FaceModelPtr LoadLSData(const std::string& data_dir, bool deep = false);
    static FaceModelPtr LoadMesh(const std::string& file);
};

struct BiLinearFaceModel : public BaseFaceModel
{
    Eigen::VectorXf sigma_id_;
    
    Eigen::MatrixXf w_mu_ex_;
    Eigen::VectorXf sigma_ex_;
    
    // bilinear core tensor (identity x expressions)
    std::vector<Eigen::MatrixXf> Cshape_;
    
    void saveBinaryModel(const std::string& file);
    void loadBinaryModel(const std::string& file);
    
    void computeModelFW(const std::string& mesh_dir,
                        const std::string& topo_mesh,
                        float scale);
    
    virtual void updateExpression(FaceData& data);
    virtual void updateIdentity(FaceData& data);
    virtual void updateColor(FaceData& data){}
    
    inline virtual int n_exp() const { return (int)Cshape_.size()-1;}
    inline virtual int n_id() const { return (int)Cshape_[0].cols();} // basis includes neutral
    inline virtual int n_clr() const { return 0;}
    
    virtual Eigen::Vector3f computeV(int vidx, const FaceData& data) const;
    virtual Eigen::Ref<const Eigen::MatrixXf> dID(int vidx, int size, const FaceData& data) const;
    virtual Eigen::Ref<const Eigen::MatrixXf> dEX(int vidx, int size, const FaceData& data) const;
    
    inline virtual const Eigen::VectorXf& sigmaID() const { return sigma_id_; }
    inline virtual const Eigen::VectorXf& sigmaEX() const { return sigma_ex_; }
    
    inline virtual const Eigen::VectorXf& meanShape() const { return mu_id_; }

    static FaceModelPtr LoadModel(const std::string& file, const std::string& fm_type);
};
