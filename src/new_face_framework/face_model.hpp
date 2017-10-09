//
//  face_model.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#pragma once

#include <array>
#include <mutex>

#include "EigenHelper.h"

#ifdef WITH_IMGUI
#include "imgui.h"
#endif

struct BaseFaceModel;
typedef std::shared_ptr<BaseFaceModel> FaceModelPtr;

// all temporary data is stored in FaceData
struct FaceData
{
    FaceData(FaceModelPtr model);
    
    FaceModelPtr model_;
    
    Eigen::VectorXf idCoeff;
    Eigen::VectorXf exCoeff;
    Eigen::VectorXf alCoeff;
    
    Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
    Eigen::Matrix3Xf SH = Eigen::Matrix3Xf::Zero(3,9);
    
    Eigen::VectorXf pts_;  // current shape
    Eigen::MatrixX3f nml_; // current normal
    Eigen::VectorXf neu_;  // current neutral
    Eigen::VectorXf clr_;  // current color
    
    Eigen::VectorXf d_ex_; // expression delta
    Eigen::VectorXf d_id_; // identity delta
    
    Eigen::MatrixXf w_ex_; // blendshape basis for bilinear model
    Eigen::MatrixXf w_id_; // identity basis for bilinear model
    
    bool id_opt_ = false; // for bilinear identity optimization
    
    void updateIdentity();
    void updateExpression();
    void updateColor();
    
    void updateAll();
    void updateShape();
    
    Eigen::Vector3f computeV(int vidx) const;
    Eigen::Ref<const Eigen::MatrixXf> dID(int vidx, int size) const;
    Eigen::Ref<const Eigen::MatrixXf> dEX(int vidx, int size) const;
    Eigen::Ref<const Eigen::MatrixXf> dCL(int vidx, int size) const;
    
    const Eigen::Matrix3Xf& dIDEdge(int fidx, int eidx) const;
    const Eigen::Matrix3Xf& dEXEdge(int fidx, int eidx) const;
    
    void dSym(int symidx, int axis, int nid, int nex, Eigen::Vector3f& v, Eigen::MatrixXf& dv) const;
    
    void init();
    void saveObj(const std::string& filename);
    
#ifdef WITH_IMGUI
    void updateIMGUI();
#endif
};

struct BaseFaceModel
{
    // texture coordinates
    Eigen::MatrixX2f uvs_;
    
    // triangle list
    Eigen::MatrixX3i tri_pts_;
    Eigen::MatrixX3i tri_uv_;
    
    virtual void updateExpression(FaceData& data){ throw std::runtime_error( "Error: Base class is called..."); }
    virtual void updateIdentity(FaceData& data){ throw std::runtime_error( "Error: Base class is called..."); }
    virtual void updateColor(FaceData& data){ throw std::runtime_error( "Error: Base class is called..."); }
    
    virtual int n_id() const { return 0;}
    virtual int n_exp() const { return 0;}
    virtual int n_clr() const { return 0;}
    
    virtual int n_sym_pair(){ return 0;}
    
    virtual Eigen::Vector3f computeV(int vidx, const FaceData& data) const { throw std::runtime_error( "Error: Base class is called..."); }
    virtual Eigen::Ref<const Eigen::MatrixXf> dID(int vidx, int size, const FaceData& data) const { throw std::runtime_error( "Error: Base class is called..."); }
    virtual Eigen::Ref<const Eigen::MatrixXf> dEX(int vidx, int size, const FaceData& data) const { throw std::runtime_error( "Error: Base class is called..."); }
    virtual Eigen::Ref<const Eigen::MatrixXf> dCL(int vidx, int size, const FaceData& data) const { throw std::runtime_error( "Error: Base class is called..."); }
    
    virtual const Eigen::Matrix3Xf& dIDEdge(int fidx, int eidx, const FaceData& data) const { throw std::runtime_error( "Error: Base class is called..."); }
    virtual const Eigen::Matrix3Xf& dEXEdge(int fidx, int eidx, const FaceData& data) const { throw std::runtime_error( "Error: Base class is called..."); }
    
    virtual const Eigen::VectorXf& sigmaID() const { throw std::runtime_error( "Error: Base class is called..."); }
    virtual const Eigen::VectorXf& sigmaEX() const { throw std::runtime_error( "Error: Base class is called..."); }
    virtual const Eigen::VectorXf& sigmaCL() const { throw std::runtime_error( "Error: Base class is called..."); }
    
    virtual const Eigen::VectorXf& meanShape() const { throw std::runtime_error( "Error: Base class is called..."); }
    
    virtual void dSym(int symidx, int axis, int nid, int nex, Eigen::Vector3f& v, Eigen::MatrixXf& dv, const FaceData& data)
    const { throw std::runtime_error( "Error: Base class is called..."); }
    
    virtual FaceModelPtr LoadModel(const std::string& file){ throw std::runtime_error( "Error: Base class is called..."); }
};

struct LinearFaceModel : public BaseFaceModel
{
    // identity basis
    Eigen::VectorXf mu_id_;
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
    
    void saveBinaryModel(const std::string& file);
    void loadBinaryModel(const std::string& file);
    
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
    
    virtual FaceModelPtr LoadModel(const std::string& file);
};

struct BiLinearFaceModel : public BaseFaceModel
{
    Eigen::VectorXf mu_id_;
    Eigen::VectorXf sigma_id_;
    
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
    //virtual void updateColor(FaceData& data);
    
    inline virtual int n_exp() const { return (int)Cshape_.size()-1;}
    inline virtual int n_id() const { return (int)Cshape_[0].cols();} // basis includes neutral
    inline virtual int n_clr() const { return 0;}
    
    virtual Eigen::Vector3f computeV(int vidx, const FaceData& data) const;
    virtual Eigen::Ref<const Eigen::MatrixXf> dID(int vidx, int size, const FaceData& data) const;
    virtual Eigen::Ref<const Eigen::MatrixXf> dEX(int vidx, int size, const FaceData& data) const;
    
    inline virtual const Eigen::VectorXf& sigmaID() const { return sigma_id_; }
    inline virtual const Eigen::VectorXf& sigmaEX() const { return sigma_ex_; }
    
    inline virtual const Eigen::VectorXf& meanShape() const { return mu_id_; }

    virtual FaceModelPtr LoadModel(const std::string& file);
};
