#ifndef HFM_UTILITIES_OPENMESHUTILITIES_H
#define HFM_UTILITIES_OPENMESHUTILITIES_H

// std includes
#include <vector>
#include <memory>
#include <iostream>

// external includes
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "EigenHelper.h"

namespace hfm
{

namespace PreDefinedColors
{
    static const OpenMesh::Vec4f grey = OpenMesh::Vec4f(0.9f, 0.9f, 0.9f, 1.0f);
    static const OpenMesh::Vec4f paper_blue = OpenMesh::Vec4f(0.647f, 0.815f, 0.929f, 1.f);
    static const OpenMesh::Vec4f paper_purple = OpenMesh::Vec4f(0.784f, 0.752f, 0.929f, 1.f);
    static const OpenMesh::Vec4f paper_pink = OpenMesh::Vec4f(0.933f, 0.694f, 0.803f, 1.f);

    static const OpenMesh::Vec4f epfl_blue = OpenMesh::Vec4f(0.235f, 0.411f, 0.639f, 1.0f);
    static const OpenMesh::Vec4f epfl_skin = OpenMesh::Vec4f(0.858f, 0.760f, 0.662f, 1.0f);
    static const OpenMesh::Vec4f epfl_lightblue = OpenMesh::Vec4f(0.533f, 0.690f, 0.796f, 1.0f);

    static const OpenMesh::Vec4f zju_skin = OpenMesh::Vec4f(0.850f, 0.780f, 0.737f, 1.0f);
    static const OpenMesh::Vec4f zju_lightblue = OpenMesh::Vec4f(0.756f, 0.827f, 0.890f, 1.0f);

    // static const OpenMesh::Vec4f my_skin = OpenMesh::Vec4f(0.777, 0.439, 0.267, 1);
    static const OpenMesh::Vec4f my_skin = OpenMesh::Vec4f(0.859f, 0.651f, 0.569f, 1.0f);
    // static const OpenMesh::Vec4f the_mask_green = OpenMesh::Vec4f(0.510, 0.745, 0.137, 1);
    static const OpenMesh::Vec4f the_mask_green = OpenMesh::Vec4f(0.447f, 0.706f, 0.106f, 1.0f);

    static const OpenMesh::Vec4f default_color = grey;
}

struct MeshTraits : public OpenMesh::DefaultTraits{

    typedef OpenMesh::Vec3f Point;
    typedef OpenMesh::Vec3f Normal;
    typedef OpenMesh::Vec2f TexCoord2D;
    typedef OpenMesh::Vec4f Color;

    VertexAttributes( OpenMesh::Attributes::Normal |
                      OpenMesh::Attributes::Color |
                      OpenMesh::Attributes::TexCoord2D |
                      OpenMesh::Attributes::Status);

    FaceAttributes( OpenMesh::Attributes::Normal |
                    OpenMesh::Attributes::Color |
                    OpenMesh::Attributes::Status);
    
    EdgeAttributes(OpenMesh::Attributes::Status);

    // example:
    // use mesh.data(v_it).set_selected(bool) to set setected_ value
    VertexTraits
    {
    private:
        bool selected_;
        double double_value_;
        int comp_id_;
        
    public:
        VertexT(): selected_(false), double_value_(0.0), comp_id_(-1) {}

        const bool& is_selected() const{return selected_;}
        void set_selected(const bool selected) {selected_ = selected;}

        const double& get_double_value() const{return double_value_;}
        void set_double_value(const double double_value) {double_value_ = double_value;}
        
        const int& get_comp_id() const{return comp_id_;}
        void set_comp_id(const int comp_id) {comp_id_ = comp_id;}
    };

    HalfedgeTraits
    {
    private:
        Color color_;
        bool selected_;

    public:
        HalfedgeT() : color_( Color(0.0,0.0,0.0,1.0) ),selected_(false){}

        const Color& get_color() const {return color_;}
        void set_color(const Color& color){color_ = color;}

        const bool& is_selected() const{return selected_;}
        void set_selected(const bool selected) {selected_ = selected;}
    };

    FaceTraits
    {
        private:
            bool selected_;
            int groupId_;

        public:
            FaceT():selected_(false), groupId_(-1){}

            const bool& is_selected() const{return selected_;}
            void set_selected(const bool selected) {selected_ = selected;}
            const int& groupId() const {return groupId_;}
            void set_groupId(const int idx) {groupId_ = idx;}
    };
};

typedef OpenMesh::TriMesh_ArrayKernelT<MeshTraits> TriangleMesh;
typedef std::shared_ptr<TriangleMesh> TriangleMeshPtr;

// OpenMesh utility functions
std::vector< Eigen::Vector3f > GetVerticesFromMesh( const TriangleMesh& mesh );
Eigen::VectorXf GetVerticesVectorFromMesh( const TriangleMesh& mesh );
std::vector< Eigen::Vector3f > GetNormalsFromMesh( const TriangleMesh& mesh );

// conversion between Eigen vectors and OpenMesh vectors
inline Eigen::Vector2f OpenMeshToEigen( const OpenMesh::Vec2f& v )
{
    Eigen::Vector2f vector_eigen;
    vector_eigen << v[0], v[1];
    return vector_eigen;
}

inline Eigen::Vector3f OpenMeshToEigen( const OpenMesh::Vec3f& v )
{
    Eigen::Vector3f vector_eigen;
    vector_eigen << v[0], v[1], v[2];
    return vector_eigen;
}

inline Eigen::Vector4f OpenMeshToEigen( const OpenMesh::Vec4f& v )
{
    Eigen::Vector4f vector_eigen;
    vector_eigen << v[0], v[1], v[2], v[3];
    return vector_eigen;
}

inline OpenMesh::Vec2f EigenToOpenMesh( const Eigen::Vector2f& v )
{
    return OpenMesh::Vec2f( v(0), v(1) );
}

inline OpenMesh::Vec3f EigenToOpenMesh( const Eigen::Vector3f& v )
{
    return OpenMesh::Vec3f( v(0), v(1), v(2) );
}

inline OpenMesh::Vec4f EigenToOpenMesh( const Eigen::Vector4f& v )
{
    return OpenMesh::Vec4f( v(0), v(1), v(2), v(3) );
}
    
#define block3(a) block<3,1>(3*(a), 0)
    
void OpenMeshToEigen(TriangleMesh& mesh,
                     Eigen::VectorXf& pts,
                     Eigen::VectorXf& clr,
                     Eigen::MatrixX3f& nml,
                     Eigen::MatrixX2f& uvs,
                     Eigen::MatrixX3i& tri);

void MoveMeshToCenterOfGravity( TriangleMesh& mesh, Eigen::Vector3f* center_of_gravity = NULL );

void GetMeshBoundingBox(
    const TriangleMesh& mesh,
    Eigen::Vector3f& bounding_box_min,
    Eigen::Vector3f& bounding_box_max );

void GetMeshBoundingSphere(
    const TriangleMesh& mesh,
    Eigen::Vector3f& center_of_gravity,
    float& radius );

void PaintSelectedVertices( TriangleMesh& mesh );
    
int GetConnectedComponents(TriangleMesh& mesh, std::vector<int>& comp_size);

void IntensityToRGBViaBCGYR(float _intensity, Eigen::Vector3f & _rgb);

} // namespace hfm



#endif // HFM_UTILITIES_OPENMESHUTILITIES_H
