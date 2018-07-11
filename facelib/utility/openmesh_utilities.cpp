#include "openmesh_utilities.h"

#include <iostream>
#include <vector>

namespace hfm
{

std::vector< Eigen::Vector3f > GetVerticesFromMesh( const TriangleMesh& mesh )
{
    std::vector< Eigen::Vector3f > vertices;

    for (TriangleMesh::ConstVertexIter v_it = mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
    {
        TriangleMesh::Point point = mesh.point( *v_it );
        vertices.push_back( OpenMeshToEigen(point) );
    }

    return vertices;
}
    
Eigen::VectorXf GetVerticesVectorFromMesh( const TriangleMesh& mesh )
{
    Eigen::VectorXf vertices(mesh.n_vertices()*3);
    
    for (TriangleMesh::ConstVertexIter v_it = mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {
        TriangleMesh::Point point = mesh.point( *v_it );
        vertices.block3(v_it->idx()) = OpenMeshToEigen(point);
    }
    
    return vertices;
}

std::vector< Eigen::Vector3f > GetNormalsFromMesh( const TriangleMesh& mesh )
{
    if (!mesh.has_vertex_normals())
    {
        throw std::runtime_error( std::string("no normals computed on the mesh") );
    }

    std::vector< Eigen::Vector3f > normals;

    for (TriangleMesh::ConstVertexIter v_it = mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
    {
        TriangleMesh::Normal normal = mesh.normal( *v_it );
        normals.push_back( OpenMeshToEigen(normal) );
    }

    return normals;
}

void OpenMeshToEigen( TriangleMesh& mesh,
                     Eigen::VectorXf& pts,
                     Eigen::VectorXf& clr,
                     Eigen::MatrixX3f& nml,
                     Eigen::MatrixX2f& uvs,
                     Eigen::MatrixX3i& tri)
{
    pts.resize(mesh.n_vertices()*3);
    if(mesh.has_vertex_normals())
        nml.resize(mesh.n_vertices(),3);
    if(mesh.has_vertex_colors())
        clr.resize(mesh.n_vertices()*3);
    if(mesh.has_vertex_texcoords2D())
        uvs.resize(mesh.n_vertices(),2);
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it ){
        pts.b3(v_it->idx()) = OpenMeshToEigen(mesh.point(*v_it));
        if(mesh.has_vertex_normals())
            nml.row(v_it->idx()) = OpenMeshToEigen(mesh.normal(*v_it)).transpose();
        if(mesh.has_vertex_colors())
            clr.b3(v_it->idx()) = OpenMeshToEigen(mesh.color(*v_it)).segment(0, 3);
        if(mesh.has_vertex_texcoords2D())
            uvs.row(v_it->idx()) = OpenMeshToEigen(mesh.texcoord2D(*v_it)).transpose();
    }
    
    tri.resize(mesh.n_faces(),3);
    int index = 0;
    for(auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it, index++){
        auto fv_it = mesh.fv_begin(*f_it);
        tri(index,0) = fv_it->idx(); fv_it++;
        tri(index,1) = fv_it->idx(); fv_it++;
        tri(index,2) = fv_it->idx();
    }
}
    
void MoveMeshToCenterOfGravity( TriangleMesh& mesh, Eigen::Vector3f* center_of_gravity )
{
    OpenMesh::Vec3f gravity(0, 0, 0);
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it )
        gravity += mesh.point(*v_it);

    if( mesh.n_vertices() > 0 ) 
        gravity /= float(mesh.n_vertices());

    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it )
        mesh.set_point(*v_it, mesh.point(*v_it)-gravity);

    if (center_of_gravity)
        *center_of_gravity = OpenMeshToEigen(gravity);
}

void GetMeshBoundingBox( 
    const TriangleMesh& mesh, 
    Eigen::Vector3f& bounding_box_min, 
    Eigen::Vector3f& bounding_box_max )
{
    TriangleMesh::ConstVertexIter vIt(mesh.vertices_begin());
    TriangleMesh::ConstVertexIter vEnd(mesh.vertices_end());      
        
    OpenMesh::Vec3f bbmin, bbmax;
    
    bbmin = bbmax = OpenMesh::vector_cast<OpenMesh::Vec3f>(mesh.point(*vIt));
    
    for (size_t count=0; vIt!=vEnd; ++vIt, ++count)
    {
      bbmin.minimize( OpenMesh::vector_cast<OpenMesh::Vec3f>(mesh.point(*vIt)));
      bbmax.maximize( OpenMesh::vector_cast<OpenMesh::Vec3f>(mesh.point(*vIt)));
    }

    bounding_box_min = OpenMeshToEigen( bbmin );
    bounding_box_max = OpenMeshToEigen( bbmax );
}

void GetMeshBoundingSphere( 
    const TriangleMesh& mesh, 
    Eigen::Vector3f& center_of_gravity,
    float& radius )
{
    OpenMesh::Vec3f center_of_gravity_openmesh(0.f, 0.f, 0.f);
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it )
    {
        center_of_gravity_openmesh += mesh.point( *v_it );
    }
    if ( mesh.n_vertices() > 0 )
        center_of_gravity_openmesh /= float(mesh.n_vertices());

    float max_distance = 0;
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it )
    {
        float distance = (center_of_gravity_openmesh - mesh.point( *v_it )).length();
        if (distance > max_distance) max_distance = distance;
    }

    center_of_gravity = OpenMeshToEigen( center_of_gravity_openmesh );
    radius = max_distance;
}
    
int GetConnectedComponents(TriangleMesh& mesh, std::vector<int>& comp_size)
{
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it )
        mesh.data(*v_it).set_selected(false);
    
    int cur_comp = 0;
    int max_comp = 0;
    int max_comp_size = -1;
    comp_size.clear();
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it )
    {
        if(mesh.data(*v_it).is_selected())
            continue;
        std::vector<int> v_idxs;
        int cur_size = 0;
        mesh.data(*v_it).set_comp_id(cur_comp);
        mesh.data(*v_it).set_selected(true);
        v_idxs.push_back(v_it->idx());
        //std::cout << "starting from " << v_it->idx() << std::endl;
        cur_size++;
        while(v_idxs.size() != 0){
            int vid = v_idxs[0];
            //std::cout << "circulating around " << vid << std::endl;
            // circulate around the current vertex
            for (TriangleMesh::VertexVertexIter vv_it=mesh.vv_iter(mesh.vertex_handle(vid)); vv_it.is_valid(); ++vv_it)
            {
                //std::cout << "checking " << vv_it->idx() << std::endl;
                if(mesh.data(*vv_it).is_selected())
                    continue;
                
                mesh.data(*vv_it).set_comp_id(cur_comp);
                mesh.data(*vv_it).set_selected(true);
                v_idxs.push_back(vv_it->idx());
                cur_size++;
                //std::cout << "added " << vv_it->idx() << std::endl;
            }
            v_idxs.erase(v_idxs.begin());
        }
        comp_size.push_back(cur_size);
        if(max_comp_size < cur_size){
            max_comp = cur_comp;
            max_comp_size = cur_size;
        }
        cur_comp++;
    }
    
    // check everything is selected
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it )
        assert(mesh.data(*v_it).is_selected(false));
    
    return max_comp;
}

void PaintSelectedVertices( TriangleMesh& mesh )
{
    for (auto vertex = mesh.vertices_begin(); vertex != mesh.vertices_end(); ++vertex )
    {
        if ( mesh.data(*vertex).is_selected() )
            mesh.set_color( *vertex, PreDefinedColors::epfl_blue );
        else
            mesh.set_color( *vertex, PreDefinedColors::default_color );
    }
}
    
void IntensityToRGBViaBCGYR(float _intensity, Eigen::Vector3f & _rgb)
{
    float v0, v1, v2, v3, v4;

    v0 = 0.0/4.0;
    v1 = 1.0/4.0;
    v2 = 2.0/4.0;
    v3 = 3.0/4.0;
    v4 = 4.0/4.0;
    
    _rgb[0]=1.0;
    _rgb[1]=1.0;
    _rgb[2]=1.0;
    
    float t=0.0;
    
    if(_intensity<v0)
    {
        // blue
        _rgb[0]=0.0;
        _rgb[1]=0.0;
        _rgb[2]=1.0;
    }
    else if(_intensity>v4)
    {
        // red
        _rgb[0]=1.0;
        _rgb[1]=0.0;
        _rgb[2]=0.0;
    }
    else if (_intensity<=v2)
    {
        // [v0,v1]
        if(_intensity <= v1)
        {
            t = (_intensity - v0) / (v1-v0);
            _rgb[0]=0.0;
            _rgb[1]=t;
            _rgb[2]=1.0;
        }
        // ]v1,v2]
        else
        {
            t = (_intensity - v1) / (v2-v1);
            _rgb[0]=0.0;
            _rgb[1]=1.0;
            _rgb[2]=1.0-t;
        }
    }
    else
    {
        // ]v2,v3]
        if(_intensity<=v3)
        {
            t = (_intensity - v2) / (v3-v2);
            _rgb[0]=t;
            _rgb[1]=1.0;
            _rgb[2]=0.0;
        }
        // ]v3,v4]
        else
        {
            t = (_intensity - v3) / (v4-v3);
            _rgb[0]=1.0;
            _rgb[1]=1.0-t;
            _rgb[2]=0.0;
        }
    }
}

} // namespace hfm
