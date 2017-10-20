//
//  obj_loader.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 8/25/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef objloader_hpp
#define objloader_hpp

#include <iostream>
#include <fstream>

#include "tiny_obj_loader.h"

#include "EigenHelper.h"

static std::string GetBaseDir(const std::string &filepath) {
    if (filepath.find_last_of("/\\") != std::string::npos)
        return filepath.substr(0, filepath.find_last_of("/\\"));
    return "";
}

inline Eigen::Vector3f getCenter(Eigen::VectorXf& pts)
{
    Eigen::Map<Eigen::Matrix3Xf> p(pts.data(), 3, pts.size()/3);
    
    return p.rowwise().mean();
}

inline void writeObj(const std::string& filename,
                     Eigen::VectorXf& pts,
                     Eigen::MatrixX3f& nml,
                     Eigen::MatrixX2f& uvs,
                     Eigen::MatrixX3i& tri_pts,
                     Eigen::MatrixX3i& tri_uv)
{
    std::ofstream fout(filename);
    if(fout.is_open())
    {
        for(int i = 0; i < pts.size()/3; ++i)
        {
            fout << "v " << pts[i*3+0] << " " << pts[i*3+1] << " " << pts[i*3+2] << std::endl;
        }
        
        for(int i = 0; i < uvs.rows(); ++i)
        {
            fout << "vt " << uvs(i,0) << " " << uvs(i,1) << std::endl;
        }
        
        for(int i = 0; i < nml.rows(); ++i)
        {
            fout << "vn " << nml(i,0) << " " << nml(i,1) << " " << nml(i,2) << std::endl;
        }
        
        assert(tri_pts.size() == tri_uv.size());
        for(int i = 0; i < tri_pts.rows(); ++i)
        {
            fout << "f " << tri_pts(i,0)+1 << "/" << tri_uv(i,0)+1 << "/" << tri_pts(i,0)+1;
            fout << " " << tri_pts(i,1)+1 << "/" << tri_uv(i,1)+1 << "/" << tri_pts(i,1)+1;
            fout << " " << tri_pts(i,2)+1 << "/" << tri_uv(i,2)+1 << "/" << tri_pts(i,2)+1 << std::endl;
        }
        
        fout.close();
    }
    else{
        std::cout << "Warning: cannot write obj file. " << std::endl;
    }
}

inline void loadObjFile(const std::string& filename,
                        Eigen::VectorXf& pts,
                        Eigen::MatrixX3f& nml,
                        Eigen::MatrixX2f& uvs,
                        Eigen::MatrixX3i& tri_pts,
                        Eigen::MatrixX3i& tri_uv)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string base_dir = GetBaseDir(filename);
    if (base_dir.empty()) {
        base_dir = ".";
    }
#ifdef _WIN32
    base_dir += "\\";
#else
    base_dir += "/";
#endif
    
    std::string err;
    bool ret =
    tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str(), base_dir.c_str(), true);
    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        std::cerr << "Failed to load " << filename << std::endl;
        return;
    }
    
    printf("# of vertices  = %d\n", (int)(attrib.vertices.size()) / 3);
    printf("# of normals   = %d\n", (int)(attrib.normals.size()) / 3);
    printf("# of texcoords = %d\n", (int)(attrib.texcoords.size()) / 2);
    printf("# of materials = %d\n", (int)materials.size());
    printf("# of shapes    = %d\n", (int)shapes.size());
    
    pts.resize(attrib.vertices.size());
    for(int i = 0; i < attrib.vertices.size()/3; ++i)
    {
        pts[i*3+0] = attrib.vertices[i*3+0];
        pts[i*3+1] = attrib.vertices[i*3+1];
        pts[i*3+2] = attrib.vertices[i*3+2];
    }
    
    if(attrib.texcoords.size() > 0){
        uvs.resize(attrib.texcoords.size()/2,2);
        for(int i = 0; i < attrib.texcoords.size()/2; ++i)
        {
            uvs(i,0) = attrib.texcoords[i*2+0];
            uvs(i,1) = attrib.texcoords[i*2+1];
        }
    }
    
    tri_pts.resize(shapes[0].mesh.indices.size()/3, 3);
    tri_uv.resize(shapes[0].mesh.indices.size()/3, 3);
    for (size_t f = 0; f < shapes[0].mesh.indices.size() / 3; f++) {
        tinyobj::index_t idx0 = shapes[0].mesh.indices[3 * f + 0];
        tinyobj::index_t idx1 = shapes[0].mesh.indices[3 * f + 1];
        tinyobj::index_t idx2 = shapes[0].mesh.indices[3 * f + 2];
        
        tri_pts(f,0) = idx0.vertex_index;
        tri_pts(f,1) = idx1.vertex_index;
        tri_pts(f,2) = idx2.vertex_index;
        if(attrib.texcoords.size() > 0){
            tri_uv(f,0) = idx0.texcoord_index;
            tri_uv(f,1) = idx1.texcoord_index;
            tri_uv(f,2) = idx2.texcoord_index;
        }
        else{
            tri_uv(f,0) = tri_pts(f,0);
            tri_uv(f,1) = tri_pts(f,1);
            tri_uv(f,2) = tri_pts(f,2);
        }
    }
    
    calcNormal(nml, pts, tri_pts);
}


#endif /* objloader_hpp */
