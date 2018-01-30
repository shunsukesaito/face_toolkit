//
//  subdiv.cpp
//  face_toolkit
//
//  Created by Shunsuke Saito on 1/27/18.
//  Copyright © 2018 Shunsuke Saito. All rights reserved.
//

#include "subdiv.h"

using namespace OpenSubdiv;

void performSubdiv(const Eigen::VectorXf& pts_src,
                   const Eigen::MatrixX2f& uvs_src,
                   const Eigen::MatrixX3i& triuv_src,
                   const Eigen::MatrixX3i& tripts_src,
                   Eigen::VectorXf& pts_dst,
                   Eigen::MatrixX2f& uvs_dst,
                   Eigen::MatrixX3i& triuv_dst,
                   Eigen::MatrixX3i& tripts_dst)
{
    assert(tripts_src.size() == triuv_src.size());

    const int nverts = pts_src.size()/3;
    const int nuvs = uvs_src.size()/2;
    const int nfaces = tripts_src.size()/3;
    std::vector<int> tripi(tripts_src.size()), triui(triuv_src.size());
    for(int i = 0; i < tripts_src.rows(); ++i)
    {
        tripi[i*3+0] = tripts_src(i,0);
        tripi[i*3+1] = tripts_src(i,1);
        tripi[i*3+2] = tripts_src(i,2);

        triui[i*3+0] = triuv_src(i,0);
        triui[i*3+1] = triuv_src(i,1);
        triui[i*3+2] = triuv_src(i,2);
    }
    const int* trip_int = &tripi[0];
    const int* triu_int = &triui[0];
    std::vector<int> vperfacei(nfaces, 3);
    const int* vperface = &vperfacei[0];

    int maxlevel = 4;

    typedef Far::TopologyDescriptor Descriptor;

    Sdc::SchemeType type = OpenSubdiv::Sdc::SCHEME_CATMARK;

    Sdc::Options options;
    options.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_ONLY);
    options.SetFVarLinearInterpolation(Sdc::Options::FVAR_LINEAR_NONE);
    options.SetTriangleSubdivision(Sdc::Options::TRI_SUB_CATMARK);

    // Populate a topology descriptor with our raw data
    Descriptor desc;
    desc.numVertices  = nverts;
    desc.numFaces     = nfaces;
    desc.numVertsPerFace = vperface;
    desc.vertIndicesPerFace  = trip_int;

    int channelUV = 0;

    // Create a face-varying channel descriptor
    Descriptor::FVarChannel channels[1];
    channels[channelUV].numValues = nuvs;
    channels[channelUV].valueIndices = triu_int;

    // Add the channel topology to the main descriptor
    desc.numFVarChannels = 1;
    desc.fvarChannels = channels;

    // Instantiate a FarTopologyRefiner from the descriptor
    Far::TopologyRefiner * refiner =
    Far::TopologyRefinerFactory<Descriptor>::Create(desc,
                                                    Far::TopologyRefinerFactory<Descriptor>::Options(type, options));

    // Uniformly refine the topology up to 'maxlevel'
    // note: fullTopologyInLastLevel must be true to work with face-varying data
    {
        Far::TopologyRefiner::UniformOptions refineOptions(maxlevel);
        refineOptions.fullTopologyInLastLevel = true;
        refiner->RefineUniform(refineOptions);
    }

    // Allocate and initialize the 'vertex' primvar data (see tutorial 2 for
    // more details).
    std::vector<Vertex> vbuffer(refiner->GetNumVerticesTotal());
    Vertex * verts = &vbuffer[0];

    for (int i=0; i<nverts; ++i) {
        verts[i].SetPosition(pts_src[i*3+0], pts_src[i*3+1], pts_src[i*3+2]);
    }

    // Allocate and initialize the first channel of 'face-varying' primvar data (UVs)
    std::vector<FVarVertexUV> fvBufferUV(refiner->GetNumFVarValuesTotal(channelUV));
    FVarVertexUV * fvVertsUV = &fvBufferUV[0];
    for (int i=0; i< nuvs; ++i) {
        fvVertsUV[i].u = uvs_src(i,0);
        fvVertsUV[i].v = uvs_src(i,1);
    }

    // Interpolate both vertex and face-varying primvar data
    Far::PrimvarRefiner primvarRefiner(*refiner);

    Vertex *     srcVert = verts;
    FVarVertexUV * srcFVarUV = fvVertsUV;

    for (int level = 1; level <= maxlevel; ++level) {
        Vertex *     dstVert = srcVert + refiner->GetLevel(level-1).GetNumVertices();
        FVarVertexUV * dstFVarUV = srcFVarUV + refiner->GetLevel(level-1).GetNumFVarValues(channelUV);

        primvarRefiner.Interpolate(level, srcVert, dstVert);
        primvarRefiner.InterpolateFaceVarying(level, srcFVarUV, dstFVarUV, channelUV);

        srcVert = dstVert;
        srcFVarUV = dstFVarUV;
    }

    { // Output OBJ of the highest level refined -----------

        Far::TopologyLevel const & refLastLevel = refiner->GetLevel(maxlevel);

        int nverts = refLastLevel.GetNumVertices();
        int nuvs   = refLastLevel.GetNumFVarValues(channelUV);
        int nfaces = refLastLevel.GetNumFaces();

        pts_dst.resize(nverts*3);
        uvs_dst.resize(nuvs,2);
        triuv_dst.resize(nfaces*2, 3);
        tripts_dst.resize(nfaces*2, 3);

        // Print vertex positions
        int firstOfLastVerts = refiner->GetNumVerticesTotal() - nverts;

        for (int vert = 0; vert < nverts; ++vert) {
            float const * pos = verts[firstOfLastVerts + vert].GetPosition();
            pts_dst[vert*3+0] = pos[0];
            pts_dst[vert*3+1] = pos[1];
            pts_dst[vert*3+2] = pos[2];
            //printf("v %f %f %f\n", pos[0], pos[1], pos[2]);
        }

        // Print uvs
        int firstOfLastUvs = refiner->GetNumFVarValuesTotal(channelUV) - nuvs;

        for (int fvvert = 0; fvvert < nuvs; ++fvvert) {
            FVarVertexUV const & uv = fvVertsUV[firstOfLastUvs + fvvert];
            uvs_dst(fvvert,0) = uv.u;
            uvs_dst(fvvert,1) = uv.v;
            //printf("vt %f %f\n", uv.u, uv.v);
        }

        // Print faces
        for (int face = 0; face < nfaces; ++face) {

            Far::ConstIndexArray fverts = refLastLevel.GetFaceVertices(face);
            Far::ConstIndexArray fuvs   = refLastLevel.GetFaceFVarValues(face, channelUV);
            
            tripts_dst(face*2+0,0) = fverts[0];
            tripts_dst(face*2+0,1) = fverts[1];
            tripts_dst(face*2+0,2) = fverts[2];
            
            tripts_dst(face*2+1,0) = fverts[0];
            tripts_dst(face*2+1,1) = fverts[2];
            tripts_dst(face*2+1,2) = fverts[3];

            triuv_dst(face*2+0,0) = fuvs[0];
            triuv_dst(face*2+0,1) = fuvs[1];
            triuv_dst(face*2+0,2) = fuvs[2];
            
            triuv_dst(face*2+1,0) = fuvs[0];
            triuv_dst(face*2+1,1) = fuvs[2];
            triuv_dst(face*2+1,2) = fuvs[3];

        }
    }
}
