#version 330
 
layout(triangles) in;
layout (triangle_strip, max_vertices=3) out;
 
in VertexData {
    vec4 color;
    vec4 normal;
    vec4 normalCamera;
    vec4 pos;
    vec2 proj_texcoord;
    vec2 texcoord;
    uint index;
} VertexIn[3];
 
out VertexData {
    vec4 color;
    vec4 normal;
    vec4 normalCamera;
    vec4 pos;
    vec2 proj_texcoord;
    vec2 texcoord;
    vec4 barycentric;
    vec4 indices;
} VertexOut;
 
void main()
{
    vec4 indices = vec4(VertexIn[0].index, VertexIn[1].index, VertexIn[2].index, 1.0f);

    for(int i = 0; i < gl_in.length(); i++)
    {
         // copy attributes
        gl_Position = gl_in[i].gl_Position;

        VertexOut.color = VertexIn[i].color;
        VertexOut.normal = VertexIn[i].normal;
        VertexOut.normalCamera = VertexIn[i].normalCamera;
        VertexOut.pos = VertexIn[i].pos;
        VertexOut.proj_texcoord = VertexIn[i].proj_texcoord;
        VertexOut.texcoord = VertexIn[i].texcoord;

        VertexOut.indices = indices;
        VertexOut.barycentric = vec4(0.0f, 0.0f, 0.0f, 1.0f);
        VertexOut.barycentric[i] = 1.0f;


        // done with the vertex
        EmitVertex();
    }
}
