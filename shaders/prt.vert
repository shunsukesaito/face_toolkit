#version 330

uniform mat4 u_mvp;

in vec3 v_position;
in vec4 v_color;
in vec3 v_normal;
in vec2 v_texcoord;
in vec3 v_tangent;
in vec3 v_bitangent;
in vec3 v_prt0;
in vec3 v_prt1;
in vec3 v_prt2;

out VertexData {
    vec4 color;
    vec3 normal;
    vec2 texcoord;
    vec3 tangent;
    vec3 bitangent;
    vec3 prt0;
    vec3 prt1;
    vec3 prt2;
} VertexOut;

void main()
{
    VertexOut.color = v_color;
    VertexOut.normal = v_normal;

    VertexOut.prt0 = v_prt0;
    VertexOut.prt1 = v_prt1;
    VertexOut.prt2 = v_prt2;

    VertexOut.texcoord = v_texcoord;
    VertexOut.tangent = v_tangent;
    VertexOut.bitangent = v_bitangent;

    gl_Position = u_mvp * vec4(v_position, 1.0);
}
