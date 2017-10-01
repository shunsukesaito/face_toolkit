#version 400

uniform mat4 u_mvp;
uniform mat4 u_modelview;
uniform mat4 u_world;
uniform mat4 u_shadow_mvp;

in vec3 v_position;
in vec4 v_color;
in vec3 v_normal;
in vec2 v_texcoord;

out VertexData {
    vec4 color;
    vec4 normal_world;
    vec4 normal_camera;
    vec4 pos_world;
    vec4 pos_camera;
    vec2 proj_texcoord;
    vec2 texcoord;
} VertexOut;

void main()
{
    VertexOut.color = v_color;
    
    VertexOut.normal_world = u_world * vec4(v_normal, 1.0);
    VertexOut.normal_camera = u_modelview * vec4(v_normal, 0.0);
    
    vec4 posWorld = u_mvp * vec4(v_position, 1.0);
    
    VertexOut.pos_camera = u_modelview * vec4(v_position, 1.0);
    VertexOut.pos_world = u_world * vec4(v_position, 1.0);
    
    VertexOut.proj_texcoord[0] = 0.5*(posWorld[0] / posWorld[3]) + 0.5;
    VertexOut.proj_texcoord[1] = 1.0 - 0.5*(posWorld[1] / posWorld[3]) + 0.5;
    
    VertexOut.texcoord = v_texcoord;
    
    gl_Position = posWorld;
}
