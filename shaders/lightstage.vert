#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;
uniform mat4 u_world;
uniform mat4 u_shadow_mvp;

uniform uint u_uv_view;

in vec3 v_position;
in vec3 v_normal;
in vec2 v_texcoord;
in vec3 v_tangent;
in vec3 v_bitangent;

out VertexData {
    vec4 normal_world;
    vec4 normal_camera;
    vec4 pos_shadow_mvp;
    vec4 pos_world;
    vec4 pos_camera;
    vec4 Qd1;
    vec4 Qd2;
    vec2 texcoord;
} VertexOut;

void main() 
{
    VertexOut.normal_world = u_world * vec4(v_normal, 0.0);
    VertexOut.normal_camera = u_modelview * vec4(v_normal, 0.0);
    
    vec4 pos_mvp = u_mvp * vec4(v_position, 1.0);
    
    VertexOut.pos_shadow_mvp = u_shadow_mvp * vec4(v_position, 1.0);
    VertexOut.pos_camera = u_modelview * vec4(v_position, 1.0);
    VertexOut.pos_world = u_world * vec4(v_position, 1.0);
    
    VertexOut.Qd1 = u_world * vec4(v_tangent, 0.0);
    VertexOut.Qd2 = u_world * vec4(v_bitangent, 0.0);
    
    VertexOut.texcoord = v_texcoord;
    
    if (u_uv_view != uint(0))
    {
        gl_Position = vec4(v_texcoord, 0.0, 1.0) - vec4(0.5, 0.5, 0.0, 0.0);
        gl_Position[0] *= 2;
        gl_Position[1] *= 2;
    }
    else
        gl_Position = pos_mvp;
}
