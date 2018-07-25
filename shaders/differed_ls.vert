#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;

in vec3 v_position;
in vec3 v_normal;
in vec2 v_texcoord;
in vec3 v_tangent;
in vec3 v_bitangent;

out VertexData {
    vec4 normal_camera;
    vec4 Qd1;
    vec4 Qd2;
    vec2 texcoord;
} VertexOut;

void main() 
{
    VertexOut.normal_camera = u_modelview * vec4(v_normal, 0.0);
    
    gl_Position = u_mvp * vec4(v_position, 1.0);
        
    VertexOut.Qd1 = u_modelview * vec4(v_tangent, 0.0);
    VertexOut.Qd2 = u_modelview * vec4(v_bitangent, 0.0);
    
    VertexOut.texcoord = v_texcoord;
}
