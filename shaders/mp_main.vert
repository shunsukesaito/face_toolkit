#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;

in vec3 v_position;
in vec3 v_normal;
in vec2 v_texcoord;

out vec4 normal_world;
out vec2 texcoord;

void main()
{
    normal_world = u_modelview*vec4(v_normal, 0);
    gl_Position = u_mvp * vec4(v_position,1.0);
    texcoord = v_texcoord;
}
