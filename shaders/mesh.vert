#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec3 v_normal;

out vec3 normal;
out vec4 color;

void main()
{
    normal = (u_modelview*vec4(v_normal, 0)).xyz;
    gl_Position = u_mvp * vec4(v_position, 1.0);
    color = vec4(1.0,1.0,1.0,1.0);
}
