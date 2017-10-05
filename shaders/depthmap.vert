#version 330

uniform mat4 u_mvp;

layout (location = 0) in vec3 v_position;

void main()
{
    gl_Position = u_mvp * vec4(v_position, 1.0);
}
