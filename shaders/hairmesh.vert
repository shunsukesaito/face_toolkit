#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;

in vec3 v_position;
in vec3 v_normal;

out vec4 normal;
out vec4 pos;

void main()
{
    normal = u_modelview * vec4(v_normal, 0.0);
    pos = u_mvp * vec4(v_position, 1.0);
    gl_Position = pos;
}
