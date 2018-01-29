#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec3 v_normal;

out vec4 normalWorld;
out vec4 normalModel;
out vec4 position;
out vec4 color;

void main()
{
    normalWorld = u_modelview*vec4(v_normal, 0);
    normalModel = vec4(v_normal,0);
    position = vec4(v_position,1.0);
    gl_Position = u_mvp * position;
    color = vec4(1.0,1.0,1.0,1.0);
}
