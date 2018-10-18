#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;
uniform uint u_uv_view;

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec3 v_normal;
layout (location = 2) in vec2 v_texcoord;

out vec4 normalWorld;
out vec4 normalModel;
out vec4 position;
out vec4 color;
out vec2 texcoord;

void main()
{
    normalWorld = u_modelview*vec4(v_normal, 0);
    normalModel = vec4(v_normal,0);
    position = vec4(v_position,1.0);
    texcoord = v_texcoord;
    color = vec4(1.0,1.0,1.0,1.0);

    if (u_uv_view != uint(0))
    {
        gl_Position = vec4(v_texcoord, 0.0, 1.0) - vec4(0.5, 0.5, 0.0, 0.0);
        gl_Position[0] *= 2;
        gl_Position[1] *= 2;
    }
    else
        gl_Position = u_mvp * position;
}
