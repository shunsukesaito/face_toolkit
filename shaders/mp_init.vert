#version 330

uniform mat4 u_mvp;

in vec3 v_position;
in vec2 v_texcoord;

out vec2 texcoord;

void main()
{
    vec4 posWorld = u_mvp * vec4(v_position, 1.0);

    texcoord[0] = 0.5*(posWorld[0] / posWorld[3]) + 0.5;
    texcoord[1] = -0.5*(posWorld[1] / posWorld[3]) + 0.5;

    gl_Position = vec4(v_texcoord, 0.0, 1.0) - vec4(0.5, 0.5, 0.0, 0.0);
    gl_Position[0] *= 2;
    gl_Position[1] *= 2;
}
