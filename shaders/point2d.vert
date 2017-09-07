#version 400

layout (location = 0) in vec2 v_p2d;
layout (location = 1) in vec4 v_color;

out vec4 color;

void main() 
{
    color = v_color;
    gl_Position = vec4(v_p2d.x * 2.0-1, 1 - v_p2d.y * 2.0, 0.0, 1.0);

}
