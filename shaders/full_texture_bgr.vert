#version 330

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec2 v_texcoord;

out vec2 uv;

void main() 
{
   gl_Position = vec4(v_position, 1.0);
   uv = v_texcoord;
}
