#version 400

layout (location = 0) in vec3 vp;
layout (location = 1) in vec2 vertex_uv;

out vec2 uv;

void main() 
{
   gl_Position = vec4(vp, 1.0);
   uv = vertex_uv;
}