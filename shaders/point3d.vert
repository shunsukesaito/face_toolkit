#version 400

uniform mat4 mvp;

layout (location = 0) in vec3 vp;
layout (location = 1) in vec3 n;
layout (location = 2) in vec4 vertex_color;

out vec3 position;
out vec3 normal;
out vec4 color;

void main() 
{
   position = vp;
   normal = n;
   color = vertex_color;
   gl_Position = mvp * vec4(vp, 1.0);
}