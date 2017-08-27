#version 400

uniform mat4 mvp;

layout (location = 0) in vec2 vp;
layout (location = 1) in vec3 vertex_color;

out vec4 color;

void main() 
{
   color = vec4(vertex_color, 1.0);
   vec4 pos = mvp * vec4(vp.x, -vp.y, 0.0, 1.0);
   pos.x = pos.x * 2.0 - 1.0;
   pos.y = pos.y * 2.0 + 1.0;
   gl_Position = pos;
}
