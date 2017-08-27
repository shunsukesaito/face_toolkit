#version 400

in vec3 position;
in vec3 normal;
in vec4 color;

layout (location = 0) out vec4 frag_color;

void main() 
{
   frag_color = vec4(dot(normal, vec3(0, 0, -1)) * color.rgb, color.a);
}