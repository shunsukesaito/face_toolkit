#version 330

in vec2 texcoord;
in vec4 color;
in vec3 pos;

layout (location = 0) out vec4 frag_color;

uniform sampler2D u_image;

void main() 
{
    frag_color = texture(u_image, texcoord);//vec4(texcoord[0],texcoord[1],0.0,1.0);//
}
