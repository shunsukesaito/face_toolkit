#version 330

in vec2 texcoord;
uniform sampler2D u_image;
layout (location = 0) out vec4 frag_color;

void main()
{
    frag_color = texture(u_image, texcoord);
}