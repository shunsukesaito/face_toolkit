#version 330

in vec4 normal_world;
in vec2 texcoord;

layout (location = 0) out vec4 frag_color;

uniform sampler2D u_texture;

void main() 
{
    frag_color = texture(u_texture, texcoord);
}
