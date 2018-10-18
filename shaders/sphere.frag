#version 330

in vec4 normalWorld;
in vec4 normalModel;
in vec4 position;
in vec4 color;
in vec2 texcoord;

uniform sampler2D u_texture;

layout (location = 0) out vec4 frag_color;

void main() 
{
    frag_color = texture(u_texture, texcoord);
}
