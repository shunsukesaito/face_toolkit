#version 330

uniform uint u_iscvmat;
uniform sampler2D  u_texture;

in vec2 uv;

layout (location = 0) out vec4 frag_color;

void main() 
{
    frag_color = texture(u_texture, uv.xy);
}
