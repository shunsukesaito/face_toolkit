#version 330

uniform uint u_iscvmat;
uniform sampler2D  u_texture;

in vec2 uv;

layout (location = 0) out vec4 frag_color;

void main() 
{
    vec2 f_uv = vec2(uv.x, uv.y);
    if(u_iscvmat != uint(0))
        f_uv.y = 1.0 - f_uv.y;
    frag_color = texture(u_texture, f_uv);
}
