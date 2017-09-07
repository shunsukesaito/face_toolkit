#version 330

uniform sampler2D  bg_texture;

in vec2 uv;

layout (location = 0) out vec4 frag_color;

void main() 
{
    vec2 flipped_uv = vec2(1.0 - uv.x, uv.y);

    frag_color = texture(bg_texture, flipped_uv);
}
