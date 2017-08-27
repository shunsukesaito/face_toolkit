#version 400

uniform sampler2D  background_texture;

in vec2 uv;

layout (location = 0) out vec4 frag_color;

void main() 
{
    vec2 flipped_uv = vec2(1.0 - uv.x, uv.y);

    vec4 color_rgb = texture(background_texture, flipped_uv);
    vec4 color_bgr = color_rgb;
    color_bgr.x = color_rgb.z;
    color_bgr.z = color_rgb.x;
    frag_color = color_bgr;
}