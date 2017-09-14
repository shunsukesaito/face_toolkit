#version 330

uniform uint u_tex_mode; // 0: openGL, 1: openCVMat (flip:0) , 2: videoCapture (flip:-1)
uniform sampler2D  u_texture;

in vec2 uv;

layout (location = 0) out vec4 frag_color;

void main() 
{
    vec2 f_uv;
    if(u_tex_mode == uint(1))
        f_uv = vec2(uv.x, 1.0 - uv.y);
    else if(u_tex_mode == uint(2))
        f_uv = vec2(1.0 - uv.x, 1.0 - uv.y);
    else
        f_uv = vec2(uv.x, uv.y);

    frag_color = texture(u_texture, f_uv);
}
