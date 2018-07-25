#version 330
#define pi 3.1415926535897932384626433832795

#include "lightstage.glsl"

// from rendering params
uniform mat4 u_world;

//uniform uint u_texture_mode; // 0: none, 1: uv space, 2: image space
uniform uint u_enable_mask;
uniform float u_mesomap_size;

in VertexData {
    vec4 normal_camera;
    vec4 Qd1;
    vec4 Qd2;
    vec2 texcoord;
} VertexIn;

uniform sampler2D u_sample_diff_albedo;
uniform sampler2D u_sample_spec_albedo;
uniform sampler2D u_sample_disp;

uniform sampler2D u_sample_mask;

layout(location = 0) out vec4 frag_diff_albedo;
layout(location = 1) out vec4 frag_spec_albedo;
layout(location = 2) out vec4 frag_normal;
layout(location = 3) out vec4 frag_texcoord;

void main(void)
{
    vec2 uv = VertexIn.texcoord;
    vec3 Nc = normalize(VertexIn.normal_camera.xyz);

    // use normal from displacement
    vec3 specNormal = calcTangentNormal(u_sample_disp, uv, Nc, VertexIn.Qd1, VertexIn.Qd2, u_mesomap_size);
    
    if (u_enable_mask != uint(0))
    {
        if (texture(u_sample_mask, uv)[0] < 0.5)
            discard;
    }
    frag_spec_albedo = texture(u_sample_spec_albedo, uv);
    frag_diff_albedo = texture(u_sample_diff_albedo, uv);
    frag_normal = vec4(specNormal,1.0);
    frag_texcoord = vec4(uv, 0.0, 1.0);
}
