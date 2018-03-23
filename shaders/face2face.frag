#version 330

// params
uniform uint u_enable_texture;
uniform uint u_enable_mask;
uniform uint u_enable_seg;
uniform float u_cull_offset;
uniform float u_alpha;

uniform vec3 u_SHCoeffs[9];

uniform sampler2D u_sample_texture;
uniform sampler2D u_sample_mask;
uniform sampler2D u_sample_seg;

in VertexData {
    vec4 color;
    vec4 normal;
    vec4 normalCamera;
    vec4 pos;
    vec2 proj_texcoord;
    vec2 texcoord;
    vec4 barycentric;
    vec4 indices;
} VertexIn;

layout(location = 0) out vec4 frag_pos;
layout(location = 1) out vec4 frag_normal;
layout(location = 2) out vec4 frag_color;
layout(location = 3) out vec4 frag_texcoord;
layout(location = 4) out vec4 frag_diffuse;
layout(location = 5) out vec4 frag_shading;
layout(location = 6) out vec4 frag_barycentric;
layout(location = 7) out vec4 frag_indices;

void evaluateH(vec3 n, out float H[9])
{
    float c1 = 0.429043, c2 = 0.511664,
        c3 = 0.743125, c4 = 0.886227, c5 = 0.247708;

    H[0] = c4;
    H[1] = 2.0 * c2 * n[1];
    H[2] = 2.0 * c2 * n[2];
    H[3] = 2.0 * c2 * n[0];
    H[4] = 2.0 * c1 * n[0] * n[1];
    H[5] = 2.0 * c1 * n[1] * n[2];
    H[6] = c3 * n[2] * n[2] - c5;
    H[7] = 2.0 * c1 * n[2] * n[0];
    H[8] = c1 * (n[0] * n[0] - n[1] * n[1]);
}

vec3 evaluateLightingModel(vec3 normal)
{
    float H[9];
    evaluateH(normal, H);
    vec3 res = vec3(0.0);
    for (int i = 0; i < 9; i++) {
        res += H[i] * u_SHCoeffs[i];
    }
    return res;
}

void main()
{
    vec4 diffuseColor = VertexIn.color;//material color
    vec3 normal = normalize(VertexIn.normal.xyz);
    vec3 normalCamera = normalize(VertexIn.normalCamera.xyz);
    vec2 texcoord = VertexIn.texcoord;
    
    frag_normal = vec4(normal, 1.0);
    frag_texcoord = vec4(texcoord, 0.0, 1.0);
    frag_barycentric = VertexIn.barycentric;
    frag_indices = VertexIn.indices;
    frag_pos = vec4(VertexIn.pos.xyz, 1.0);
    frag_shading = vec4(evaluateLightingModel(normal), 1.0f);

    if (u_enable_texture != uint(0))
        frag_color = texture(u_sample_texture, texcoord);
    else
        frag_color = vec4(clamp(VertexIn.color, vec4(0.0), vec4(1.0)));
    
    frag_diffuse = vec4(clamp(frag_color.xyz*frag_shading.xyz, vec3(0.0), vec3(1.0)), frag_color.a);
    
    if(dot(normalize(frag_pos.xyz),normalCamera.xyz) > u_cull_offset)
        discard;
   
    if (u_enable_mask != uint(0)){
        if (texture(u_sample_mask, VertexIn.texcoord)[0] < 0.5)
            discard;
    }

    if (u_enable_seg != uint(0)){
        if (texture(u_sample_seg, VertexIn.proj_texcoord)[0] > 0.5)
            discard;
    }

    frag_pos.a = u_alpha;
    frag_normal.a = u_alpha;
    frag_color.a = u_alpha;
    frag_texcoord.a = u_alpha;
    frag_diffuse.a = u_alpha;
    frag_shading.a = u_alpha;
    frag_barycentric.a = u_alpha;
    frag_indices.a = u_alpha;
}
