#version 330

// params
uniform uint g_proj_tex;
uniform uint g_tex_enable;
uniform uint g_inv_diffuse;
uniform uint g_normal_type; // 0: model space, 1: camera space, 
uniform uint g_enable_mask;
uniform uint g_enable_seg;
uniform float g_cull_offset;

uniform vec3 g_SHCoeffs[9];

uniform sampler2D u_tex1; 
uniform sampler2D u_tex2;
uniform sampler2D u_tex3;

in VertexData {
    vec4 color;
    vec4 normal;
    vec4 normalCamera;
    vec4 light;
    vec4 pos;
    vec2 proj_texcoord;
    vec2 texcoord;
} VertexIn;

layout(location = 0) out vec4 frag_color;

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
        res += H[i] * g_SHCoeffs[i];
    }
    return res;
}

void main()
{
    vec4 diffuseColor = VertexIn.color;//material color
    
    vec3 normal = normalize(VertexIn.normal.xyz);
    vec3 normalCamera = normalize(VertexIn.normalCamera.xyz);
    
    vec2 texcoord;
    if(g_proj_tex != uint(0)){
        texcoord = VertexIn.proj_texcoord;
    }
    else{
        texcoord = VertexIn.texcoord;
    }
    shading = vec4(, 1.0f);
    
    if (g_tex_enable != uint(0))
        frag_color = texture(u_tex1, texcoord);
    else
        frag_color = vec4(clamp(VertexIn.color, vec4(0.0), vec4(1.0)));
    
    vec3 shading = evaluateLightingModel(normal);

    if (g_inv_diffuse != uint(0))
    {
        vec3 inv_dif = clamp(VertexIn.color.xyz, vec3(0.0), vec3(1.0)) + (frag_color.xyz - clamp(VertexIn.color.xyz*shading, vec3(0.0), vec3(1.0)));
        if (shading[0] < 0.1 || shading[1] < 0.1 || shading[2] < 0.1) discard;
        else frag_color = vec4(inv_dif, frag_color.a);
        if (frag_color[0] == 1.0 || frag_color[1] == 1.0 || frag_color[2] == 1.0) discard;
    }
    else
    {
        frag_color = vec4(clamp(frag_color.xyz*shading, vec3(0.0), vec3(1.0)), frag_color.a);
    }
    
    if(g_cull_offset != 0.0){
        if(normalCamera[2] > g_cull_offset){
            discard;
        }
    }

    if (g_enable_mask != uint(0))
    {
        if (texture(u_tex2, VertexIn.texcoord)[0] < 0.5)
            discard;
    }

    if (g_enable_seg != uint(0)){
        if (texture(u_tex3, VertexIn.proj_texcoord)[0] > 0.5)
            discard;
    }
}