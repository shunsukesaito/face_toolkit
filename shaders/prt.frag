#version 330

uniform vec3 u_SHCoeffs[9];
uniform uint u_analytic;

uniform uint u_hasAlphaMap;
uniform uint u_hasNormalMap;
uniform uint u_hasAlbedoMap;

uniform sampler2D AlbedoMap;
uniform sampler2D NormalMap;
uniform sampler2D AlphaMap;

in VertexData {
    vec4 color;
    vec3 normal;
    vec2 texcoord;
    vec3 tangent;
    vec3 bitangent;
    vec3 prt0;
    vec3 prt1;
    vec3 prt2;
} VertexIn;

layout (location = 0) out vec4 frag_color;
layout (location = 1) out vec4 frag_n_coarse;
layout (location = 2) out vec4 frag_n_fine;
layout (location = 3) out vec4 frag_shading;

vec4 gammaCorrection(vec4 vec, float g)
{
    return vec4(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g), vec.w);
}

vec3 gammaCorrection(vec3 vec, float g)
{
    return vec3(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g));
}

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

// nC: coarse geometry normal, nH: fine normal from normal map
vec3 evaluateLightingModelHybrid(vec3 nC, vec3 nH, vec3 prt[3])
{
    float HC[9], HH[9];
    evaluateH(nC, HC);
    evaluateH(nH, HH);
    
    vec3 res = vec3(0.0);
    vec3 shadow = vec3(0.0);
    vec3 unshadow = vec3(0.0);
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            int id = i*3+j;
            res += HH[id]* u_SHCoeffs[id];
            shadow += prt[i][j] * u_SHCoeffs[id];
            unshadow += HC[id] * u_SHCoeffs[id];
        }
    }
    vec3 ratio = clamp(shadow/unshadow,0.0,1.0);
    res = ratio * res;

    return res;
}

vec3 evaluateLightingModelPRT(vec3 prt[3])
{
    vec3 res = vec3(0.0);
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            res += prt[i][j] * u_SHCoeffs[i*3+j];
        }
    }

    return res;
}

void main()
{
    vec3 prt[3];
    vec4 shading;
    vec2 uv = VertexIn.texcoord;
    uv[1] = 1.0 - uv[1];
    vec3 nC = normalize(VertexIn.normal);

    vec4 diff;
    if(u_hasAlbedoMap == uint(0))
        diff = VertexIn.color;
    else
        diff = gammaCorrection(texture(AlbedoMap, uv), 1.0/2.2);

    prt[0] = VertexIn.prt0;
    prt[1] = VertexIn.prt1;
    prt[2] = VertexIn.prt2;
    if(u_hasNormalMap == uint(0))
    {
        if(u_analytic == uint(0))
            shading = vec4(evaluateLightingModelPRT(prt), 1.0f);
        else
            shading = vec4(evaluateLightingModel(nC), 1.0f);
    }
    else
    {
        vec3 n_tan = normalize(texture(NormalMap, uv).rgb*2.0-vec3(1.0));

        mat3 TBN = mat3(normalize(VertexIn.tangent),normalize(VertexIn.bitangent),nC);
        vec3 nH = normalize(TBN * n_tan);

        if(u_analytic == uint(0))
            shading = vec4(evaluateLightingModelHybrid(nC,nH,prt),1.0f);
        else
            shading = vec4(evaluateLightingModel(nH), 1.0f);
        frag_n_fine = vec4(nH*0.5+vec3(0.5),1.0f);
    }

    frag_color = clamp(gammaCorrection(diff * shading, 2.2), 0.0, 1.0);
    frag_shading = clamp(gammaCorrection(shading,2.2), 0.0, 1.0);
    frag_n_coarse = vec4(nC*0.5+vec3(0.5),1.0f);
    if(u_hasAlphaMap != uint(0))
        frag_color.w = texture(AlphaMap, uv).r;
}