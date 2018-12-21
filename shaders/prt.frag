#version 330

uniform vec3 u_SHCoeffs[9];
uniform uint u_analytic;

in VertexData {
    vec4 color;
    vec3 normal;
    vec3 prt0;
    vec3 prt1;
    vec3 prt2;
} VertexIn;

layout (location = 0) out vec4 frag_color;

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
    vec3 normal = normalize(VertexIn.normal);

    prt[0] = VertexIn.prt0;
    prt[1] = VertexIn.prt1;
    prt[2] = VertexIn.prt2;
    if(u_analytic == uint(0))
        shading = vec4(evaluateLightingModelPRT(prt), 1.0f);
    else
        shading = vec4(evaluateLightingModel(normal), 1.0f);

    frag_color = shading;
}
