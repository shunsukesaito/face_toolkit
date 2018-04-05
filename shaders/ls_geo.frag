#version 330
#define pi 3.1415926535897932384626433832795

#include "lightstage.glsl"

// from rendering params
uniform mat4 u_world;

uniform uint u_enable_mask;
uniform uint u_cull_occlusion;
uniform float u_cull_offset;
uniform float u_mesomap_size;

uniform vec3 u_light_pos1;
uniform vec3 u_light_pos2;
uniform vec3 u_light_pos3;

// from camera
uniform vec3 u_camera_pos;

in VertexData {
    vec4 normal_world;
    vec4 normal_camera;
    vec4 pos_shadow_mvp;
    vec4 pos_world;
    vec4 pos_camera;
    vec4 Qd1;
    vec4 Qd2;
    vec2 texcoord;
} VertexIn;

uniform sampler2D u_sample_disp;

uniform sampler2D u_sample_mask;
uniform sampler2D u_sample_depth;

const vec3 SSSColor = vec3(0.75, 0.6, 0.5);

layout(location = 0) out vec4 frag_color;

void main(void)
{
    vec2 uv = VertexIn.texcoord;
    vec3 posw = VertexIn.pos_world.xyz;
    vec3 Nw = normalize(VertexIn.normal_world.xyz);
    vec3 Nc = normalize(VertexIn.normal_camera.xyz);
    vec3 vieww = normalize(u_camera_pos-VertexIn.pos_world.xyz);
    vec3 viewc = normalize(VertexIn.pos_camera.xyz);

    // use normal from displacement
    vec3 specNormal = calcTangentNormal(u_sample_disp, uv, Nw, VertexIn.Qd1, VertexIn.Qd2, u_mesomap_size);
    float F = fresnelGraham(0.05, dot(specNormal, vieww), 2.0);

    if (u_enable_mask != uint(0))
    {
        if (texture(u_sample_mask, uv)[0] < 0.5)
            discard;
    }
    
    if(u_cull_occlusion != uint(0))
    {
        if(dot(viewc,Nc) > u_cull_offset)
            discard;
        
        vec3 ShadowMapTexCoord = VertexIn.pos_shadow_mvp.xyz / VertexIn.pos_shadow_mvp.w;
        float bias = 0.005*tan(acos(clamp(dot(Nc, viewc),0.0,1.0)));
        bias = clamp(bias, 0,0.01);
        if(texture(u_sample_depth, ShadowMapTexCoord.xy).r <= ShadowMapTexCoord.z-bias)
            discard;
    }

    if(dot(vieww,specNormal) < 0.0)
        specNormal = -specNormal;

    /***************************
	* CookñTorrance: http://www.codinglabs.net/article_physically_based_rendering_cook_torrance.aspx
	****************************/
    vec3 N = specNormal;
    vec3 V = vieww;
	float IOR = 2.4; // ALshader use IOR = 1.38, solid angle uses 1.44
	float microBaseRoughness = 0.15;
	float mesoBaseRoughness = 0.25;
	float F0 = (1.0 - IOR) / (1.0 + IOR); 
	F0 = F0 * F0;	
	vec4 specular = vec4(0,0,0,0);
	vec4 diffuse = vec4(0,0,0,0);
    vec3 lights[3];
    lights[0] = u_light_pos1;
    lights[1] = u_light_pos2;
    lights[2] = u_light_pos3;
	for (int i = 0; i < 3; i++)
	{
		vec3 lightpos = lights[i] + vec3(0,0,0);
        vec3 L = normalize(lights[i] - posw);
		vec3 H = normalize(L + V);
		vec3 R = normalize(-reflect(L,N));

		float cosTheta = clamp(dot(V, H),0.0,1.0);
		float F = Fresnel_Schlick(cosTheta, F0, 5.0);
		float ks = F; // the amount of light that is specularly reflected
		float kd = 1.0 - ks; // the amount of diffuse light that is reflected

		float D = GGX_Distribution(N, H, mesoBaseRoughness);	    
		float G = GGX_PartialGeometryTerm(L, N, H, mesoBaseRoughness) * GGX_PartialGeometryTerm(V, N, H, mesoBaseRoughness);
        
		// Calculate the Cook-Torrance denominator
		float NoV = clamp(dot(N, V),0.0,1.0);
		float NoH = clamp(dot(N, H),0.0,1.0);
		float denominator = clamp(4*(NoV * NoH) + 0.05, 1.0, 1.0); // [MAGIC] this 0.05 is magic
		float brdf_meso = D*F*G / denominator;

		D = GGX_Distribution(N, H, microBaseRoughness);	    
		G = GGX_PartialGeometryTerm(L, N, H, microBaseRoughness) * GGX_PartialGeometryTerm(V, N, H, microBaseRoughness);
		float brdf_micro = D*F*G / denominator;

		float specular_reflection = 0.7*brdf_meso + 0.3*brdf_micro;
		diffuse = diffuse + (kd * vec4(0.06,0.3,0.31,1.0) * max(dot(N,L),0.0)); // use fixed diffuse color
		specular = specular + (ks * specular_reflection);	
	}
	frag_color = diffuse + specular;
    frag_color.a = 1.0;
    frag_color = gammaCorrection(frag_color, 2.2);
}
