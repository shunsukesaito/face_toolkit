#version 330
#define pi 3.1415926535897932384626433832795

#include "lightstage.glsl"

// from rendering params
uniform mat4 u_world;

//uniform uint u_texture_mode; // 0: none, 1: uv space, 2: image space
uniform uint u_enable_mask;
uniform uint u_cull_occlusion;
uniform uint u_use_pointlight;
uniform float u_cull_offset;
uniform float u_light_rot;
uniform vec3 u_light_pos;
uniform float u_specscale;
uniform float u_diffscale;
uniform float u_mesomap_size;

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

uniform sampler2D u_sample_diff_albedo;
uniform sampler2D u_sample_spec_albedo;
uniform sampler2D u_sample_diff_normal;
//uniform sampler2D u_sample_spec_normal;
uniform sampler2D u_sample_disp;
uniform sampler2D u_sample_diff_env;
uniform sampler2D u_sample_spec_env1;
uniform sampler2D u_sample_spec_env2;

uniform sampler2D u_sample_mask;
uniform sampler2D u_sample_depth;

const vec3 SSSColor = vec3(0.75, 0.6, 0.5);

layout(location = 0) out vec4 frag_all;
layout(location = 1) out vec4 frag_diff;
layout(location = 2) out vec4 frag_spec;
layout(location = 3) out vec4 frag_diff_albedo;
layout(location = 4) out vec4 frag_spec_albedo;
layout(location = 5) out vec4 frag_spec_normal;
layout(location = 6) out vec4 frag_diff_normal;
layout(location = 7) out vec4 frag_texcoord;

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

	/*
	** reconstruct diffuse normals
	*/
	vec3 alpha;
	alpha.x = pow(1.0 + SSSColor.x, -5.0) - 1.0 / 32.0; // [NOTE] empirical fit
	alpha.y = pow(1.0 + SSSColor.y, -5.0) - 1.0 / 32.0; // [NOTE] empirical fit
	alpha.z = pow(1.0 + SSSColor.z, -5.0) - 1.0 / 32.0; // [NOTE] empirical fit
	vec3 diffNormalR = normalize(getNormalMap(u_sample_diff_normal, uv, u_world));
	vec3 diffNormalG = normalize(mix(diffNormalR, specNormal, alpha.g));
	vec3 diffNormalB = normalize(mix(diffNormalR, specNormal, alpha.b));

	vec4 alpha_channel = vec4(1.0, 1.0, 1.0, 1.0);
    
	frag_spec_albedo = texture(u_sample_spec_albedo, uv);
	frag_diff_albedo = texture(u_sample_diff_albedo, uv);
    
    vec4 specular_reflection = vec4(0,0,0,0);
	vec4 diff_reflection = vec4(0.0, 0.0, 0.0, 1.0);

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
	/***************************
	** Point Lights
	****************************/
	if(u_use_pointlight != uint(0))
	{
		//micro
		float microBaseRoughness = 0.15;

		//meso
		float mesoBaseRoughness = 0.25;

		float brdfMicro, brdfMeso;

		vec4 lightColor = vec4(1.0, 1.0, 1.0, 1.0);

		vec3 lights[3];
		lights[0] = u_light_pos;
		lights[1] = vec3(100,0,0);
		lights[2] = u_light_pos;

		vec4 lColor[3];
		lColor[0] = vec4(1,1,1,1);
		lColor[1] = vec4(1,1,1,1);
		lColor[2] = vec4(0,0,1,1);

		for (int i = 0; i<2; i++)
		{
			vec3 light = normalize(lights[i] - posw);
			vec3 H = normalize(light + vieww);
			/***************************
			** specular
			****************************/
			float G = geometric_attenuation(specNormal, light, vieww);

			float D = microfacet_distribution(specNormal, H, microBaseRoughness);
			brdfMicro = D*G*F / (4.0*dot(specNormal, light)*dot(specNormal, vieww));

			D = microfacet_distribution(specNormal, H, mesoBaseRoughness);
			brdfMeso = D*G*F / (4.0*dot(specNormal, light) * dot(specNormal, vieww));

			float brdf = 0.3 * brdfMicro + 0.7 * brdfMeso;
			specular_reflection += brdf*dot(specNormal, light) * lColor[i];

			/***************************
			** diffuse
			****************************/
			diff_reflection += vec4(diffuseReflection(diffNormalR, diffNormalG, diffNormalB, light, SSSColor), 1.0) * lColor[i];
		}

		frag_spec = u_specscale * alpha_channel * specular_reflection * frag_spec_albedo;
        frag_spec.w = 1.0;
		frag_diff = u_diffscale * alpha_channel * frag_diff_albedo * diff_reflection;
        frag_diff.w = 1.0;
		frag_all = frag_spec + frag_diff;
        frag_all.w = 1.0;
	}
	/***************************
	** Light Probe
	****************************/
	else
	{	
		vec3 freflect = normalize(reflect(-vieww, specNormal));
		vec4 specularEnv1 = F * texture(u_sample_spec_env1, world2UV(freflect, u_light_rot));
		vec4 specularEnv2 = F * texture(u_sample_spec_env2, world2UV(freflect, u_light_rot));
		float lobeWeight = 0.2;
		specular_reflection = lobeWeight * specularEnv1 + (1.0 - lobeWeight) * specularEnv2;
		frag_spec = u_specscale * frag_spec_albedo * specular_reflection;
        frag_spec.w = 1.0;

		vec4 diffuseEnvR = texture(u_sample_diff_env, world2UV(diffNormalR, u_light_rot));
		vec4 diffuseEnvG = texture(u_sample_diff_env, world2UV(diffNormalG, u_light_rot));
		vec4 diffuseEnvB = texture(u_sample_diff_env, world2UV(diffNormalB, u_light_rot));
		diff_reflection = vec4(diffuseEnvR.r, diffuseEnvG.g, diffuseEnvB.b, 1.0);
		frag_diff = u_diffscale * frag_diff_albedo * diff_reflection;
		
        frag_all = frag_spec + frag_diff;
        frag_all.w = 1.0;
	}
    
    frag_all = gammaCorrection(frag_all, 2.2);
    frag_diff = gammaCorrection(frag_diff, 2.2);
    frag_spec = gammaCorrection(frag_spec, 2.2);
    frag_diff_albedo = gammaCorrection(frag_diff_albedo, 2.2);
    frag_spec_albedo = gammaCorrection(frag_spec_albedo, 2.2);
    frag_spec_normal = vec4(specNormal,1.0);
    frag_diff_normal = vec4(diffNormalR,1.0);
	frag_texcoord = vec4(uv, 0.0, 1.0);
}
