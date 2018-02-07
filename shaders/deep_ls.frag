#version 330
#define pi 3.1415926535897932384626433832795

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

vec4 gammaCorrection(vec4 vec, float g)
{
    return vec4(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g), vec.w);
}

vec3 gammaCorrection(vec3 vec, float g)
{
    return vec3(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g));
}

vec2 world2UV(vec3 vec)
{
    mat3 Rot = mat3(cos(u_light_rot), 0, -sin(u_light_rot),
                    0, 1, 0,
                    sin(u_light_rot), 0, cos(u_light_rot));
    vec = Rot * vec;
    
    vec2 uv = vec2(-0.5f / pi * atan(vec.x, -vec.z), 1.0 - 1.0 / pi * acos(vec.y));
    
    return uv;
}

vec3 calcTangentNormal(sampler2D mesogeo)
{
	float mesoMapSize = 6000; // [FIXME] hardcoded. size of the displacement
	float ddMeso = 1.0/mesoMapSize;
	vec3 e1_world = VertexIn.Qd1.xyz;
	vec3 e2_world = VertexIn.Qd2.xyz;

    vec3 n = normalize(VertexIn.normal_world.xyz);

	float scaleFactor = 1.0; // [FIXME] adjust this to control the bumpiness of the normal
    vec2 uv = VertexIn.texcoord;
	float dxx = texture(mesogeo, vec2(uv.x+ddMeso, uv.y)).r;
	float xxd = texture(mesogeo, vec2(uv.x-ddMeso, uv.y)).r;
	float dyy = texture(mesogeo, vec2(uv.x, uv.y+ddMeso)).r;
	float yyd = texture(mesogeo, vec2(uv.x, uv.y-ddMeso)).r;

	float dx = dxx - xxd; 
	float dy = dyy - yyd;
   
	vec3 e1 = e1_world*ddMeso*2.0 + n*dx*scaleFactor;
	vec3 e2 = e2_world*ddMeso*2.0 + n*dy*scaleFactor;

	vec3 normal = normalize(cross(e1, e2));
	return normal;
}

vec3 getNormalMap(sampler2D sampler, vec2 uv)
{
    vec3 normalMap = normalize(texture(sampler, uv).xyz);
    return normalize((u_world * vec4(normalMap,0.0)).xyz);
}

vec3 getNormalMap2(sampler2D sampler, vec2 uv)
{
    return normalize(texture(sampler, uv).xyz);
}

float calcFresnel(float R0, float F, float fresExp, vec3 view, vec3 fspecNormal)
{
	return R0 + (F - R0) * pow(1.0 - dot(view, fspecNormal), fresExp);
}

float fresnelGraham(float r, float cos_theta, float power)
{
      if(cos_theta<0.0){
            return 0.0;
      }
      return r+(1.0-r)*(pow(1.0-cos_theta, power));
}

vec3 calcFresnel3(vec3 R0, float F, float fresExp, vec3 view, vec3 fspecNormal)
{
	return R0 + (vec3(F) - R0) * pow(1.0 - dot(view, fspecNormal), fresExp);
}

vec3 diffuseReflection(vec3 normal_R, vec3 normal_G, vec3 normal_B, vec3 L)
{
    vec3 dots = vec3(dot(normal_R, L), dot(normal_G, L), dot(normal_B, L));
    return pow(clamp((1.0f - dots) * SSSColor *0.5 + dots, 0.0, 1.0), SSSColor * 0.5 * 4.0f + 1.0f);
}

float geometric_attenuation(vec3 n, vec3 wi, vec3 wo)
{
	vec3 wh = normalize(wi+wo);
	float NdotWh = abs(dot(n,wh));
	float NdotWo = abs(dot(n,wo));
	float NdotWi = abs(dot(n,wi));
	float WodotWh = abs(dot(wo,wh));
	
    if(WodotWh==0.0)
		return 0.0;

	return min(1.0, min((2.0 * NdotWh * NdotWo / WodotWh),
						(2.0 * NdotWh * NdotWi / WodotWh)));
}

float microfacet_distribution(vec3 n, vec3 wh, float m)
{
	float NdotWh = (dot(n,wh));
	if(NdotWh<=0.0) return 0.0;
	if(NdotWh >= 1.0)
		NdotWh = 1.0;

    float tanByMSq = (NdotWh*NdotWh-1.0)/(NdotWh*NdotWh*m*m);
	float upper = exp(tanByMSq);
	float lower = pi*m*m*NdotWh*NdotWh*NdotWh*NdotWh;
	return upper/lower;
}

void main(void)
{
	vec2 uv = VertexIn.texcoord;

    // use normal from displacement
	vec3 specNormal = calcTangentNormal(u_sample_disp);

    vec3 posw = VertexIn.pos_world.xyz;
    vec3 Nw = normalize(VertexIn.normal_world.xyz);
    vec3 Nc = normalize(VertexIn.normal_camera.xyz);
    vec3 vieww = normalize(u_camera_pos-VertexIn.pos_world.xyz);
    vec3 viewc = normalize(VertexIn.pos_camera.xyz);
    float F = fresnelGraham(0.05, dot(specNormal, vieww), 2.0);

	/*
	** reconstruct diffuse normals
	*/
	vec3 alpha;
	alpha.x = pow(1.0 + SSSColor.x, -5.0) - 1.0 / 3.0; // [NOTE] empirical fit
	alpha.y = pow(1.0 + SSSColor.y, -5.0) - 1.0 / 3.0; // [NOTE] empirical fit
	alpha.z = pow(1.0 + SSSColor.z, -5.0) - 1.0 / 3.0; // [NOTE] empirical fit
	vec3 diffNormalR = normalize(mix(Nw, specNormal, alpha.r));
	vec3 diffNormalG = normalize(mix(Nw, specNormal, alpha.g));
	vec3 diffNormalB = normalize(mix(Nw, specNormal, alpha.b));

	vec4 alpha_channel = vec4(1.0, 1.0, 1.0, 1.0);
    
	frag_spec_albedo = texture(u_sample_spec_albedo, uv);
    //frag_spec_albedo.xyz *= 0.5;
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
			diff_reflection += vec4(diffuseReflection(diffNormalR, diffNormalG, diffNormalB, light), 1.0) * lColor[i];
		}

		frag_spec = u_specscale * alpha_channel * gammaCorrection(specular_reflection,2.2) * frag_spec_albedo;
        frag_spec.w = 1.0;
		frag_diff = u_diffscale * alpha_channel * frag_diff_albedo * gammaCorrection(diff_reflection,2.2);
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
		vec4 specularEnv1 = F * gammaCorrection(texture(u_sample_spec_env1, world2UV(freflect)),2.2);
		vec4 specularEnv2 = F * gammaCorrection(texture(u_sample_spec_env2, world2UV(freflect)),2.2);
		float lobeWeight = 0.2;
		specular_reflection = lobeWeight * specularEnv1 + (1.0 - lobeWeight) * specularEnv2;
		frag_spec = u_specscale * frag_spec_albedo * specular_reflection;
        frag_spec.w = 1.0;

		vec4 diffuseEnvR = gammaCorrection(texture(u_sample_diff_env, world2UV(diffNormalR)),2.2);
		vec4 diffuseEnvG = gammaCorrection(texture(u_sample_diff_env, world2UV(diffNormalG)),2.2);
		vec4 diffuseEnvB = gammaCorrection(texture(u_sample_diff_env, world2UV(diffNormalB)),2.2);
		diff_reflection = vec4(diffuseEnvR.r, diffuseEnvG.g, diffuseEnvB.b, 1.0);
		frag_diff = u_diffscale * frag_diff_albedo * diff_reflection;
		
        frag_all = frag_spec + frag_diff;
        frag_all.w = 1.0;
	}
    
    frag_all = frag_all;
    frag_diff = frag_diff;
    frag_spec = frag_spec;
    frag_diff_albedo = frag_diff_albedo;
    frag_spec_albedo = frag_spec_albedo;
    frag_spec_normal = vec4(specNormal,1.0);
    frag_diff_normal = vec4(diffNormalR,1.0);
}
