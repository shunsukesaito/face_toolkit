#version 330

#extension GL_EXT_gpu_shader4 : enable
#define pi 3.1415926535897932384626433832795

// from rendering params
uniform uint u_enable_mask;
uniform uint u_cull_occlusion;
uniform float u_cull_offset;
uniform float u_light_pos;

// from camera
uniform vec3 u_camera_pos;

in VertexData {
    vec4 color;
    vec4 normal_world;
    vec4 normal_camera;
    vec4 pos_shadow_mvp;
    vec4 pos_world;
    vec4 pos_camera;
    vec2 proj_texcoord;
    vec2 texcoord;
} VertexIn;

uniform sampler2D u_sample_mask;
uniform sampler2D u_sample_diffAlbedo;
uniform sampler2D u_sample_specAlbedo;
uniform sampler2D u_sample_diffNormal;
uniform sampler2D u_sample_specNormal;
uniform sampler2D u_sample_depth;

const vec3 SSSColor = vec3(0.75, 0.6, 0.5);

vec4 gammaCorrection(vec4 vec, float g)
{
    return vec4(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g), vec.w);
}

vec3 getNormalMap(sampler2D sampler, vec2 uv)
{
    vec3 normalMap = normalize(texture2D(sampler, uv.st).xyz);
    return normalMap;
}

float calcFresnel(float R0, float F, float fresExp, vec3 view, vec3 fspecNormal)
{
	return R0 + (F - R0) * pow(1.0 - dot(view, fspecNormal), fresExp);
}

float schlick(float r, float cos_theta)
{
      if(cos_theta<0.0){
            return 0.0;
      }
      return r+(1.0-r)*(pow(1.0-cos_theta,5.0));
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


vec3 diffuseReflection2(vec3 normal_R, vec3 normal_G, vec3 normal_B, vec3 L)
{
	float ar = acos( dot( normal_R, L) );
	float ag = acos( dot( normal_G, L) );
	float ab = acos( dot( normal_B, L) );
	
	vec3 diff;

	diff.x = exp( -0.5*ar*ar - 0.1*ar*ar*ar*ar*ar*ar);
	diff.y = exp( -0.5*ag*ag - 0.1*ag*ag*ag*ag*ag*ag );
	diff.z = exp( -0.5*ab*ab - 0.1*ab*ab*ab*ab*ab*ab );
	
	return diff;
}

float geometric_attenuation(vec3 n, vec3 wi, vec3 wo)
{
	vec3 wh = (wi+wo);
	wh = normalize(wh);
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
    vec3 Nw = normalize(VertexIn.normal_world.xyz);
    vec3 Nc = normalize(VertexIn.normal_camera.xyz);
    vec3 vieww = normalize(VertexIn.pos_world.xyz-u_camera_pos);
    vec3 viewc = normalize(VertexIn.pos_camera.xyz);

    if (u_enable_mask != uint(0)){
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

	vec3 specNormal = normalize(getNormalMap(u_sample_specNormal, uv));
	/*
	** reconstruct diffuse normals
	*/
	vec3 alpha;
	alpha.x = pow(1.0 + SSSColor.x, -5.0) - 1.0 / 32.0; // [NOTE] empirical fit
	alpha.y = pow(1.0 + SSSColor.y, -5.0) - 1.0 / 32.0; // [NOTE] empirical fit
	alpha.z = pow(1.0 + SSSColor.z, -5.0) - 1.0 / 32.0; // [NOTE] empirical fit
	vec3 diffNormalR = normalize(getNormalMap(u_sample_diffNormal, uv));
	vec3 diffNormalG = normalize(mix(diffNormalR, specNormal, alpha.g));
	vec3 diffNormalB = normalize(mix(diffNormalR, specNormal, alpha.b));

	vec4 alpha_channel = vec4(1.0, 1.0, 1.0, 1.0);
	vec4 spec_color = texture2D(u_sample_specAlbedo, uv);
	vec4 diff_color = texture2D(u_sample_diffAlbedo, uv);
	
	/***************************
	** specular
	****************************/
	/***************************
	*ward 1
	****************************/

	//micro
	float microBaseRoughness = 0.15;

	//meso
	float mesoBaseRoughness = 0.25;

	float brdfMicro;
	float brdfMeso;
    
    vec4 specular_reflection = vec4(0,0,0,0);
	vec4 diff_reflection = vec4(0.0, 0.0, 0.0, 1.0);
	vec4 lightColor = vec4(1.0, 1.0, 1.0, 1.0);

	vec3 lights[3];
	lights[0] = lightPos;
	lights[1] = vec3(-100,0,0);
	lights[2] = lightPos;
	vec4 lColor[3];
	lColor[0] = vec4(1,1,1,1);
	lColor[1] = vec4(1,1,1,1);
	lColor[2] = vec4(0,0,1,1);
	for (int i = 0; i<2; i++)
	{
		vec3 light = lights[i];
		light = normalize(light - VertexIn.pos_world.xyz);
		vec3 H = normalize(light + vieww);

		float F = fresnelGraham(0.05, dot(specNormal, vieww), 2.0);

		float G = geometric_attenuation(specNormal, light, vieww);

		float D = microfacet_distribution(specNormal, H, microBaseRoughness);
		brdfMicro = D*G*F / (4.0*dot(specNormal, light)*dot(specNormal, vieww));

		D = microfacet_distribution(specNormal, H, mesoBaseRoughness);
		brdfMeso = D*G*F / (4.0*dot(specNormal, light) * dot(specNormal, vieww));

		float brdf = 0.3 * brdfMicro + 0.7 * brdfMeso;
		specular_reflection += brdf*dot(specNormal, light) * lColor[i];

		diff_reflection += vec4(diffuseReflection(diffNormalR, diffNormalG, diffNormalB, light), 1.0) * lColor[i];
	}

	vec4 specular = alpha_channel * specular_reflection * spec_color;
	vec4 diffuse = alpha_channel * diff_color * diff_reflection;

	gl_FragColor = gammaCorrection(specular + diffuse, 2.2);
}
