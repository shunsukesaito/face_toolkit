vec4 gammaCorrection(vec4 vec, float g)
{
    return vec4(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g), vec.w);
}

vec3 gammaCorrection(vec3 vec, float g)
{
    return vec3(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g));
}

vec2 world2UV(vec3 vec, float light_rot)
{
    mat3 Rot = mat3(cos(light_rot), 0, -sin(light_rot),
                    0, 1, 0,
                    sin(light_rot), 0, cos(light_rot));
    vec = Rot * vec;
    
    vec2 uv = vec2(-0.5f / pi * atan(vec.x, -vec.z), 1.0 - 1.0 / pi * acos(vec.y));
    
    return uv;
}

vec3 calcTangentNormal(sampler2D mesogeo, vec2 uv, vec3 n, vec4 Qd1, vec4 Qd2, float mesoMapSize)
{
	float ddMeso = 1.0/mesoMapSize;
	vec3 e1_world = Qd1.xyz;
	vec3 e2_world = Qd2.xyz;

	float scaleFactor = 0.5; // [FIXME] adjust this to control the bumpiness of the normal
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

vec3 getNormalMap(sampler2D sampler, vec2 uv, mat4 world)
{
    vec3 normalMap = normalize(texture(sampler, uv).xyz);
    return normalize((world * vec4(normalMap,0.0)).xyz);
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

vec3 diffuseReflection(vec3 normal_R, vec3 normal_G, vec3 normal_B, vec3 L, vec3 SSSColor)
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

float ward_anisotropic(vec3 n, vec3 light, vec3 view, float ax, float ay, vec3 x, vec3 y)
{
    float NdotL = dot(n, light);
    vec3 r = normalize(reflect(-light, n));
	float NdotV = dot(n, view);
    if(NdotL < 0.0 || NdotV < 0.0) return 0.0;
    vec3 h = normalize(light+view);
    float NdotH = dot(n, h);
    float HdotX = dot(h, x);
    float HdotY = dot(h, y);
	float power = -2.0*((HdotX/ax)*(HdotX/ax) + (HdotY/ay)*(HdotY/ay))/(1.0+NdotH);
    float upper = exp(power);
    float lower = 4.0*pi*ax*ay*sqrt(NdotV*NdotL);
    return upper/lower;
}

// theta is the angle between the viewing direction and the half vector
float Fresnel_Schlick(float cosTheta, float F0, float fresExp)
{
    return F0 + (1.0-F0) * pow( 1.0 - cosTheta, fresExp);
}

float chiGGX(float v)
{
    return (v > 0) ? 1 : 0;
}

float GGX_Distribution(vec3 n, vec3 h, float alpha)
{
    float NoH = dot(n,h);
    float alpha2 = alpha * alpha;
    float NoH2 = NoH * NoH;
    float den = NoH2 * alpha2 + (1 - NoH2);
    return (chiGGX(NoH) * alpha2) / ( pi * den * den );
}

float GGX_PartialGeometryTerm(vec3 v, vec3 n, vec3 h, float alpha)
{
    float VoH2 = clamp(dot(v,h),0.0,1.0);
    float chi = chiGGX( VoH2 / clamp(dot(v,n),0.0,1.0) );
    VoH2 = VoH2 * VoH2;
    float tan2 = ( 1 - VoH2 ) / VoH2;
    return (chi * 2) / ( 1 + sqrt( 1 + alpha * alpha * tan2 ) );
}
