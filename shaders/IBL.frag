#version 400
#define pi 3.1415926535897932384626433832795

// for clearity,
// here world space = camera space and
// face space = model space!!
// so don't confuse them

uniform uint u_texture_mode; // 0: none, 1: uv space, 2: image space
uniform uint u_diffuse_mode; // 0: SH, 1: HDRI
uniform uint u_enable_mask;
uniform uint u_enable_seg;
uniform uint u_cull_occlusion;
uniform float u_cull_offset;

uniform vec3 u_camera_pos;
uniform vec3 u_SHCoeffs[9];

in VertexData {
    vec4 color;
    vec4 normal_world;
    vec4 normal_camera;
    vec4 pos_world;
    vec4 pos_camera;
    vec2 proj_texcoord;
    vec2 texcoord;
    vec4 barycentric;
} VertexIn;

uniform sampler2D u_sample_texture;
uniform sampler2D u_sample_mask;
uniform sampler2D u_sample_HDRI;
uniform sampler2D u_sample_specHDRI;
uniform sampler2D u_sample_depth;

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

vec2 world2UV(vec3 vec)
{
	mat3 Rot = mat3(cos(u_light.x), 0, -sin(u_light.x),
					         0, 1, 0,
					sin(u_light.x), 0, cos(u_light.x));
	vec = Rot * vec;

	vec2 uv = vec2(-0.5f / pi * atan(vec.x, - vec.z), 1.0 - 1.0 / pi * acos(vec.y));

    return uv;
}

vec4 gammaCorrection(vec4 vec, float g)
{
    return vec4(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g), vec.w);
}

float fresnelGraham(float r, float cos_theta, float power)
{
      if(cos_theta<0.0){
            return 0.0;
      }
      return r+(1.0-r)*(pow(1.0-cos_theta, power));
}

//spec albedo average: 0.3753, std: 0.1655
//spec brdf (constant here) average: 0.3032, std: 0.0891
//from MERL database for the whole face

void main()
{
    vec4 diffuseColor = VertexIn.color;//material color
    vec3 Nw = normalize(VertexIn.normal_world.xyz);
    vec3 Nc = normalize(VertexIn.normal_camera.xyz);
    vec2 texcoord = VertexIn.texcoord;
    
    if(u_enable_texture == uint(1)){
        diffuseColor = texture(u_sample_texture, VertexIn.texcoord);
    }
    else if(u_enable_texture == uint(2)){
        diffuseColor = texture(u_sample_texture, VertexIn.proj_texcoord);
    }

    vec3 vieww = normalize(VertexIn.pos_world.xyz-u_cam_pose);
    vec3 viewc = normalize(VertexIn.pos_camera.xyz);
    
    vec3 freflect = normalize(reflect(-vieww, Nw));
	
    // should be nc
    vec4 diffuseReflection = gammaCorrection(texture2D(u_sample_HDRI, world2UV(Nw)),2.2);

    if(u_cull_occlusion != uint(0))
    {
        if(dot(viewc,Nc) > u_cull_offset)
            discard;

        vec3 ShadowMapTexCoord = f_position.xyz / f_position.w;
        
        float bias = 0.005*tan(acos(clamp(dot(Nc, viewc),0.0,1.0)));
        bias = clamp(bias, 0,0.01);
        if(texture2D(u_sample_depth, ShadowMapTexCoord.xy).r <= ShadowMapTexCoord.z-bias)
            discard;
    }
	
	float F = fresnelGraham(0.05, dot(Nc, viewc), 2.0);
	vec4 specularReflectionEM = F * gammaCorrection(texture2D(u_sample_specHDRI, world2UV(freflect)),2.2);

	float specScale = 1.0;
    if(u_show_specular == 0) specScale = 0.0;
    float specAlbedo = max(0, spec_albedo + 0.3753);

    if(u_diffuse_mode == uint(0))
    {
        vec3 shading = evaluateLightingModel(Nw);
        gl_FragColor = vec4(clamp(diffuseColor.xyz*shading.xyz, vec3(0.0), vec3(1.0)), diffuseColor.a);
    }
    else 
        gl_FragColor = specScale * specAlbedo * specularReflectionEM + diffuseColor * diffuseReflection;
    
    if (u_enable_mask != uint(0)){
        if (texture(u_sample_mask, texcoord)[0] < 0.5)
            discard;
    }

}
