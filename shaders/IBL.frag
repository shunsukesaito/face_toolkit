#version 330
#define pi 3.1415926535897932384626433832795

// for clearity,
// here world space = camera space and
// face space = model space!!
// so don't confuse them

// from rendering params
uniform uint u_texture_mode; // 0: none, 1: uv space, 2: image space
uniform uint u_enable_mask;
uniform uint u_cull_occlusion;
uniform float u_cull_offset;
uniform float u_light_rot;
uniform float u_alpha;

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

uniform sampler2D u_sample_dif_albedo;
uniform sampler2D u_sample_mask;
uniform sampler2D u_sample_diffHDRI;
uniform sampler2D u_sample_specHDRI;
uniform sampler2D u_sample_depth;

layout (location = 0) out vec4 frag_color;

vec2 world2UV(vec3 vec)
{
	mat3 Rot = mat3(cos(u_light_rot), 0, -sin(u_light_rot),
					         0, 1, 0,
					sin(u_light_rot), 0, cos(u_light_rot));
	vec = Rot * vec;

	vec2 uv = vec2(-0.5f / pi * atan(vec.x, -vec.z), 1.0 - 1.0 / pi * acos(vec.y));

    return uv;
}

vec4 gammaCorrection(vec4 vec, float g)
{
    return vec4(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g), vec.w);
}

vec3 gammaCorrection(vec3 vec, float g)
{
    return vec3(pow(vec.x, 1.0/g), pow(vec.y, 1.0/g), pow(vec.z, 1.0/g));
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
    float specScale = 1.0;
    float specAlbedo = 0.3753;

    vec4 diffuseColor = VertexIn.color;//material color
    vec3 Nw = normalize(VertexIn.normal_world.xyz);
    vec3 Nc = normalize(VertexIn.normal_camera.xyz);
    vec3 vieww = normalize(u_camera_pos-VertexIn.pos_world.xyz);
    vec3 viewc = normalize(VertexIn.pos_camera.xyz);
    vec3 freflect = normalize(reflect(-vieww, Nw));
    vec4 diffuseReflection = gammaCorrection(texture(u_sample_diffHDRI, world2UV(Nw)),2.2);
    
    float F = fresnelGraham(0.05, dot(Nw, vieww), 2.0);
    vec4 specularReflectionEM = F * gammaCorrection(texture(u_sample_specHDRI, world2UV(freflect)),2.2);

    vec2 texcoord = VertexIn.texcoord;
    
    if(u_texture_mode == uint(1)){
        diffuseColor = texture(u_sample_dif_albedo, VertexIn.texcoord);
    }
    else if(u_texture_mode == uint(2)){
        diffuseColor = texture(u_sample_dif_albedo, VertexIn.proj_texcoord);
    }
    
    if (u_enable_mask != uint(0)){
        if (texture(u_sample_mask, texcoord)[0] < 0.5)
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
    
    frag_color = specScale * specAlbedo * specularReflectionEM + diffuseColor * diffuseReflection;
    frag_color.a = u_alpha;
}
