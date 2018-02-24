#version 400

uniform sampler2D u_sample_pos;
uniform mat4 u_modelview;
uniform float u_delta;

layout (location = 0) in vec2 v_texcoord;

out vec2 vTexcoord;
out vec4 vNormal;

vec3 computeNormalFromUV(vec2 uv)
{
    float delta = u_delta;
	vec2 uv00 = vec2(uv[0]-delta,uv[1]-delta);
	vec2 uv10 = vec2(uv[0],uv[1]-delta);
	vec2 uv01 = vec2(uv[0]-delta,uv[1]);
	vec2 uv11 = vec2(uv[0],uv[1]);
	vec2 uv20 = vec2(uv[0]+delta,uv[1]-delta);
	vec2 uv02 = vec2(uv[0]-delta,uv[1]+delta);
    vec2 uv21 = vec2(uv[0]+delta,uv[1]);
    vec2 uv12 = vec2(uv[0],uv[1]+delta);	
    vec2 uv22 = vec2(uv[0]+delta,uv[1]+delta);

    vec3 p00 = texture(u_sample_pos, uv00).xyz;
    vec3 p10 = texture(u_sample_pos, uv10).xyz;
    vec3 p01 = texture(u_sample_pos, uv01).xyz;
    vec3 p11 = texture(u_sample_pos, uv11).xyz;
    vec3 p20 = texture(u_sample_pos, uv20).xyz;
    vec3 p02 = texture(u_sample_pos, uv02).xyz;
    vec3 p21 = texture(u_sample_pos, uv21).xyz;
    vec3 p12 = texture(u_sample_pos, uv12).xyz;
    vec3 p22 = texture(u_sample_pos, uv22).xyz;
	
	if(length(p00) < 1.e-8) p00 = p11;
	if(length(p10) < 1.e-8) p10 = p11;
	if(length(p01) < 1.e-8) p01 = p11;
	if(length(p20) < 1.e-8) p20 = p11;
	if(length(p02) < 1.e-8) p02 = p11;
	if(length(p21) < 1.e-8) p21 = p11;
	if(length(p12) < 1.e-8) p12 = p11;
	if(length(p22) < 1.e-8) p22 = p11;

	vec3 dx = 3.0*p20+10.0*p21+3.0*p22-3.0*p00-10.0*p01-3.0*p02;
	vec3 dy = 3.0*p02+10.0*p12+3.0*p22-3.0*p00-10.0*p10-3.0*p20;

	return normalize(-cross(dy,dx));
}

void main(void)
{
	// copy output
    vTexcoord = v_texcoord;
    vNormal = u_modelview * vec4(computeNormalFromUV(v_texcoord),0.0);
}
