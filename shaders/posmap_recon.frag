#version 400
//#extension GL_ARB_tessellation_shader : enable
// in / out

uniform float u_alpha;

in vec4 tePosition;
in vec4 teWorldPos;
in vec4 teNormal;

layout (location = 0) out vec4 frag_color;

void main() 
{
    vec4 color = vec4(1,1,1,1);
    vec3 light_direction = vec3(0, 0, -1);
    vec3 f_normal = normalize(teNormal.xyz);
    vec4 specular_reflection = vec4(0.2) * pow(max(0.0, dot(reflect(-light_direction, f_normal), vec3(0, 0, -1))), 16.f);
    frag_color = vec4(dot(f_normal, light_direction) * color.rgb, color.a) + specular_reflection;
    frag_color.a = u_alpha;
}
