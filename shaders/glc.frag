#version 330

uniform float u_depth;
uniform float u_den;

in vec4 normalWorld;
in vec4 normalModel;
in vec4 position;
in vec2 barycentric;

uniform vec2 u_uv1;
uniform vec2 u_uv2;
uniform vec2 u_uv3;

layout (location = 0) out vec4 frag_color;

void main() 
{
    if(barycentric[0]<0.0 || barycentric[1]<0.0 || barycentric[0]+barycentric[1]>1.0)
        discard;

    vec3 light_direction = vec3(0, 0, -1);
    vec4 color = vec4(1.0);

    vec3 f_normal = normalize(normalWorld.xyz);
    vec4 specular_reflection = vec4(0.2) * pow(max(0.0, dot(reflect(-light_direction, f_normal), vec3(0, 0, -1))), 16.f);
    frag_color = vec4(abs(dot(f_normal, light_direction))* color.rgb, color.a) + specular_reflection;
}
