#version 330

in vec3 normal;
in vec4 color;

layout (location = 0) out vec4 frag_color;

void main() 
{
    vec3 light_direction = vec3(0, 0, -1);
    vec3 f_normal = normalize(normal);
    vec4 specular_reflection = vec4(0.2) * pow(max(0.0, dot(reflect(-light_direction, f_normal), vec3(0, 0, -1))), 16.f);
    frag_color = vec4(dot(f_normal, light_direction) * color.rgb, color.a) + specular_reflection;
}
