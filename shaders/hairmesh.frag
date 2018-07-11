#version 330

in vec4 normal;
in vec4 pos;

uniform sampler2D u_sample_depth;

layout(location = 0) out vec4 frag_color;
layout(location = 1) out vec4 frag_tri;

void main()
{
    vec3 DepthMapTexCoord = 0.5*pos.xyz / pos.w + vec3(0.5);
    if(texture(u_sample_depth, DepthMapTexCoord.xy).r <= DepthMapTexCoord.z)
        discard;

    vec3 light_direction = vec3(0, 0, -1);
    vec3 f_normal = normalize(normal.xyz);
    vec4 specular_reflection = vec4(0.2) * pow(max(0.0, dot(reflect(-light_direction, f_normal), vec3(0, 0, -1))), 16.f);
    frag_color = vec4(vec3(dot(f_normal, light_direction)), 1.0) + specular_reflection;

    frag_tri = vec4(gl_PrimitiveID,gl_PrimitiveID,gl_PrimitiveID,1.0);
}
