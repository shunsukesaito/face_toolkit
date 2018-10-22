#version 330

uniform mat4 u_T;
uniform mat4 u_modelview;

uniform vec2 u_uv1;
uniform vec2 u_uv2;
uniform vec2 u_uv3;

uniform vec4 u_d1;
uniform vec4 u_d2;
uniform vec4 u_d3;

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec3 v_normal;

out vec4 normalWorld;
out vec4 normalModel;
out vec2 barycentric;
out vec4 position;

float determinant(float m11, float m12, float m13,
                  float m21, float m22, float m23,
                  float m31, float m32, float m33)
{
    return m11*(m22*m33-m23*m32)-m12*(m21*m33-m23*m31)+m13*(m21*m32-m22*m31);
}

void main()
{
    normalWorld = u_modelview*vec4(v_normal, 0);
    normalModel = vec4(v_normal,0);
    position = u_modelview*vec4(v_position,1.0);
    vec4 pos_norm = u_T * position;

    float x = pos_norm[0];
    float y = pos_norm[1];
    float z = pos_norm[2];
    float den = determinant(z*u_d1[0]+u_d1[1],z*u_d1[2]+u_d1[3],1.f,
                            z*u_d2[0]+u_d2[1],z*u_d2[2]+u_d2[3],1.f,
                            z*u_d3[0]+u_d3[1],z*u_d3[2]+u_d3[3],1.f);
    if (abs(den) < 1.e-8) den = den > 0 ? 1.e-8 : -1.e-8;
    float u = determinant(z*u_d1[0]+u_d1[1],z*u_d1[2]+u_d1[3],1.f,
                          x,y,1.f,
                          z*u_d3[0]+u_d3[1],z*u_d3[2]+u_d3[3],1.f)/den;

    float v = determinant(z*u_d1[0]+u_d1[1],z*u_d1[2]+u_d1[3],1.f,
                          z*u_d2[0]+u_d2[1],z*u_d2[2]+u_d2[3],1.f,
                          x,y,1.f)/den;

    barycentric = vec2(u,v);
    vec2 uv = (1.0-u-v)*u_uv1+u*u_uv2+v*u_uv3; // actual screen coordinate
    gl_Position = vec4(2.0*uv[0]-1.0,2.0*uv[1]-1.0,z,1.0); // TODO: the current z is approximation
}
