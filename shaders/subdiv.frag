#version 400
//#extension GL_ARB_tessellation_shader : enable
// in / out

in vec4 tePosition;
in vec4 teWorldPos;
in vec4 teNormal;

layout (location = 0) out vec4 frag_position;
layout (location = 1) out vec4 frag_normal;

void main(void)
{
	frag_position = teWorldPos;
    frag_normal = teNormal;
}
