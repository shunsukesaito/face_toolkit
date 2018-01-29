#version 400

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec3 v_normal;
layout (location = 2) in vec2 v_texcoord;

out vec3 vPosition;
out vec3 vWorldPos;
out vec3 vNormal;

void main(void)
{
	// copy output
	vWorldPos = v_position;
    vPosition = vec3(v_texcoord, 0.0) - vec3(0.5, 0.5, 0.0);
    vPosition[0] *= 2;
    vPosition[1] *= 2;
	vNormal = normalize(v_normal);
}
