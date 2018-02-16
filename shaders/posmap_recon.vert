#version 400

layout (location = 0) in vec2 v_texcoord;

out vec2 vTexcoord;

void main(void)
{
	// copy output
    vTexcoord = v_texcoord;
}
