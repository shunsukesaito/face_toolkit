#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;

// params
uniform	uint g_tex_view;

in vec3 v_position;
in vec4 v_color;
in vec3 v_normal;
in vec2 v_texcoord;

out VertexData {
    vec4 color;
    vec4 normal;
    vec4 normalCamera;
    vec4 light;
    vec4 pos;
    vec2 proj_texcoord;
    vec2 texcoord;
} VertexOut;

void main()
{
    VertexOut.color = v_color;

    VertexOut.normal = vec4(v_normal, 1.0);
    VertexOut.normalCamera = u_modelview * vec4(v_normal, 0.0);
    
    VertexOut.light = vec4(normalize(vec3(0.0, 0.1, -1.0)),0.0);
    
    vec4 posWorld = u_mvp * vec4(v_position, 1.0);
	if (g_tex_view != uint(0))
	{
		VertexOut.pos = vec4(v_texcoord, 0.0, 1.0) - vec4(0.5, 0.5, 0.0, 0.0);
		VertexOut.pos[0] *= 2;
		VertexOut.pos[1] *= 2;
	}
	else
	{
		VertexOut.pos = posWorld;
	}

    VertexOut.proj_texcoord[0] = 0.5*(posWorld[0] / posWorld[3]) + 0.5;
    VertexOut.proj_texcoord[1] = 1.0 - 0.5*(posWorld[1] / posWorld[3]) + 0.5;
    
    VertexOut.texcoord = v_texcoord;

    gl_Position = VertexOut.pos;
}
