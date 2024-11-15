#version 330

uniform mat4 u_mvp;
uniform mat4 u_modelview;
uniform mat4 u_shadow_mvp;

uniform	uint u_tex_mode;

in vec3 v_position;
in vec4 v_color;
in vec3 v_normal;
in vec2 v_texcoord;

out VertexData {
    vec4 color;
    vec4 normal;
    vec4 normalCamera;
    vec4 pos;
    vec4 pos_shadow_mvp;
    vec2 proj_texcoord;
    vec2 texcoord;
    uint index;
} VertexOut;

void main()
{
    VertexOut.color = v_color;

    VertexOut.normal = vec4(v_normal, 1.0);
    VertexOut.normalCamera = u_modelview * vec4(v_normal, 0.0);
    
    vec4 posWorld = u_mvp * vec4(v_position, 1.0);

    VertexOut.pos = u_modelview * vec4(v_position, 1.0);;
    VertexOut.pos_shadow_mvp = u_shadow_mvp * vec4(v_position, 1.0);
    VertexOut.proj_texcoord[0] = 0.5*(posWorld[0] / posWorld[3]) + 0.5;
    VertexOut.proj_texcoord[1] = -0.5*(posWorld[1] / posWorld[3]) + 0.5;
    
    VertexOut.texcoord = v_texcoord;

    VertexOut.index = uint(gl_VertexID);

    if (u_tex_mode != uint(0)){
		gl_Position = vec4(v_texcoord, 0.0, 1.0) - vec4(0.5, 0.5, 0.0, 0.0);
		gl_Position[0] *= 2;
		gl_Position[1] *= 2;
	}
	else{
		gl_Position = posWorld;
	}
}
