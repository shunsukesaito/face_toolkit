#version 330

uniform mat4 u_mvp;

in vec3 v_position;
in vec3 v_normal; // normal is actually previous position
in vec4 v_color;

out vec2 texcoord;
out vec4 color;
out vec3 pos;

void main()
{
    if(v_color[0] == 1.0){
        gl_Position = u_mvp * vec4(v_position, 1.0);
        gl_Position[0] /= gl_Position[3];
        gl_Position[1] /= gl_Position[3];
        gl_Position[2] /= gl_Position[3];
        gl_Position[3] = 1.0;
        texcoord[0] = 0.5*v_normal[0]+0.5;
        texcoord[1] = -0.5*v_normal[1]+0.5;
    }
    else{
        texcoord[0] = v_normal[0];
        texcoord[1] = v_normal[1];
        gl_Position = vec4(2.0*(v_position[0]-0.5),-2.0*(v_position[1]-0.5),v_position[2],1.0);
    }
    color = v_color;
    pos = gl_Position.xyz;
}
