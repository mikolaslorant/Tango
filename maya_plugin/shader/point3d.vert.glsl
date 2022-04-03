#version 330 core
layout (location = 0) in vec3 vPos;

uniform float uSize;
uniform mat4 uModel;
uniform mat4 uProjView;

void main()
{
    gl_Position = uProjView * uModel * vec4(vPos, 1);
    gl_PointSize = uSize;
}