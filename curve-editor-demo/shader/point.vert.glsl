#version 330 core
layout (location = 0) in vec3 vPos;

uniform float size;

void main()
{
    gl_Position = vec4(vPos, 1);
    gl_PointSize = size;
}