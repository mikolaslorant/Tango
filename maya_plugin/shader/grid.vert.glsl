#version 330 core
layout (location = 0) in vec3 vPos;
layout (location = 1) in vec3 vCol;

uniform mat4 uProjViewModel;

out vec3 col;

void main()
{

    gl_Position = uProjViewModel * vec4(vPos, 1.0);
    col = vCol;
}