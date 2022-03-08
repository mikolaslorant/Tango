#version 330 core
in vec3 col;

layout(location = 0) out vec4 fCol;

uniform float uAlpha = 1.0;

void main()
{
    fCol = vec4(col, uAlpha);
} 