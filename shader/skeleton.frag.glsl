#version 330 core
out vec4 fColor;

uniform vec3 uColor;

void main()
{
    fColor = vec4(uColor, 1.0);
} 