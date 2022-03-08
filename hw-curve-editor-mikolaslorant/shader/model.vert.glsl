#version 330 core
layout (location = 0) in vec3 vPos;
layout (location = 1) in vec3 vNor;

uniform mat4 uModel;
uniform mat3 uModelInvTr; // The inverse transpose of the model matrix.
uniform mat4 uProjView;
uniform vec3 uLightPos;

out vec3 nor;
out vec3 lightDir;

void main()
{
    vec4 modelPos = uModel * vec4(vPos, 1.0);
    lightDir = uLightPos - vec3(modelPos);

    
    nor = normalize(uModelInvTr * vNor);

    gl_Position = uProjView * modelPos;
}