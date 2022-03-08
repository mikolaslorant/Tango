#version 330 core
layout (location = 0) in vec3 vPos;
layout (location = 1) in vec3 vNor;
layout (location = 2) in int vJointNum;  // The number of joints that influence this vertex
layout (location = 3) in vec2 vJointWeights[6];   // The array of joints' weights that influence this vertex

uniform mat4 uModel;
uniform mat3 uModelInvTr; // The inverse transpose of the model matrix.
uniform mat4 uProjView;
uniform vec3 uLightPos;

uniform mat4 uBindMats[64]; // The bind matrix of each joint
uniform mat4 uJointTransMats[64]; // The overall transform matrix of each joint
uniform vec3 color = vec3(0.7, 0.7, 0.6);

out vec3 nor;
out vec3 lightDir;
out vec3 vcolor;

void main()
{
    vec4 deformPos = vec4(0, 0, 0, 0);
    vec4 deformNor = vec4(0, 0, 0, 0);
    for (int i = 0; i < vJointNum; ++i)
    {
        int id = int(vJointWeights[i][0]);
        float weight = vJointWeights[i][1];
        deformPos += weight * uJointTransMats[id] * uBindMats[id] * vec4(vPos, 1);
        deformNor += weight * transpose(inverse(uJointTransMats[id])) * transpose(inverse(uBindMats[id])) * vec4(vNor, 0);
    }

    vec4 modelPos = uModel * deformPos;
    //vec4 modelPos = uModel * vec4(vPos, 1);
    lightDir = uLightPos - vec3(modelPos);
    nor = normalize(uModelInvTr * vec3(deformNor));
    //nor = normalize(uModelInvTr * vec3(vNor));
    gl_Position = uProjView * modelPos;
    vcolor = color;
}