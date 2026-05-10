#version 330 core

layout (location = 0) in vec3 v_localPosition;
layout (location = 1) in vec3 v_localNormal;
layout (location = 2) in int v_material;
layout (location = 3) in vec4 v_instanceColor;
layout (location = 4) in mat4 v_instanceMatrix;

uniform mat4 lightSpaceMatrix;

void main()
{
    gl_Position = lightSpaceMatrix * v_instanceMatrix * vec4(v_localPosition, 1.0);
}
