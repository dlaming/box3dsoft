#version 330 core

layout (location = 0) out vec4 fragmentColor;

in vec3 overrideColor;

void main()
{
    fragmentColor = vec4(overrideColor, 1.0);
}
