#version 330 core

layout (location = 0) in vec3 v_position;
layout (location = 1) in int v_flags;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform vec3 color;

out vec3 overrideColor;

void main()
{
    overrideColor = v_flags != 0 ? vec3(0.9, 0.9, 0.9) : color;
    gl_Position = projection * view * model * vec4(v_position, 1.0);
}
