#version 330 core

// Static
layout (location = 0) in vec3 v_localPosition;
layout (location = 1) in int v_flags;

// Instance
layout (location = 2) in vec4 v_instanceColor;
layout (location = 3) in mat4 v_instanceMatrix;

out vec3 overrideColor;

uniform mat4 projection;
uniform mat4 view;

void main()
{
    overrideColor = v_flags != 0 ? vec3(0.6, 0.6, 0.6) : 0.5 * v_instanceColor.rgb;
    gl_Position = projection * view * v_instanceMatrix * vec4(v_localPosition, 1.0);
}
