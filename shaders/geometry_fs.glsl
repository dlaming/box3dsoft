#version 330 core

layout (location = 0) out vec3 gPosition;
layout (location = 1) out vec3 gNormal;
layout (location = 2) out vec4 gAlbedoSpec;

in vec3 fragmentPosition;
in vec3 normal;
in vec4 color;

void main()
{    
    // store the fragment position vector in the first gbuffer texture
    gPosition = fragmentPosition;

    // also store the per-fragment normals into the gbuffer
    gNormal = normalize(normal);

    gAlbedoSpec = color;
}