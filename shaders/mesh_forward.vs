#version 330 core

layout (location = 0) in vec3 v_localPosition;
layout (location = 1) in vec3 v_localNormal;
layout (location = 2) in int v_material;
layout (location = 3) in vec4 v_instanceColor;
layout (location = 4) in mat4 v_instanceMatrix;

out vec3 position;
out vec3 normal;
out vec4 color;

uniform mat4 view;
uniform mat4 projection;

void main()
{
    // Define an array of 10 colors
    vec3 colors[10];
    colors[0] = vec3(0.4706, 0.6118, 0.2);
    colors[1] = vec3(0.5255, 0.2941, 0.3529); 
    colors[2] = vec3(0.2, 0.2, 0.4588);
    colors[3] = vec3(0.7451, 0.5725, 0.2471);
    colors[4] = vec3(0.6784, 0.2627, 0.6784);
    colors[5] = vec3(0.2784, 0.6, 0.6);
    colors[6] = vec3(0.502, 0.251, 0.0039);
    colors[7] = vec3(0.698, 0.5608, 0.8314);
    colors[8] = vec3(0.0, 0.2941, 0.0);
    colors[9] = vec3(0.5, 0.5, 0.5);

    // world space position
    position = vec3(v_instanceMatrix * vec4(v_localPosition, 1.0));

    vec4 viewPos = view * vec4(position, 1.0);

    // world space normal, accounting for non-uniform scale
    mat3 normalMatrix = transpose(inverse(mat3(v_instanceMatrix)));
    normal = normalMatrix * v_localNormal;
    
    int colorIndex = max(0, v_material) % 10;
    color = v_material >= 0 ? vec4(colors[colorIndex], v_instanceColor.a) : v_instanceColor;
    
    // color = vec4(colors[2], 0.5f);

    gl_Position = projection * viewPos;

}