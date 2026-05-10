#version 330 core

in vec2 TexCoords;

uniform sampler2D depthTexture;

void main()
{
    gl_FragDepth = texture(depthTexture, TexCoords).r;
}
