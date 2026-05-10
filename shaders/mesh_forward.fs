#version 330 core

#include "common.glsl"

layout (location = 0) out vec4 fragmentColor;

in vec3 position;   // world space
in vec3 normal;     // world space
in vec4 color;      // sRGB albedo, alpha is opacity

uniform vec3 sunDirWorld;
uniform vec3 cameraPos;

void main()
{
    vec3 N = normalize(normal);
    vec3 viewRay = normalize(position - cameraPos);

    vec3 baseColor = SrgbToLinear(color.rgb);

    vec3 ambient = HemisphericAmbient(N);

    vec3 sunColor = SrgbToLinear(SUN_COLOR_SRGB) * SUN_INTENSITY;
    float NdotL = max(dot(N, sunDirWorld), 0.0);
    vec3 diffuse = NdotL * sunColor;

    vec3 lit = baseColor * (ambient + diffuse);

    float dist = length(position - cameraPos);
    float fog = 1.0 - exp(-FOG_DENSITY * dist);
    lit = mix(lit, SkyColor(viewRay, sunDirWorld), fog);

    fragmentColor = vec4(DitherSrgb(LinearToSrgb(ACESFilm(lit)), gl_FragCoord.xy), color.a);
}
