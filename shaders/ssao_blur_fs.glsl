#version 330 core
out float FragColor;

in vec2 TexCoords;

uniform sampler2D ssaoInput;
uniform sampler2D gPosition;

// Depth difference (in view-space units) over which a tap's weight halves.
// Tight enough to keep silhouette edges sharp, loose enough not to leave
// per-tap noise behind on smooth surfaces.
const float DEPTH_SIGMA = 0.25;

void main()
{
    vec2 texelSize = 1.0 / vec2(textureSize(ssaoInput, 0));
    float centerZ = texture(gPosition, TexCoords).z;

    // Sky / no-geometry: pass through. Avoids contaminating real geometry's
    // bilateral weights with the zero-init sentinel from the G-buffer.
    if (centerZ == 0.0)
    {
        FragColor = texture(ssaoInput, TexCoords).r;
        return;
    }

    float invTwoSigmaSq = 1.0 / (2.0 * DEPTH_SIGMA * DEPTH_SIGMA);

    float result = 0.0;
    float weightSum = 0.0;
    for (int x = -2; x < 2; ++x)
    {
        for (int y = -2; y < 2; ++y)
        {
            vec2 sampleUV = TexCoords + vec2(float(x), float(y)) * texelSize;
            float tapZ = texture(gPosition, sampleUV).z;
            // Drop sky taps so they don't pull the weighted average toward 1.0
            // along silhouettes against the background.
            if (tapZ == 0.0) continue;
            float dz = centerZ - tapZ;
            float w = exp(-dz * dz * invTwoSigmaSq);
            result += texture(ssaoInput, sampleUV).r * w;
            weightSum += w;
        }
    }
    FragColor = (weightSum > 0.0) ? (result / weightSum) : texture(ssaoInput, TexCoords).r;
}
