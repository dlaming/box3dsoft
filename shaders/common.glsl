// Shared constants and helpers used across deferred and forward fragment shaders.
// Included via the loader's `#include "common.glsl"` directive (no `#version`
// here; the including shader provides it).
//
// All color constants are authored in sRGB (display) space and converted to
// linear at use. Intensities are linear-space scalars.

vec3 SrgbToLinear(vec3 c) { return pow(c, vec3(2.2)); }
vec3 LinearToSrgb(vec3 c) { return pow(c, vec3(1.0 / 2.2)); }

vec3 ACESFilm(vec3 x)
{
    float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

const vec3  SUN_COLOR_SRGB      = vec3(1.00, 0.97, 0.92);
const float SUN_INTENSITY       = 0.5;
const vec3  SKY_AMBIENT_SRGB    = vec3(0.55, 0.60, 0.68);
const vec3  GROUND_AMBIENT_SRGB = vec3(0.22, 0.20, 0.17);
const float AMBIENT_INTENSITY   = 0.4;
const float SPEC_POWER          = 32.0;
const float SPEC_SCALE          = 0.20;

const vec3  SKY_ZENITH_SRGB  = vec3(0.28, 0.48, 0.78);
const vec3  SKY_HORIZON_SRGB = vec3(0.55, 0.65, 0.78);
const vec3  SKY_GROUND_SRGB  = vec3(0.28, 0.25, 0.22);
const vec3  SUN_DISK_SRGB    = vec3(1.00, 0.97, 0.85);
const float SUN_DISK_BRIGHT  = 8.0;
const vec3  SUN_HALO_SRGB    = vec3(1.00, 0.92, 0.78);
const float SUN_HALO_BRIGHT  = 0.35;

const float FOG_DENSITY = 0.002;

vec3 SkyColor(vec3 rayDir, vec3 sunDir)
{
    float t = clamp(rayDir.y, -1.0, 1.0);
    vec3 skySrgb = (t >= 0.0)
        ? mix(SKY_HORIZON_SRGB, SKY_ZENITH_SRGB, pow(t, 1.2))
        : mix(SKY_HORIZON_SRGB, SKY_GROUND_SRGB, pow(-t, 0.50));
    vec3 sky = SrgbToLinear(skySrgb);

    float sunCos = max(dot(rayDir, sunDir), 0.0);
    sky += SrgbToLinear(SUN_DISK_SRGB) * SUN_DISK_BRIGHT * pow(sunCos, 800.0);
    sky += SrgbToLinear(SUN_HALO_SRGB) * SUN_HALO_BRIGHT * pow(sunCos, 8.0);
    return sky;
}

// Hemispheric ambient term for a +Y-up world. N is world-space normal.
vec3 HemisphericAmbient(vec3 N)
{
    float hemi = 0.5 + 0.5 * N.y;
    return AMBIENT_INTENSITY * mix(SrgbToLinear(GROUND_AMBIENT_SRGB),
                                    SrgbToLinear(SKY_AMBIENT_SRGB), hemi);
}

// TPDF (triangular) dither for 8-bit sRGB output. Two independent IGN samples
// give noise in [-1, +1] LSB; converting smooth gradients (sky, fog) from
// hard contour bands into high-frequency texture the eye averages out. Apply
// AFTER LinearToSrgb so quantization happens in display space.
vec3 DitherSrgb(vec3 c, vec2 fragCoord)
{
    const vec2 magic = vec2(0.06711056, 0.00583715);
    const float k = 52.9829189;
    float r1 = fract(k * fract(dot(fragCoord, magic)));
    float r2 = fract(k * fract(dot(fragCoord + 7.0, magic)));
    float t = r1 + r2 - 1.0;
    return c + vec3(t) * (1.0 / 255.0);
}

// Anti-aliased 1-pixel-wide world-space grid. uv is the tiled coordinate
// (e.g. worldXZ divided by the desired cell size). Returns ~1.0 on a line,
// ~0.0 in the cell interior; fwidth filtering keeps the line crisp at any
// distance and prevents the grid from aliasing on far receivers.
float WorldGrid(vec2 uv)
{
    vec2 grid = abs(fract(uv - 0.5) - 0.5);
    vec2 line = grid / fwidth(uv);
    return 1.0 - smoothstep(0.0, 1.5, min(line.x, line.y));
}
