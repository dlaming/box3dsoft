#version 330 core

#include "common.glsl"

out vec4 outColor;

in vec2 TexCoords;

uniform sampler2D gPosition;       // view-space position (xyz=0 for sky)
uniform sampler2D gNormal;         // view-space normal
uniform sampler2D gAlbedoSpec;     // sRGB albedo, a specular strength
uniform sampler2D ssao;
uniform sampler2DShadow shadowMap;

uniform mat4 lightSpaceMatrix;
uniform mat4 cameraMatrix;         // camera-to-world
uniform float worldTexelSize;      // world-space span of one shadow-map texel
uniform float lightZRange;         // lightFarPlane - lightNearPlane, in world units

// Sun + sky
uniform vec3 sunDirWorld;          // normalized, points from scene toward sun
uniform float tanHalfFovY;
uniform float aspectRatio;

uniform bool useSSAO;
uniform bool useShadow;
uniform bool useGrid;
uniform float aoPower;
uniform float aoDirect;
uniform bool aoOnly;

// Vogel disk (phyllotaxis spiral) sampling. Produces naturally well-distributed
// taps for any sample count, and the rotation phase is just an additive angle
// per tap — no rotation matrix needed.
const float GOLDEN_ANGLE = 2.39996323;
const int   PCF_TAP_COUNT = 32;
const float PCF_RADIUS_TEXELS = 3.0;

float ShadowCalculation(vec4 lightSpacePos, vec3 worldNormal, vec3 worldPos)
{
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    projCoords = projCoords * 0.5 + 0.5;

    // dFdx/dFdy must run before any early return so neighboring fragments in the
    // 2x2 quad always reach this point with valid projCoords values.
    vec3 ddxCoords = dFdx(projCoords);
    vec3 ddyCoords = dFdy(projCoords);

    if (projCoords.z > 1.0 || projCoords.z < 0.0) return 0.0;

    // Soft-fade near the shadow map boundary so geometry just outside the fitted
    // frustum doesn't produce a hard cutoff line. Falls off over the outer 6%
    // of the xy range and outer 10% of the depth range.
    vec2 edge = min(projCoords.xy, 1.0 - projCoords.xy);
    float xyFade = smoothstep(0.0, 0.06, min(edge.x, edge.y));
    float zFade  = 1.0 - smoothstep(0.90, 1.0, projCoords.z);
    float fade = xyFade * zFade;
    if (fade <= 0.0) return 0.0;

    // Receiver-plane depth bias: convert the screen-space gradient of projCoords.z
    // into a gradient with respect to shadow-map UV via the inverse Jacobian.
    // Each PCF tap then gets the depth the receiver surface would *actually* have
    // at that tap's offset, so per-texel acne disappears without lifting shadows
    // off contact via a large constant bias.
    //
    // Silhouette guard: at object edges, dFdx/dFdy mix samples from two unrelated
    // surfaces, producing a meaningless gradient that would over-bias the test
    // and paint a bright fringe along the silhouette. fwidth flags that case and
    // we fall back to a constant bias for those fragments.
    vec2 dzduv = vec2(0.0);
    // True silhouettes (adjacent pixels on different surfaces) have huge zWidth,
    // close to 1.0. Continuous surfaces even at grazing angles stay well below.
    float zWidth = abs(ddxCoords.z) + abs(ddyCoords.z);
    bool onSilhouette = zWidth > 0.3;
    if (!onSilhouette)
    {
        float det = ddxCoords.x * ddyCoords.y - ddxCoords.y * ddyCoords.x;
        if (abs(det) > 1e-8)
        {
            float invDet = 1.0 / det;
            dzduv.x = (ddyCoords.y * ddxCoords.z - ddxCoords.y * ddyCoords.z) * invDet;
            dzduv.y = (ddxCoords.x * ddyCoords.z - ddyCoords.x * ddxCoords.z) * invDet;
        }
        // Clamp loose enough to cover legitimate steep slopes (e.g. surfaces
        // nearly aligned with the light direction) but still finite.
        dzduv = clamp(dzduv, vec2(-2.0), vec2(2.0));
    }

    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);

    // Adapt PCF kernel radius to the screen pixel's footprint in shadow texels.
    // At far view distances each screen pixel covers many shadow texels; a
    // fixed 3-texel disk fits *inside* a single pixel, so adjacent pixels see
    // nearly the same texel set just rotated differently — that resonates with
    // the texel grid into the swirly moire visible on distant flat receivers.
    // Growing the kernel with the footprint makes the taps actually average
    // over the pixel's shadow-texel coverage, and as a bonus distant shadows
    // get the gentle softening they should naturally have. Clamped so very
    // distant geometry doesn't blow the kernel up to a useless smear.
    float footprintTexels = max(length(ddxCoords.xy), length(ddyCoords.xy)) / texelSize.x;
    float radiusTexels = clamp(footprintTexels, PCF_RADIUS_TEXELS, 8.0);
    vec2 sampleScale = radiusTexels * texelSize;

    // Hardware bilinear PCF samples a 2x2 texel neighborhood against a single
    // refZ, but the four stored depths differ by up to dzduv * 0.5 * texelSize.
    // Without absorbing that, half the bilinear neighbors pass and half fail,
    // producing the residual banding RPDB alone can't kill. Scaling the bias
    // with the receiver slope keeps adjacent texels on the same side of the
    // comparison.
    float slopeBias = (abs(dzduv.x) + abs(dzduv.y)) * texelSize.x;
    // World-unit bias floor: the existing constants are in normalized [0,1]
    // light-space depth, but the geometric scale that actually causes acne
    // (stacked-brick seams, hull-on-hull contacts) is in world units. A
    // 1-world-unit caster spans `1.0 / lightZRange` of normalized depth, so
    // any bias smaller than that lets self-shadow bands appear at every
    // brick interface. Floor it at ~half a world unit so the bias adapts
    // automatically to camera distance (which controls lightZRange).
    float minBias = 0.5 / max(lightZRange, 1.0);
    float bias = max(minBias, onSilhouette ? 0.003 : (slopeBias + 0.0005));

    // Per-pixel rotation phase. At extreme grazing angles each screen pixel
    // covers many shadow texels and rotation makes adjacent pixels sample
    // wildly different texel subsets — that resonates with the texel grid into
    // visible moire. Fade rotation out as the receiver becomes foreshortened.
    //
    // Use Interleaved Gradient Noise (Jimenez 2014) for the angle. The basic
    // fract(sin(dot(...))*43758.5453) hash has low-frequency structure that
    // shows through as concentric/swirl moire when each screen pixel covers
    // many shadow texels (distant view of a flat receiver). IGN was designed
    // for shader dithering and decorrelates much better at neighboring pixels.
    float rotStrength = 1.0 - smoothstep(0.05, 0.20, zWidth);
    float ign = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    float phi = ign * 6.2831853 * rotStrength;

    // Vogel-disk PCF: each tap is one rotation step further around the golden-
    // angle spiral, with radius growing as sqrt(i/N) so density is uniform
    // across the disk. sampler2DShadow + GL_LEQUAL returns 1.0 when refZ <=
    // storedDepth (lit), with hardware bilinear PCF inside each tap thanks to
    // GL_LINEAR filtering on the depth texture.
    float lit = 0.0;
    float invN = 1.0 / float(PCF_TAP_COUNT);
    for (int i = 0; i < PCF_TAP_COUNT; ++i)
    {
        float r = sqrt((float(i) + 0.5) * invN);
        float theta = float(i) * GOLDEN_ANGLE + phi;
        vec2 offset = vec2(cos(theta), sin(theta)) * r * sampleScale;
        float refZ = projCoords.z + dot(dzduv, offset) - bias;
        lit += texture(shadowMap, vec3(projCoords.xy + offset, refZ));
    }
    return (1.0 - lit * invN) * fade;
}

void main()
{
    vec2 ndc = TexCoords * 2.0 - 1.0;
    vec3 camRight   =  cameraMatrix[0].xyz;
    vec3 camUp      =  cameraMatrix[1].xyz;
    vec3 camForward = -cameraMatrix[2].xyz;
    vec3 viewDirWorld = normalize(
        camForward
        + ndc.x * tanHalfFovY * aspectRatio * camRight
        + ndc.y * tanHalfFovY * camUp
    );

    vec3 viewPos       = texture(gPosition, TexCoords).rgb;
    vec3 viewNormal    = texture(gNormal, TexCoords).rgb;
    vec3 albedoSrgb    = texture(gAlbedoSpec, TexCoords).rgb;
    float specStrength = texture(gAlbedoSpec, TexCoords).a;
    float ao = useSSAO ? texture(ssao, TexCoords).r : 1.0;

    if (viewPos.z == 0.0)
    {
        // AO-only debug still shows sky as sky so the geometry framing reads.
        vec3 sky = LinearToSrgb(ACESFilm(SkyColor(viewDirWorld, sunDirWorld)));
        outColor = vec4(DitherSrgb(sky, gl_FragCoord.xy), 1.0);
        return;
    }

    if (aoOnly)
    {
        // Bypass tonemap so AO of 1.0 reads as pure white.
        outColor = vec4(vec3(ao), 1.0);
        return;
    }

    vec3 baseColor   = SrgbToLinear(albedoSrgb);
    vec3 worldPos    = (cameraMatrix * vec4(viewPos, 1.0)).xyz;
    vec3 worldNormal = normalize(mat3(cameraMatrix) * viewNormal);

    // World-space grid on near-horizontal upward faces. Two tiers so distance
    // reads at any zoom: minor lines every 1 unit, major every 5. fwidth keeps
    // each line ~1 pixel wide regardless of camera distance, which both
    // avoids aliasing and produces a natural "fade with distance" since lines
    // sub-pixel-merge into the surface tint as they recede. The orientation
    // gate lets the same grid apply to stacked-cube tops without leaking onto
    // walls or sloped faces. Disabled per-sample on mesh-collision grounds
    // where the wireframe overlay already provides the same signal.
    if (useGrid && abs(worldPos.y) < 0.5)
    {
        float groundFactor = smoothstep(0.95, 0.99, worldNormal.y);
        if (groundFactor > 0.0)
        {
            float minor = WorldGrid(worldPos.xz);
            float major = WorldGrid(worldPos.xz * 0.2);
            float gridDarken = (1.0 - 0.15 * minor) * (1.0 - 0.40 * major);
            baseColor *= mix(1.0, gridDarken, groundFactor);
        }
    }

    float aoFactor = pow(ao, aoPower);

    vec3 ambient = HemisphericAmbient(worldNormal) * aoFactor;

    vec3 sunColor = SrgbToLinear(SUN_COLOR_SRGB) * SUN_INTENSITY;

    float NdotL = max(dot(worldNormal, sunDirWorld), 0.0);
    vec3 diffuse = NdotL * sunColor;

    vec3 viewDirToCam = -viewDirWorld;
    vec3 halfDir = normalize(sunDirWorld + viewDirToCam);
    float spec = pow(max(dot(worldNormal, halfDir), 0.0), SPEC_POWER);
    vec3 specular = spec * specStrength * SPEC_SCALE * sunColor;

    float shadow = 0.0;
    if (useShadow)
    {
        // Normal-offset bias: push the receiver point along the world normal
        // before projecting to light space. RPDB inside ShadowCalculation
        // handles per-tap bias well on flat receivers, but its Jacobian breaks
        // down at grazing angles where dzduv saturates the ±2 clamp. Offsetting
        // along N concentrates extra bias exactly on faces near-perpendicular
        // to the light (NdotL near 0), killing the residual acne RPDB misses.
        vec3 biasedWorldPos = worldPos + worldNormal * worldTexelSize * 1.5 * (1.0 - NdotL);
        vec4 lightSpacePos = lightSpaceMatrix * vec4(biasedWorldPos, 1.0);
        shadow = ShadowCalculation(lightSpacePos, worldNormal, worldPos);
    }
    
    // todo for disabling shadows
    // shadow = 0.0f;

    // Physically AO should only modulate indirect light, but direct sun overwhelms ambient on
    // sunlit surfaces and the contact darkening disappears. aoDirect dials in a controllable
    // amount of AO on the direct term — a fudge, but the only way to get visible contact AO
    // on a brightly lit floor without a separate contact-shadow pass.
    float directAO = mix(1.0, aoFactor, aoDirect);
    vec3 color = baseColor * (ambient + (1.0 - shadow) * (diffuse + specular) * directAO);

    float dist = length(viewPos);
    float fog = 1.0 - exp(-FOG_DENSITY * dist);
    color = mix(color, SkyColor(viewDirWorld, sunDirWorld), fog);

    outColor = vec4(DitherSrgb(LinearToSrgb(ACESFilm(color)), gl_FragCoord.xy), 1.0);
}
