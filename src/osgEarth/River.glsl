#pragma vp_name River Water - Vertex
#pragma vp_function oe_river_vertex, vertex_view
#pragma import_defines(OE_LOG_DEPTH_BUFFER)

in vec4 oe_river_flowAttr; // flow in texture units/s (xy), speed m/s, turbulence
in vec4 oe_river_dataAttr; // shore distance m, depth m, across, meters per texture unit

out vec2 oe_river_uv;
out vec2 oe_river_fall; // 1 where the water falls, meters fallen since the lip
out vec4 oe_river_flow;
out vec4 oe_river_data;
out vec3 oe_river_east;
out vec3 oe_river_north;
out vec3 oe_river_up;
out vec3 oe_river_viewPos;
flat out float oe_river_depthFar;

uniform float oe_river_depthBias;

vec3 vp_Normal; // stage global: view-space normal

void oe_river_vertex(inout vec4 vertexView)
{
    oe_river_uv = gl_MultiTexCoord0.xy;
    oe_river_fall = gl_MultiTexCoord1.xy;
    oe_river_flow = oe_river_flowAttr;
    oe_river_data = oe_river_dataAttr;

    // Tangent frame: tile-local x/y are east/north, z is up.
    oe_river_up = normalize(vp_Normal);
    vec3 east = gl_NormalMatrix * vec3(1.0, 0.0, 0.0);
    oe_river_north = normalize(cross(oe_river_up, east));
    oe_river_east = normalize(cross(oe_river_north, oe_river_up));
    oe_river_viewPos = vertexView.xyz;

#ifdef OE_LOG_DEPTH_BUFFER
    // The far distance exactly as the logarithmic depth shader computes it, so the
    // depth copy can be inverted consistently.
    vec4 farPoint = inverse(gl_ProjectionMatrix) * vec4(0.0, 0.0, 1.0, 1.0);
    oe_river_depthFar = -farPoint.z / farPoint.w;
#else
    oe_river_depthFar = 0.0;
#endif

    // Coarse terrain levels of detail can bulge a little above the water; pull the
    // surface toward the eye by an amount that grows with range like that error does.
    float range = length(vertexView.xyz);
    float bias = min(range * oe_river_depthBias, 40.0);
    vertexView.xyz *= max(1.0 - bias / max(range, 1.0), 0.0);
}


[break]
#pragma vp_name River Water - Surface
#pragma vp_function oe_river_fragment, fragment_coloring, 0.9
#pragma import_defines(OE_RIVER_SCREEN, OE_RIVER_SSR, OE_NUM_LIGHTS, OE_LOG_DEPTH_BUFFER)

#ifdef OE_NUM_LIGHTS
struct osg_LightSourceParameters
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    vec4 position;
    vec3 spotDirection;
    float spotExponent;
    float spotCutoff;
    float spotCosCutoff;
    float constantAttenuation;
    float linearAttenuation;
    float quadraticAttenuation;
    bool enabled;
};
uniform osg_LightSourceParameters osg_LightSource[OE_NUM_LIGHTS];
#endif

uniform sampler2D oe_river_waveTex;    // rg = normal xy, b = height, a = caustics
uniform sampler2D oe_river_foamTex;    // r = coarse foam, g = fine foam, b = low noise, a = mid noise
uniform sampler2D oe_river_bedTex;     // rgb = bed albedo (sRGB), a = stone height
uniform sampler2D oe_river_sceneColor; // copy of the opaque scene
uniform sampler2D oe_river_sceneDepth; // copy of its depth buffer
uniform vec4 oe_river_viewport;        // x, y, width, height of the copies in window pixels
uniform float oe_river_screenValid;    // 1 when the copies hold this camera's scene
uniform float oe_river_time;
uniform vec4 oe_river_p0;              // flow speed, wave strength, wave scale, clarity
uniform vec4 oe_river_p1;              // turbidity, foam, shore foam, rapids foam
uniform vec4 oe_river_p2;              // reflection, refraction, caustics, sun glint
uniform vec4 oe_river_p3;              // edge softness, debug view, ssr, max range
uniform vec4 oe_river_p4;              // wet banks
uniform vec4 oe_river_freq;            // wave octave frequencies (repeats per texture unit), foam
uniform vec4 oe_river_freq2;           // caustics, patch noise, bed stones
uniform vec3 oe_river_deepColor;       // linear
uniform vec3 oe_river_sedimentColor;   // linear
uniform vec3 oe_river_skyColor;        // linear
uniform mat4 osg_ProjectionMatrix;

in vec2 oe_river_uv;
in vec2 oe_river_fall;
in vec4 oe_river_flow;
in vec4 oe_river_data;
in vec3 oe_river_east;
in vec3 oe_river_north;
in vec3 oe_river_up;
in vec3 oe_river_viewPos;
flat in float oe_river_depthFar;

vec3 vp_Normal;
struct OE_PBR { float displacement, roughness, ao, metal; } oe_pbr;

// Handed to the composite stage, which runs after the scene's lighting model.
vec3 oe_river_N;          // wave normal (view space)
vec3 oe_river_V;          // direction to the eye
vec3 oe_river_U;          // local up
vec3 oe_river_reflection; // linear radiance the surface mirrors
vec3 oe_river_seen;       // linear scene color seen through the surface (refracted)
vec3 oe_river_behind;     // linear scene color straight behind the surface
vec3 oe_river_bed;        // linear albedo of the procedural river bed
vec3 oe_river_T;          // transmittance along the view path through the water
vec3 oe_river_scatter;    // linear albedo of light scattered back by the water body
vec4 oe_river_debug;      // rgb = debug color, a = 1 when a debug view is active
float oe_river_bedMix;    // weight of the procedural bed over the scene color
float oe_river_caustic;
float oe_river_F;
float oe_river_foam;
float oe_river_alpha;
float oe_river_glint;
float oe_river_roughness;
float oe_river_wet;       // wet bank fragment: wetness 0..1; water fragment: -1
float oe_river_sheet;     // 1 where the water falls

// The water hands the scene's lighting model a neutral gray "probe" and reads back
// how much light it received, so whatever sky is active lights the water too.
const float OE_RIVER_PROBE = 0.18;

vec3 oe_river_linear(vec3 c) { return pow(max(c, vec3(0.0)), vec3(2.2)); }
vec3 oe_river_gamma(vec3 c) { return pow(max(c, vec3(0.0)), vec3(1.0 / 2.2)); }

// Distance along -z of a depth buffer value (perspective projection, standard or
// osgEarth's logarithmic depth buffer).
float oe_river_eyeDepth(float d)
{
#if defined(OE_LOG_DEPTH_BUFFER) && OE_LOG_DEPTH_BUFFER == 2
    // gl_FragDepth = log(w*C + 1) / log(far*C + 1)
    const float C = 0.001;
    return (pow(oe_river_depthFar * C + 1.0, d) - 1.0) / C;
#elif defined(OE_LOG_DEPTH_BUFFER)
    // vertex-only variant: depth = log2(w + 1) / log2(far + 1)
    return pow(oe_river_depthFar + 1.0, d) - 1.0;
#else
    return osg_ProjectionMatrix[3][2] / (d * 2.0 - 1.0 + osg_ProjectionMatrix[2][2]);
#endif
}

// View-space position of a depth buffer sample.
vec3 oe_river_viewFromDepth(vec2 uv, float d)
{
    mat4 P = osg_ProjectionMatrix;
    float vz = -oe_river_eyeDepth(d);
    vec2 ndc = uv * 2.0 - 1.0;
    return vec3(-vz * (ndc.x + P[2][0]) / P[0][0], -vz * (ndc.y + P[2][1]) / P[1][1], vz);
}

// Screen coordinates of a view-space point; z <= 0 means behind the eye.
vec3 oe_river_project(vec3 v)
{
    vec4 c = osg_ProjectionMatrix * vec4(v, 1.0);
    return vec3(c.xy / max(c.w, 1e-6) * 0.5 + 0.5, c.w);
}

// Rotates a texture-space vector back into the east/north frame for an integer
// rotation-scale matrix [[a, b], [-b, a]] used to decorrelate wave octaves.
vec2 oe_river_unrotate(vec2 n, float a, float b)
{
    return vec2(a * n.x - b * n.y, b * n.x + a * n.y) / sqrt(a * a + b * b);
}

// Sky reflection when nothing on screen can supply one.
vec3 oe_river_sky(vec3 R, vec3 up, float daylight)
{
    float e = clamp(dot(R, up), 0.0, 1.0);
    vec3 horizon = oe_river_skyColor;
    vec3 zenith = oe_river_skyColor * vec3(0.55, 0.68, 0.95);
    return mix(horizon, zenith, sqrt(e)) * daylight;
}

void oe_river_fragment(inout vec4 color)
{
    const float PI = 3.14159265;

    float flowScale = oe_river_p0.x;
    float waveStrength = oe_river_p0.y;
    float clarity = max(oe_river_p0.w, 0.02);
    float turbidity = clamp(oe_river_p1.x, 0.0, 1.0);
    float foamAmount = oe_river_p1.y;
    float shoreFoam = oe_river_p1.z;
    float rapidsFoam = oe_river_p1.w;
    float reflectionStrength = oe_river_p2.x;
    float refractionStrength = oe_river_p2.y;
    float edgeSoftness = max(oe_river_p3.x, 0.01);
    float maxRange = max(oe_river_p3.w, 1.0);

    vec3 up = normalize(oe_river_up);
    vec3 east = normalize(oe_river_east);
    vec3 north = normalize(oe_river_north);
    vec3 V = normalize(-oe_river_viewPos);
    float dist = length(oe_river_viewPos);

    vec2 flowUV = oe_river_flow.xy * flowScale;
    float speed = oe_river_flow.z * flowScale;
    float turbulence = oe_river_flow.w;
    float shoreDistance = oe_river_data.x;
    float designDepth = oe_river_data.y;
    float metersPerUV = max(oe_river_data.w, 0.01);
    vec2 uv = oe_river_uv;
    float t = oe_river_time;
    float fall = clamp(oe_river_fall.x, 0.0, 1.0);
    float fallen = max(oe_river_fall.y, 0.0);

    // ---- Wet bank: a band floating a little above the water along the shore. Ground that
    // shows through it lies near the water level and is darkened by moisture; ground below
    // the water level is river bed seen through the water, which the water itself draws.
    oe_river_wet = -1.0;
    oe_river_sheet = 0.0;
    if (shoreDistance < 0.0)
    {
        float wet = 0.0;
        vec3 ground = vec3(0.0);
#ifdef OE_RIVER_SCREEN
        if (oe_river_screenValid > 0.5)
        {
            vec2 buv = (gl_FragCoord.xy - oe_river_viewport.xy) / oe_river_viewport.zw;
            float d = texture(oe_river_sceneDepth, buv).r;
            float lift = max(oe_river_data.y, 0.05);
            // height of the visible ground above the water surface
            float height = lift - dot(oe_river_viewPos - oe_river_viewFromDepth(buv, d), up);
            wet = (1.0 - smoothstep(0.15 * lift, lift, height)) * step(-0.05, height) * step(d, 0.999999)
                * (1.0 - smoothstep(0.5, 1.0, oe_river_data.z)) * clamp(oe_river_p4.x, 0.0, 2.0);
            ground = oe_river_linear(texture(oe_river_sceneColor, buv).rgb);
        }
#endif
        int bankView = int(oe_river_p3.y + 0.5);
        if (wet < 0.004 || (bankView != 0 && bankView != 2))
            discard;

        float bankDaylight = 1.0;
#ifdef OE_NUM_LIGHTS
        if (osg_LightSource[0].enabled)
            bankDaylight = mix(0.02, 1.0, smoothstep(-0.1, 0.25, dot(normalize(osg_LightSource[0].position.xyz), up)));
#endif
        color = vec4(oe_river_gamma(vec3(OE_RIVER_PROBE)), 1.0);
        vp_Normal = up;
        oe_pbr.roughness = 1.0;
        oe_pbr.metal = 0.0;
        oe_pbr.ao = 1.0;
        oe_river_wet = min(wet, 1.0);
        oe_river_behind = ground;
        oe_river_N = up;
        oe_river_V = V;
        oe_river_U = up;
        oe_river_reflection = oe_river_sky(reflect(-V, up), up, bankDaylight) * oe_river_p2.x;
        // debug: the water-depth view shows wetness in orange
        oe_river_debug = bankView == 2 ? vec4(oe_river_linear(vec3(1.0, 0.55, 0.1)) * oe_river_wet, 1.0) : vec4(0.0);
        return;
    }

    // ---- Flow: two copies of each texture slide along the current half a cycle apart
    // and cross-fade, so the surface moves without ever stretching. A noise offset
    // de-synchronizes the cycle across the river to hide the cross-fade.
    vec4 noise = texture(oe_river_foamTex, uv * oe_river_freq2.y);
    const float cycle = 1.7;
    float phase = t / cycle + noise.b * 1.7;
    float p0 = fract(phase);
    float p1 = fract(phase + 0.5);
    float w0 = 1.0 - abs(2.0 * p0 - 1.0);
    float still = 1.0 - smoothstep(0.03, 0.2, speed);
    vec2 drift = vec2(0.045, 0.027) * t * still; // breeze on still water
    vec2 off0 = flowUV * (p0 * cycle) - drift;
    vec2 off1 = flowUV * (p1 * cycle) - drift;
    vec2 jitter = vec2(0.371, 0.613);

    // ---- Waves: three octaves (the second and third rotated by integer matrices so
    // they stay seamless across tiles), each sampled in both phases.
    vec2 a0 = (uv - off0) * oe_river_freq.x;
    vec2 a1 = (uv - off1) * oe_river_freq.x + jitter;
    vec2 nA = mix(texture(oe_river_waveTex, a1).rg, texture(oe_river_waveTex, a0).rg, w0) * 2.0 - 1.0;

    vec2 b0 = (uv - off0) * oe_river_freq.y;
    b0 = vec2(3.0 * b0.x + 4.0 * b0.y, -4.0 * b0.x + 3.0 * b0.y);
    vec2 b1 = (uv - off1) * oe_river_freq.y;
    b1 = vec2(3.0 * b1.x + 4.0 * b1.y, -4.0 * b1.x + 3.0 * b1.y) + jitter;
    vec2 nB = mix(texture(oe_river_waveTex, b1).rg, texture(oe_river_waveTex, b0).rg, w0) * 2.0 - 1.0;
    nB = oe_river_unrotate(nB, 3.0, 4.0);

    vec2 c0 = (uv - off0 * 1.15) * oe_river_freq.z;
    c0 = vec2(c0.x - 2.0 * c0.y, 2.0 * c0.x + c0.y);
    vec2 c1 = (uv - off1 * 1.15) * oe_river_freq.z;
    c1 = vec2(c1.x - 2.0 * c1.y, 2.0 * c1.x + c1.y) + jitter.yx;
    vec2 nC = mix(texture(oe_river_waveTex, c1).rg, texture(oe_river_waveTex, c0).rg, w0) * 2.0 - 1.0;
    nC = oe_river_unrotate(nC, 1.0, -2.0);

    // Faster and steeper water is choppier; still water is a soft ripple.
    float chop = mix(0.55, 1.25, smoothstep(0.15, 2.2, speed)) + turbulence * 0.9;
    chop = mix(chop, 0.4, still);
    float fineFade = 1.0 - smoothstep(40.0, 400.0, dist);
    vec2 slope = (nA * 0.45 + nB * 0.55 + nC * 0.35 * fineFade) * chop * waveStrength;

    // A current combs its ripples into streaks: suppress slopes along the flow and boost
    // them across it, so highlights stretch downstream.
    vec2 fdir = speed > 0.03 ? normalize(flowUV) : vec2(1.0, 0.0);
    float aniso = mix(1.0, 0.3, smoothstep(0.08, 0.9, speed) * (1.0 - 0.5 * turbulence));
    float along = dot(slope, fdir);
    slope = fdir * (along * aniso) + (slope - fdir * along) * (1.0 + 0.5 * (1.0 - aniso));
    vec2 calm = (nA * 0.45 + nB * 0.4) * chop * waveStrength;
    float calmAlong = dot(calm, fdir);
    calm = fdir * (calmAlong * aniso) + (calm - fdir * calmAlong) * (1.0 + 0.5 * (1.0 - aniso));

    vec3 N = normalize(up + east * slope.x + north * slope.y);
    vec3 Ng = normalize(up + east * calm.x + north * calm.y); // glint normal: no fine chop

    // Falling water faces out from the rock: shade it by the sheet's own orientation (from the
    // screen-space derivatives of its position) rather than by the horizontal frame.
    vec3 sheet = cross(dFdx(oe_river_viewPos), dFdy(oe_river_viewPos));
    sheet = dot(sheet, sheet) > 1e-20 ? normalize(sheet) : up;
    if (dot(sheet, V) < 0.0)
        sheet = -sheet;
    N = normalize(mix(N, normalize(sheet + (east * slope.x + north * slope.y) * 0.5), fall));
    Ng = normalize(mix(Ng, sheet, fall));

    // Keep the normal facing the eye so grazing views don't light the back of a wave.
    float NdotV = dot(N, V);
    if (NdotV < 0.02)
    {
        N = normalize(N + V * (0.02 - NdotV));
        NdotV = 0.02;
    }

    // ---- Foam lace, advected with the flow
    vec2 f0 = (uv - off0) * oe_river_freq.w;
    vec2 f1 = (uv - off1) * oe_river_freq.w + jitter;
    vec4 lace4 = mix(texture(oe_river_foamTex, f1), texture(oe_river_foamTex, f0), w0);
    float lace = lace4.r * 0.65 + lace4.g * 0.55 * (0.6 + 0.4 * noise.a);
    // Whitewater trails downstream: smear the lace along the current.
    vec2 trail = fdir * (0.7 * oe_river_freq.w);
    float trailA = mix(texture(oe_river_foamTex, f1 - trail).r, texture(oe_river_foamTex, f0 - trail).r, w0);
    float trailB = mix(texture(oe_river_foamTex, f1 - 2.0 * trail).r, texture(oe_river_foamTex, f0 - 2.0 * trail).r, w0);
    float streak = max(lace, max(trailA * 0.85, trailB * 0.7));

    // ---- What lies under the surface
    vec3 behind = vec3(0.0);
    vec3 seen = vec3(0.0);
    float vDepth = designDepth;  // vertical water depth
    float thickness = designDepth / max(dot(V, up), 0.1); // view-ray path through the water
    float edge = smoothstep(0.0, 0.6, shoreDistance);
    vec3 bedView = oe_river_viewPos - up * vDepth;
    bool screen = false;

#ifdef OE_RIVER_SCREEN
    screen = oe_river_screenValid > 0.5;
#endif

    mat4 P = osg_ProjectionMatrix;
    float waterEye = -oe_river_viewPos.z;
    vec2 suv = (gl_FragCoord.xy - oe_river_viewport.xy) / oe_river_viewport.zw;

    if (screen)
    {
        float d = texture(oe_river_sceneDepth, suv).r;
        if (d >= 0.999999)
        {
            // Nothing behind the surface: a grazing view past the horizon, or a hairline crack
            // between terrain tiles under the water. A crack has ground right beside it; borrow it.
            for (int i = 0; i < 8; ++i)
            {
                float a = float(i) * 0.7853982;
                vec2 nuv = suv + vec2(cos(a), sin(a)) * 4.0 / oe_river_viewport.zw;
                float nd = texture(oe_river_sceneDepth, nuv).r;
                if (nd < 0.999999)
                {
                    suv = nuv;
                    d = nd;
                    break;
                }
            }
        }
        vec3 sceneView = oe_river_viewFromDepth(suv, d);
        behind = oe_river_linear(texture(oe_river_sceneColor, suv).rgb);

        if (-sceneView.z > waterEye && d < 0.999999)
        {
            vDepth = max(dot(oe_river_viewPos - sceneView, up), 0.0);
            thickness = length(sceneView - oe_river_viewPos);
            edge = smoothstep(0.0, edgeSoftness, vDepth);
            bedView = sceneView;
        }
        else if (d >= 0.999999)
        {
            // nothing behind the water (grazing view over the horizon): treat as deep
            vDepth = max(designDepth, 2.0);
            thickness = 30.0;
            edge = 1.0;
        }

        // Refraction: displace the lookup by the wave slope, scaled by how much water the
        // light crosses, and refuse samples from things in front of the water.
        vec3 tilt = N - up;
        vec2 shift = tilt.xy * (refractionStrength * 0.9 * min(thickness, 2.5));
        vec2 ruv = suv + shift * vec2(P[0][0], P[1][1]) * 0.5 / max(waterEye, 0.5);
        float rd = texture(oe_river_sceneDepth, ruv).r;
        if (oe_river_eyeDepth(rd) < waterEye || any(lessThan(ruv, vec2(0.0))) || any(greaterThan(ruv, vec2(1.0))))
        {
            ruv = suv;
            rd = d;
        }
        seen = oe_river_linear(texture(oe_river_sceneColor, ruv).rgb);
    }

    // ---- Absorption and scattering. Pure water absorbs red fastest; sediment scatters
    // all wavelengths and shortens visibility.
    vec3 sigmaWater = vec3(0.38, 0.075, 0.048);
    vec3 sigmaSediment = vec3(1.0, 0.86, 0.68) * (turbidity * 1.4 + 0.02);
    vec3 sigma = (sigmaWater + sigmaSediment) / clarity;
    vec3 T = exp(-sigma * thickness);
    // A falling sheet is thick, churning water: it hides the rock behind it.
    T *= 1.0 - 0.85 * fall;
    vec3 scatterColor = mix(oe_river_deepColor, oe_river_sedimentColor, turbidity);

    // ---- The bed: rounded stones and gravel under clear water, silt under turbid water,
    // measured in bed-plane texture units so it stays put as the surface moves.
    // The bed point comes from the unrefracted depth (smooth across the surface); the waves
    // then bend the view across the bed in proportion to the depth. Mip levels follow the
    // smooth surface coordinates so the stones never shimmer.
    vec3 rel = bedView - oe_river_viewPos;
    vec2 bendUV = vec2(dot(N, east), dot(N, north)) * (refractionStrength * 0.35 * min(vDepth, 2.0));
    vec2 bedUV = uv + (vec2(dot(rel, east), dot(rel, north)) + bendUV) / metersPerUV;
    vec2 bedScale = vec2(oe_river_freq2.z);
    vec4 bedSample = textureGrad(oe_river_bedTex, bedUV * bedScale, dFdx(uv) * bedScale, dFdy(uv) * bedScale);
    vec3 bedAlbedo = oe_river_linear(bedSample.rgb);
    bedAlbedo = mix(bedAlbedo, vec3(0.11, 0.095, 0.07), turbidity * 0.75);
    // The procedural bed takes over from the scene's own colors where the water is deep
    // enough to hide the terrain imagery and near enough for stones to matter.
    float bedMix = screen ? smoothstep(0.04, 0.4, vDepth) * (1.0 - smoothstep(90.0, 500.0, dist)) : 1.0;

    // ---- Caustics on the bed
    vec2 k0 = bedUV * oe_river_freq2.x + vec2(0.021, 0.013) * t + flowUV * t * oe_river_freq2.x * 0.35;
    vec2 k1 = bedUV * oe_river_freq2.x;
    k1 = vec2(2.0 * k1.x + k1.y, -k1.x + 2.0 * k1.y) * 0.5 - vec2(0.017, -0.025) * t;
    vec2 kdx = dFdx(uv) * oe_river_freq2.x, kdy = dFdy(uv) * oe_river_freq2.x;
    float caustic = min(textureGrad(oe_river_waveTex, k0, kdx, kdy).a, textureGrad(oe_river_waveTex, k1, kdx, kdy).a);
    caustic = pow(caustic, 1.6) * 3.0;
    float causticFade = exp(-vDepth * 0.4) * smoothstep(0.03, 0.3, vDepth) * (1.0 - smoothstep(60.0, 300.0, dist));
    causticFade *= 1.0 - fall;

    // A falling sheet hugs the rock; the waterline fade must not thin it out, though a ribbon
    // of falling water frays at its edges.
    edge = max(edge, fall * smoothstep(0.0, 2.0, shoreDistance));

    // ---- Foam: whitewater on steep, fast reaches; churn where the water shallows against
    // banks, rocks and piers.
    float rapids = turbulence * rapidsFoam * clamp(0.2 + 1.1 * noise.a * noise.a + 0.3 * noise.b, 0.0, 1.2);
    float shallow = 1.0 - smoothstep(0.0, 0.25 + 0.55 * min(speed, 2.0), vDepth);
    float churn = shallow * shoreFoam * (0.25 + 0.75 * smoothstep(0.1, 1.2, speed));
    // far away the lace averages out; keep distant whitewater from reading as solid paint
    float distanceFade = 1.0 - 0.6 * smoothstep(150.0, 1500.0, dist);
    float rapidsCover = clamp(rapids * 0.9 * foamAmount * distanceFade, 0.0, 1.0);
    float churnCover = clamp(churn * foamAmount, 0.0, 1.0);
    float foamRapids = smoothstep(1.0 - rapidsCover, 1.0 - rapidsCover + 0.25, streak) * smoothstep(0.0, 0.15, rapidsCover);
    float foamChurn = smoothstep(1.0 - churnCover, 1.0 - churnCover + 0.22, lace) * smoothstep(0.0, 0.15, churnCover);

    // ---- Falling water: streaks of aerated water stretched down the drop and racing down it
    // (two layers at different scales and speeds). Right at the lip the sheet is still glassy;
    // it whitens as it falls and breaks up into air.
    vec2 fallDir = speed > 0.03 ? normalize(flowUV) : vec2(0.0, 1.0);
    float fallU = dot(uv, vec2(-fallDir.y, fallDir.x)) * metersPerUV; // meters across the fall
    vec4 sheetA = texture(oe_river_foamTex, vec2(fallU / 3.1, fallen / 9.0 - fract(t * 0.6667)));
    vec4 sheetB = texture(oe_river_foamTex, vec2(fallU / 1.3 + 0.37, fallen / 4.2 - fract(t * 2.2619)));
    float streaks = max(sheetA.r, sheetB.r * 0.9) * (0.75 + 0.5 * sheetB.g);
    float aerated = mix(0.35, 0.9, smoothstep(0.5, 12.0, fallen)) * clamp(rapidsFoam * foamAmount, 0.0, 1.5);
    float foamFall = smoothstep(1.0 - aerated, 1.0 - aerated + 0.18, streaks);
    // Far away the streaks blur into a slightly translucent white, with denser and thinner
    // surges sliding down the fall.
    float surge = texture(oe_river_foamTex, vec2(fallU / 25.0, fallen / 60.0 - fract(t * 0.12))).b;
    foamFall = mix(foamFall, min(aerated, 0.82) * (0.6 + 0.6 * surge), smoothstep(150.0, 900.0, dist)) * fall;

    float foam = max(max(foamRapids, foamChurn) * edge, foamFall);

    // ---- Reflection: screen-space trace, then the scene at infinity, then the sky.
    vec3 R = reflect(-V, N);
    float rup = dot(R, up);
    if (rup < 0.01) R = normalize(R + up * (0.01 - rup));
    float F = 0.02 + 0.98 * pow(1.0 - clamp(NdotV, 0.0, 1.0), 5.0);
    float roughness = clamp(0.06 + 0.12 * chop + dist * 0.00025, 0.04, 0.5);
    float daylight = 1.0;
#ifdef OE_NUM_LIGHTS
    if (osg_LightSource[0].enabled)
        daylight = mix(0.02, 1.0, smoothstep(-0.1, 0.25, dot(normalize(osg_LightSource[0].position.xyz), up)));
#endif
    vec3 refl = oe_river_sky(R, up, daylight);

    if (screen)
    {
        float lod = roughness * 6.0;

        // Distant scenery and sky: where the reflected direction meets the screen.
        vec3 inf = oe_river_project(R * 1e5);
        vec2 top = vec2(clamp(suv.x + (inf.x - suv.x) * 0.25, 0.02, 0.98), 0.985);
        if (texture(oe_river_sceneDepth, top).r >= 0.999999)
        {
            // the top of the screen shows sky: use it, deepening toward the zenith
            vec3 skyTop = oe_river_linear(textureLod(oe_river_sceneColor, top, 4.0).rgb);
            refl = mix(skyTop, skyTop * vec3(0.7, 0.8, 0.95), smoothstep(0.3, 1.0, rup));
        }
        if (inf.z > 0.0)
        {
            vec2 e = min(inf.xy, 1.0 - inf.xy);
            float onScreen = smoothstep(0.0, 0.05, min(e.x, e.y));
            if (onScreen > 0.0)
            {
                float id = texture(oe_river_sceneDepth, inf.xy).r;
                float far = id >= 0.999999 ? 1.0 : smoothstep(400.0, 3000.0, oe_river_eyeDepth(id));
                refl = mix(refl, oe_river_linear(textureLod(oe_river_sceneColor, inf.xy, lod).rgb), onScreen * far);
            }
        }

#ifdef OE_RIVER_SSR
        if (oe_river_p3.z > 0.5)
        {
            // Nearby banks, trees and buildings: march the reflected ray against the depth copy.
            vec3 pos = oe_river_viewPos + up * 0.05;
            float stepLength = max(0.35, dist * 0.012);
            vec3 prev = pos;
            for (int i = 0; i < 32; ++i)
            {
                prev = pos;
                pos += R * stepLength;
                stepLength *= 1.17;
                vec3 s = oe_river_project(pos);
                if (s.z <= 0.0 || s.x < 0.0 || s.y < 0.0 || s.x > 1.0 || s.y > 1.0)
                    break;
                float sd = texture(oe_river_sceneDepth, s.xy).r;
                if (sd >= 0.999999)
                    continue;
                float behindBy = -pos.z - oe_river_eyeDepth(sd);
                if (behindBy > 0.0 && behindBy < stepLength * 3.0 + 0.5)
                {
                    // refine between the last two samples
                    vec3 lo = prev, hi = pos;
                    for (int j = 0; j < 5; ++j)
                    {
                        vec3 m = (lo + hi) * 0.5;
                        vec3 ms = oe_river_project(m);
                        float md = oe_river_eyeDepth(texture(oe_river_sceneDepth, ms.xy).r);
                        if (-m.z > md) hi = m; else lo = m;
                    }
                    vec3 hs = oe_river_project(hi);
                    vec2 e = min(hs.xy, 1.0 - hs.xy);
                    float fade = smoothstep(0.0, 0.07, min(e.x, e.y)) * (1.0 - smoothstep(18.0, 32.0, float(i)));
                    vec3 hitColor = oe_river_linear(textureLod(oe_river_sceneColor, hs.xy, lod * 0.6).rgb);
                    refl = mix(refl, hitColor, fade);
                    break;
                }
            }
        }
#endif
    }

    // ---- Hand the probe to the scene's lighting; the composite stage builds the water.
    color = vec4(oe_river_gamma(vec3(OE_RIVER_PROBE)), 1.0);
    vp_Normal = up;
    oe_pbr.roughness = 1.0;
    oe_pbr.metal = 0.0;
    oe_pbr.ao = 1.0;

    oe_river_N = Ng;
    oe_river_V = V;
    oe_river_U = up;
    oe_river_reflection = refl * reflectionStrength;
    oe_river_seen = seen;
    oe_river_behind = behind;
    oe_river_bed = bedAlbedo;
    oe_river_T = T;
    oe_river_scatter = scatterColor;
    oe_river_bedMix = bedMix;
    oe_river_caustic = caustic * causticFade * oe_river_p2.z;
    oe_river_F = clamp(F, 0.0, 1.0);
    oe_river_foam = foam;
    // Without a scene copy there is nothing to fade into, so the edges stay hard.
    oe_river_alpha = screen ? edge * (1.0 - smoothstep(0.85 * maxRange, maxRange, dist)) : 1.0;
    oe_river_glint = oe_river_p2.w * (1.0 - foam) * (1.0 - fall);
    oe_river_sheet = fall;
    oe_river_roughness = roughness;

    // Debug views
    int view = int(oe_river_p3.y + 0.5);
    oe_river_debug = vec4(0.0);
    if (view == 1)
    {
        // flow: hue = direction, brightness = speed
        vec2 f = flowUV * metersPerUV;
        float ang = atan(f.y, f.x) / (2.0 * PI) + 0.5;
        vec3 hue = clamp(abs(mod(ang * 6.0 + vec3(0.0, 4.0, 2.0), 6.0) - 3.0) - 1.0, 0.0, 1.0);
        oe_river_debug = vec4(oe_river_linear(hue * clamp(length(f) / 2.0, 0.1, 1.0)), 1.0);
    }
    else if (view == 2) // vertical water depth: 0 m white, 4 m blue
        oe_river_debug = vec4(oe_river_linear(mix(vec3(1.0), vec3(0.0, 0.15, 0.6), clamp(vDepth / 4.0, 0.0, 1.0))), 1.0);
    else if (view == 3) // foam coverage
        oe_river_debug = vec4(vec3(foam), 1.0);
    else if (view == 4) // fresnel
        oe_river_debug = vec4(vec3(F), 1.0);
    else if (view == 5) // green = scene copy in use, red = not; blue = waterline fade
        oe_river_debug = vec4(screen ? 0.0 : 1.0, screen ? 1.0 : 0.0, oe_river_alpha, 1.0);
    else if (view == 6) // what the refraction sees
        oe_river_debug = vec4(seen, 1.0);
    else if (view == 7) // what the reflection sees
        oe_river_debug = vec4(refl, 1.0);
    else if (view == 8) // wave normal (view space)
        oe_river_debug = vec4(oe_river_linear(N * 0.5 + 0.5), 1.0);
    else if (view == 9) // bed albedo
        oe_river_debug = vec4(bedAlbedo, 1.0);
    else if (view == 10) // falling water: red = falling, green = its streaks, blue = height fallen (0-50 m)
        oe_river_debug = vec4(fall, foamFall, clamp(fallen / 50.0, 0.0, 1.0), 1.0);
}


[break]
#pragma vp_name River Water - Composite
#pragma vp_function oe_river_composite, fragment_lighting, 0.9
#pragma import_defines(OE_NUM_LIGHTS, OE_SHADOWING)

vec3 oe_river_N;
vec3 oe_river_V;
vec3 oe_river_U;
vec3 oe_river_reflection;
vec3 oe_river_seen;
vec3 oe_river_behind;
vec3 oe_river_bed;
vec3 oe_river_T;
vec3 oe_river_scatter;
vec4 oe_river_debug;
float oe_river_bedMix;
float oe_river_caustic;
float oe_river_F;
float oe_river_foam;
float oe_river_alpha;
float oe_river_glint;
float oe_river_roughness;
float oe_river_wet;
float oe_river_sheet;

#ifdef OE_NUM_LIGHTS
struct osg_LightSourceParameters
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    vec4 position;
    vec3 spotDirection;
    float spotExponent;
    float spotCutoff;
    float spotCosCutoff;
    float constantAttenuation;
    float linearAttenuation;
    float quadraticAttenuation;
    bool enabled;
};
uniform osg_LightSourceParameters osg_LightSource[OE_NUM_LIGHTS];
#endif

#ifdef OE_SHADOWING
float oe_shadow_visibility;
#endif

vec3 oe_river_linear(vec3 c);
vec3 oe_river_gamma(vec3 c);

void oe_river_composite(inout vec4 color)
{
    const float PI = 3.14159265;
    const float PROBE = 0.18;

    // Light received by a horizontal surface here, as the scene's lighting model sees it
    // (sun, sky, shadows, exposure).
    vec3 E = oe_river_linear(color.rgb) / PROBE;

    // Sun direction, color and visibility.
    vec3 L = oe_river_U;
    vec3 sunColor = vec3(0.0);
    float sun = 0.0;
#ifdef OE_NUM_LIGHTS
    if (osg_LightSource[0].enabled)
    {
        L = normalize(osg_LightSource[0].position.xyz);
        sunColor = osg_LightSource[0].diffuse.rgb;
        sun = smoothstep(-0.02, 0.1, dot(L, oe_river_U));
    }
#endif
#ifdef OE_SHADOWING
    sun *= oe_shadow_visibility;
#endif

    if (oe_river_wet >= 0.0)
    {
        // Wet ground: water fills the pores (darker, a little more saturated) and a thin
        // film mirrors some sky and sun.
        float w = clamp(oe_river_wet, 0.0, 1.0);
        vec3 ground = oe_river_behind * mix(1.0, 0.58, w);
        float luma = dot(ground, vec3(0.2126, 0.7152, 0.0722));
        ground = max(mix(vec3(luma), ground, 1.0 + 0.3 * w), vec3(0.0));
        float film = 0.02 + 0.98 * pow(1.0 - clamp(dot(oe_river_U, oe_river_V), 0.0, 1.0), 5.0);
        vec3 Hw = normalize(L + oe_river_V);
        float sheen = pow(max(dot(oe_river_U, Hw), 0.0), 48.0) * 0.35;
        vec3 wetResult = ground + (oe_river_reflection * 0.5 + sunColor * sheen * sun) * film * w;
        color.rgb = oe_river_gamma(oe_river_debug.a > 0.5 ? oe_river_debug.rgb : wetResult);
        color.a = 1.0;
        return;
    }

    // Under the surface: the bed (procedural near, the scene's own colors far or at the
    // waterline) brightened by caustics, fading into the water body's scattered light.
    float caustics = oe_river_caustic * sun;
    vec3 bed = mix(oe_river_seen * (1.0 + caustics), oe_river_bed * E * (1.0 + 2.5 * caustics), oe_river_bedMix);
    vec3 body = oe_river_scatter * E;
    vec3 under = mix(body, bed, oe_river_T);
    vec3 water = mix(under, oe_river_reflection, oe_river_F);

    // Foam is lit like any other bright diffuse surface. A falling sheet stands on edge: it
    // takes the sun by its own orientation rather than as the horizontal surface E describes.
    // (Aerated water scatters light through itself, so even the shaded side stays bright.)
    float facing = mix(1.0, (0.55 + 0.45 * max(dot(oe_river_N, L), 0.0)) / (0.55 + 0.45 * max(dot(oe_river_U, L), 0.0)),
        clamp(oe_river_sheet, 0.0, 1.0) * sun);
    vec3 result = mix(water, vec3(0.82, 0.86, 0.88) * E * facing, oe_river_foam);

    // Sun glint: a sharp GGX lobe on the wave normal.
    vec3 N = oe_river_N, V = oe_river_V;
    vec3 H = normalize(L + V);
    float a = oe_river_roughness * oe_river_roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float denom = NdotH * NdotH * (a2 - 1.0) + 1.0;
    float D = a2 / (PI * denom * denom);
    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.05);
    float FH = 0.02 + 0.98 * pow(1.0 - max(dot(H, V), 0.0), 5.0);
    result += sunColor * min(D * FH * 0.25 / NdotV * NdotL, 24.0) * sun * oe_river_glint;

    // Fade out at the waterline (and at the edge of the drawing range).
    result = mix(oe_river_behind, result, clamp(oe_river_alpha, 0.0, 1.0));

    if (oe_river_debug.a > 0.5)
        result = oe_river_debug.rgb;

    color.rgb = oe_river_gamma(result);
    color.a = 1.0;
}
