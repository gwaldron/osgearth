#pragma vp_name River Spray - Vertex
#pragma vp_function oe_spray_vertex, vertex_view
#pragma import_defines(OE_LOG_DEPTH_BUFFER)

// Spray and mist from waterfalls and rapids. Each particle is a quad whose four corners all sit
// at the particle's source; this stage expands it toward the camera and animates it over its
// life, so the CPU never touches the particles after building them.

uniform float oe_river_time;
uniform vec4 oe_river_p5; // spray, rainbows

out vec2 oe_spray_corner;
out vec3 oe_spray_viewPos;
out vec3 oe_spray_up;
out float oe_spray_alpha;
out float oe_spray_radius;
out float oe_spray_seed;
flat out float oe_spray_depthFar;

vec3 vp_Normal; // stage global: view-space normal (local up at the particle's source)
vec4 vp_Color;  // stage global: r = particle density

void oe_spray_vertex(inout vec4 vertexView)
{
    vec4 shape = gl_MultiTexCoord0;  // corner xy, seed, radius (m)
    vec4 motion = gl_MultiTexCoord1; // drift east/north (m/s), rise (m/s), life (s)
    float seed = shape.z;
    float life = max(motion.w, 0.5);

    // Each particle loops through its life on a phase of its own.
    float age = fract(oe_river_time / life + seed * 7.13);
    float t = age * life;

    vec3 up = normalize(vp_Normal);
    vec3 east = normalize(gl_NormalMatrix * vec3(1.0, 0.0, 0.0));
    vec3 north = normalize(cross(up, east));
    east = cross(north, up);

    // Puffs boil up and slow as they go, drift with the current, swell and swirl.
    float rise = motion.z * t * (1.0 - 0.4 * age);
    float spin = seed * 6.2831853 + t * (0.6 + seed);
    vec3 swirl = (east * cos(spin) + north * sin(spin)) * (0.25 * shape.w * age);
    vec3 center = vertexView.xyz + up * rise + (east * motion.x + north * motion.y) * t + swirl;
    float radius = shape.w * (0.4 + 0.9 * sqrt(age));

    // A camera-facing quad, turning slowly so the puffs never look stamped.
    float turn = seed * 6.2831853 + t * (seed - 0.5) * 0.5;
    vec2 c = shape.xy;
    vec2 rc = vec2(c.x * cos(turn) - c.y * sin(turn), c.x * sin(turn) + c.y * cos(turn));
    vertexView.xyz = center + vec3(rc * radius, 0.0);

    oe_spray_corner = c;
    oe_spray_viewPos = vertexView.xyz;
    oe_spray_up = up;
    oe_spray_radius = radius;
    oe_spray_seed = seed;

    // Fade in fast and out slowly; thin out when the eye is inside a puff (so mist never
    // smothers the view) and far away.
    float range = length(center);
    oe_spray_alpha = vp_Color.r * oe_river_p5.x
        * smoothstep(0.0, 0.12, age) * (1.0 - smoothstep(0.5, 1.0, age))
        * smoothstep(0.6 * radius, 2.5 * radius, range)
        * (1.0 - smoothstep(5000.0, 9000.0, range));

#ifdef OE_LOG_DEPTH_BUFFER
    // The far distance exactly as the logarithmic depth shader computes it.
    vec4 farPoint = inverse(gl_ProjectionMatrix) * vec4(0.0, 0.0, 1.0, 1.0);
    oe_spray_depthFar = -farPoint.z / farPoint.w;
#else
    oe_spray_depthFar = 0.0;
#endif
}


[break]
#pragma vp_name River Spray - Shape
#pragma vp_function oe_spray_fragment, fragment_coloring, 0.9
#pragma import_defines(OE_RIVER_SCREEN, OE_LOG_DEPTH_BUFFER)

uniform sampler2D oe_river_foamTex;    // b = low-frequency noise, a = mid-frequency noise
uniform sampler2D oe_river_sceneDepth; // depth of the opaque scene (copied before the water)
uniform vec4 oe_river_viewport;        // x, y, width, height of the copy in window pixels
uniform float oe_river_screenValid;    // 1 when the copy holds this camera's scene
uniform float oe_river_time;
uniform mat4 osg_ProjectionMatrix;

in vec2 oe_spray_corner;
in vec3 oe_spray_viewPos;
in vec3 oe_spray_up;
in float oe_spray_alpha;
in float oe_spray_radius;
in float oe_spray_seed;
flat in float oe_spray_depthFar;

vec3 vp_Normal;
struct OE_PBR { float displacement, roughness, ao, metal; } oe_pbr;

// Handed to the composite stage, which runs after the scene's lighting model.
float oe_spray_density;
vec3 oe_spray_V;
vec3 oe_spray_U;

// Distance along -z of a depth buffer value (perspective projection, standard or osgEarth's
// logarithmic depth buffer), as the water shader computes it.
float oe_spray_eyeDepth(float d)
{
#if defined(OE_LOG_DEPTH_BUFFER) && OE_LOG_DEPTH_BUFFER == 2
    const float C = 0.001;
    return (pow(oe_spray_depthFar * C + 1.0, d) - 1.0) / C;
#elif defined(OE_LOG_DEPTH_BUFFER)
    return pow(oe_spray_depthFar + 1.0, d) - 1.0;
#else
    return osg_ProjectionMatrix[3][2] / (d * 2.0 - 1.0 + osg_ProjectionMatrix[2][2]);
#endif
}

// Shapes a puff (soft disk with churning billows, thinned where it touches anything opaque)
// and hands the scene's lighting model a gray probe to light it with.
void oe_spray_fragment(inout vec4 color)
{
    float r = length(oe_spray_corner);
    if (r >= 1.0 || oe_spray_alpha <= 0.002)
        discard;

    // A soft round puff with a billowing texture that churns slowly.
    vec2 base = oe_spray_corner * 0.3 + vec2(oe_spray_seed * 3.7, oe_spray_seed * 1.9);
    vec4 n1 = texture(oe_river_foamTex, base + vec2(0.011, -0.035) * oe_river_time);
    vec4 n2 = texture(oe_river_foamTex, base * 2.3 + vec2(-0.027, 0.019) * oe_river_time);
    float billow = clamp(n1.b * 0.9 + n2.a * 0.6 - 0.35, 0.0, 1.0);
    float core = 1.0 - smoothstep(0.0, 1.0, r);
    float density = core * core * (0.3 + 1.1 * billow) * oe_spray_alpha;

#ifdef OE_RIVER_SCREEN
    // Soft contact: thin out where the puff meets the ground or anything else opaque.
    if (oe_river_screenValid > 0.5)
    {
        vec2 suv = (gl_FragCoord.xy - oe_river_viewport.xy) / oe_river_viewport.zw;
        float sceneEye = oe_spray_eyeDepth(texture(oe_river_sceneDepth, suv).r);
        density *= smoothstep(0.0, max(0.5 * oe_spray_radius, 1.0), sceneEye + oe_spray_viewPos.z);
    }
#endif
    if (density < 0.003)
        discard;

    // Hand the scene's lighting model a neutral gray probe, as the water does; the composite
    // turns the light it receives into lit mist.
    color = vec4(pow(vec3(0.18), vec3(1.0 / 2.2)), 1.0);
    vp_Normal = oe_spray_up;
    oe_pbr.roughness = 1.0;
    oe_pbr.metal = 0.0;
    oe_pbr.ao = 1.0;
    oe_spray_density = density;
    oe_spray_V = normalize(-oe_spray_viewPos);
    oe_spray_U = oe_spray_up;
}


[break]
#pragma vp_name River Spray - Composite
#pragma vp_function oe_spray_composite, fragment_lighting, 0.95
#pragma import_defines(OE_NUM_LIGHTS, OE_SHADOWING)

uniform vec4 oe_river_p5; // spray, rainbows

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

float oe_spray_density;
vec3 oe_spray_V;
vec3 oe_spray_U;

// Color across a rainbow band: 0 = violet (inner edge) to 1 = red (outer edge).
vec3 oe_spray_spectrum(float x)
{
    float h = (1.0 - clamp(x, 0.0, 1.0)) * 0.76;
    return clamp(abs(fract(h + vec3(0.0, 2.0 / 3.0, 1.0 / 3.0)) * 6.0 - 3.0) - 1.0, 0.0, 1.0);
}

// Weight of a rainbow band at position x (0..1 across it), soft at both edges.
float oe_spray_band(float x)
{
    return smoothstep(-0.25, 0.15, x) * (1.0 - smoothstep(0.85, 1.25, x));
}

// Turns the light the probe received into lit mist: diffuse light, sunlight scattered forward
// through the droplets, and rainbows; outputs straight (not premultiplied) alpha.
void oe_spray_composite(inout vec4 color)
{
    const float PROBE = 0.18;

    // Light reaching the mist, as the scene's lighting model sees it.
    vec3 E = pow(max(color.rgb, vec3(0.0)), vec3(2.2)) / PROBE;

    vec3 L = oe_spray_U;
    vec3 sunColor = vec3(0.0);
    float sun = 0.0;
#ifdef OE_NUM_LIGHTS
    if (osg_LightSource[0].enabled)
    {
        L = normalize(osg_LightSource[0].position.xyz);
        sunColor = osg_LightSource[0].diffuse.rgb;
        sun = smoothstep(-0.02, 0.1, dot(L, oe_spray_U));
    }
#endif
#ifdef OE_SHADOWING
    sun *= oe_shadow_visibility;
#endif

    // Droplets scatter sunlight mostly forward: mist glows when seen against the sun.
    vec3 D = -oe_spray_V; // from the eye into the mist
    const float g = 0.6;
    float hg = (1.0 - g * g) / pow(max(1.0 + g * g - 2.0 * g * dot(D, L), 1e-4), 1.5);
    vec3 mist = E * 0.75 + sunColor * sun * min(hg, 30.0) * 0.08;

    // Rainbows: sunlight bent and reflected inside the droplets comes back about 42 degrees from
    // the antisolar point (red outside, violet inside), with a fainter, reversed secondary bow
    // near 51 degrees.
    float angle = degrees(acos(clamp(dot(D, -L), -1.0, 1.0)));
    float primary = (angle - 40.6) / 1.8;
    float secondary = (53.6 - angle) / 3.2;
    vec3 bow = oe_spray_spectrum(primary) * oe_spray_band(primary)
        + oe_spray_spectrum(secondary) * oe_spray_band(secondary) * 0.35;
    mist += bow * sunColor * sun * oe_river_p5.y * 0.5;

    color = vec4(pow(max(mist, vec3(0.0)), vec3(1.0 / 2.2)), clamp(oe_spray_density, 0.0, 1.0));
}
