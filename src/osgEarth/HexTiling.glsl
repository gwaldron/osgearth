#pragma vp_name osgEarth Hex Tiling Library

#pragma import_defines(OE_ENABLE_HEX_TILER_ANISOTROPIC_FILTERING)

// Adapted and ported to GLSL from:
// https://github.com/mmikk/hextile-demo

const float ht_g_fallOffContrast = 0.6;
const float ht_g_exp = 7;

#ifdef VP_STAGE_FRAGMENT

#ifndef mul
#define mul(X, Y) ((X)*(Y))
#endif

#ifndef M_PI
#define M_PI 3.1415927
#endif

#define HEX_SCALE 3.46410161

// Output:\ weights associated with each hex tile and integer centers
void ht_TriangleGrid_f(
    out float w1, out float w2, out float w3,
    out vec2 vertex1, out vec2 vertex2, out vec2 vertex3,
    in vec2 st)
{
    // Scaling of the input
    st *= HEX_SCALE; // 2 * 1.sqrt(3);

    // Skew input space into simplex triangle grid
    const mat2 gridToSkewedGrid = mat2(1.0, -0.57735027, 0.0, 1.15470054);
    vec2 skewedCoord = gridToSkewedGrid * st;

    vec2 baseId = floor(skewedCoord);
    vec3 temp = vec3(fract(skewedCoord), 0.0);
    temp.z = 1.0 - temp.x - temp.y;

    float s = step(0.0, -temp.z);
    float s2 = 2.0 * s - 1.0;

    w1 = -temp.z * s2;
    w2 = s - temp.y * s2;
    w3 = s - temp.x * s2;

    vertex1 = baseId + vec2(s, s);
    vertex2 = baseId + vec2(s, 1.0 - s);
    vertex3 = baseId + vec2(1.0 - s, s);
}

vec2 ht_hash(in vec2 p)
{
    vec2 r = mat2(127.1, 311.7, 269.5, 183.3) * p;
    return fract(sin(r) * 43758.5453);
}

// Hextiling function optimized for no rotations and to 
// sample and interpolate both color and material vectors
void ht_hex2colTex_optimized(
    in sampler2D color_tex,
    in sampler2D material_tex,
    in vec2 st,
    out vec4 color,
    out vec4 material,
    inout vec3 weighting)
{
    // Get triangle info
    vec3 weights;
    vec2 vertex1, vertex2, vertex3;
    ht_TriangleGrid_f(weights[0], weights[1], weights[2], vertex1, vertex2, vertex3, st);

    // randomize the sampling offsets:
    vec2 st1 = st + ht_hash(vertex1);
    vec2 st2 = st + ht_hash(vertex2);
    vec2 st3 = st + ht_hash(vertex3);

    // Use the same partial derivitives to sample all three locations
    // to avoid rendering artifacts.

#if OE_ENABLE_HEX_TILER_ANISOTROPIC_FILTERING

    // Original approach: use textureGrad to supply the same gradient
    // for each sample point (slow)
    float bias = pow(2.0, -0.5);
    vec2 ddx = dFdx(st) * bias, ddy = dFdy(st) * bias;

    vec4 c1 = textureGrad(color_tex, st1, ddx, ddy);
    vec4 c2 = textureGrad(color_tex, st2, ddx, ddy);
    vec4 c3 = textureGrad(color_tex, st3, ddx, ddy);

    vec4 m1 = textureGrad(material_tex, st1, ddx, ddy);
    vec4 m2 = textureGrad(material_tex, st2, ddx, ddy);
    vec4 m3 = textureGrad(material_tex, st3, ddx, ddy);

#else
    // Fast way: replace textureGrad by manually calculating the LOD
    // and using textureLod instead (much faster than textureGrad)
    // https://web.archive.org/web/20231209114942/https://solidpixel.github.io/2022/03/27/texture_sampling_tips.html
    // Beware: this approach will disable anisotropic filtering

    ivec2 tex_dim;
    vec2 ddx, ddy;
    float lod;

    vec2 st_ddx = dFdx(st), st_ddy = dFdy(st);

    tex_dim = textureSize(color_tex, 0);
    ddx = st_ddx * float(tex_dim.x), ddy = st_ddy * float(tex_dim.y);
    lod = 0.5 * log2(max(dot(ddx, ddx), dot(ddy, ddy)));

    vec4 c1 = textureLod(color_tex, st1, lod);
    vec4 c2 = textureLod(color_tex, st2, lod);
    vec4 c3 = textureLod(color_tex, st3, lod);

    tex_dim = textureSize(material_tex, 0);
    ddx = st_ddx * float(tex_dim.x), ddy = st_ddy * float(tex_dim.y);
    lod = 0.5 * log2(max(dot(ddx, ddx), dot(ddy, ddy)));

    vec4 m1 = textureLod(material_tex, st1, lod);
    vec4 m2 = textureLod(material_tex, st2, lod);
    vec4 m3 = textureLod(material_tex, st3, lod);
#endif

    vec3 W = weighting;
    if (W == vec3(0))
    {
        // Use color's luminance as weighting factor
        vec3 Lw = vec3(0.299, 0.587, 0.114);
        vec3 Dw = vec3(dot(c1.xyz, Lw), dot(c2.xyz, Lw), dot(c3.xyz, Lw));
        Dw = mix(vec3(1.0), Dw, ht_g_fallOffContrast);
        W = Dw * pow(weights, vec3(ht_g_exp));
        W /= (W.x + W.y + W.z);
    }

    weighting = W;
    color = W.x * c1 + W.y * c2 + W.z * c3;
    material = W.x * m1 + W.y * m2 + W.z * m3;
}

#endif // VP_STAGE_FRAGMENT