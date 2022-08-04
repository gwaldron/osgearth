// Adapted and ported to GLSL from:
// https://github.com/mmikk/hextile-demo

const float ht_g_fallOffContrast = 0.6;
const float ht_g_exp = 7;

#ifndef mul
#define mul(X, Y) ((X)*(Y))
#endif

#ifndef M_PI
#define M_PI 3.1417927
#endif

#define HEX_SCALE 3.46410161

// Output:\ weights associated with each hex tile and integer centers
void ht_TriangleGrid(
    out float w1, out float w2, out float w3,
    out ivec2 vertex1, out ivec2 vertex2, out ivec2 vertex3,
    in vec2 st)
{
    // Scaling of the input
    st *= HEX_SCALE; // 2 * 1.sqrt(3);

    // Skew input space into simplex triangle grid
    const mat2 gridToSkewedGrid = mat2(1.0, -0.57735027, 0.0, 1.15470054);
    vec2 skewedCoord = mul(gridToSkewedGrid, st);

    ivec2 baseId = ivec2(floor(skewedCoord));
    vec3 temp = vec3(fract(skewedCoord), 0);
    temp.z = 1.0 - temp.x - temp.y;

    float s = step(0.0, -temp.z);
    float s2 = 2 * s - 1;

    w1 = -temp.z*s2;
    w2 = s - temp.y*s2;
    w3 = s - temp.x*s2;

    vertex1 = baseId + ivec2(s, s);
    vertex2 = baseId + ivec2(s, 1 - s);
    vertex3 = baseId + ivec2(1 - s, s);
}

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
    vec2 skewedCoord = mul(gridToSkewedGrid, st);

    vec2 baseId = floor(skewedCoord);
    vec3 temp = vec3(fract(skewedCoord), 0);
    temp.z = 1.0 - temp.x - temp.y;

    float s = step(0.0, -temp.z);
    float s2 = 2 * s - 1;

    w1 = -temp.z*s2;
    w2 = s - temp.y*s2;
    w3 = s - temp.x*s2;

    vertex1 = baseId + vec2(s, s);
    vertex2 = baseId + vec2(s, 1 - s);
    vertex3 = baseId + vec2(1 - s, s);
}

vec2 ht_hash(vec2 p)
{
    vec2 r = mat2(127.1, 311.7, 269.5, 183.3) * p;
    return fract(sin(r)*43758.5453);
}

vec2 ht_MakeCenST(ivec2 Vertex)
{
    mat2 invSkewMat = mat2(1.0, 0.5, 0.0, 1.0 / 1.15470054);
    return mul(invSkewMat, Vertex) / HEX_SCALE;
}

mat2 ht_LoadRot2x2(ivec2 idx, float rotStrength)
{
    float angle = abs(idx.x*idx.y) + abs(idx.x + idx.y) + M_PI;

    // remap to +/-pi
    angle = mod(angle, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    if (angle > M_PI) angle -= 2 * M_PI;

    angle *= rotStrength;

    float cs = cos(angle), si = sin(angle);

    return mat2(cs, -si, si, cs);
}

vec3 ht_Gain3(vec3 x, float r)
{
    // increase contrast when r>0.5 and
    // reduce contrast if less
    float k = log(1 - r) / log(0.5);

    vec3 s = 2 * step(0.5, x);
    vec3 m = 2 * (1 - s);

    vec3 res = 0.5*s + 0.25*m * pow(max(vec3(0.0), s + x * m), vec3(k));

    return res.xyz / (res.x + res.y + res.z);
}

vec3 ht_ProduceHexWeights(vec3 W, ivec2 vertex1, ivec2 vertex2, ivec2 vertex3)
{
    vec3 res;

    int v1 = (vertex1.x - vertex1.y) % 3;
    if (v1 < 0) v1 += 3;

    int vh = v1 < 2 ? (v1 + 1) : 0;
    int vl = v1 > 0 ? (v1 - 1) : 2;
    int v2 = vertex1.x < vertex3.x ? vl : vh;
    int v3 = vertex1.x < vertex3.x ? vh : vl;

    res.x = v3 == 0 ? W.z : (v2 == 0 ? W.y : W.x);
    res.y = v3 == 1 ? W.z : (v2 == 1 ? W.y : W.x);
    res.z = v3 == 2 ? W.z : (v2 == 2 ? W.y : W.x);

    return res;
}

// Input: vM is tangent space normal in [-1;1].
// Output: convert vM to a derivative.
vec2 ht_TspaceNormalToDerivative(in vec3 vM)
{
    const float scale = 1.0 / 128.0;

    // Ensure vM delivers a positive third component using abs() and
    // constrain vM.z so the range of the derivative is [-128; 128].
    const vec3 vMa = abs(vM);
    const float z_ma = max(vMa.z, scale*max(vMa.x, vMa.y));

    // Set to match positive vertical texture coordinate axis.
    const bool gFlipVertDeriv = true;
    const float s = gFlipVertDeriv ? -1.0 : 1.0;
    return -vec2(vM.x, s*vM.y) / z_ma;
}

vec2 ht_sampleDeriv(sampler2D nmap, vec2 st, vec2 dSTdx, vec2 dSTdy)
{
    // sample
    vec3 vM = 2.0*textureGrad(nmap, st, dSTdx, dSTdy).xyz - 1.0;
    return ht_TspaceNormalToDerivative(vM);
}


// Input:\ nmap is a normal map
// Input:\ r increase contrast when r>0.5
// Output:\ deriv is a derivative dHduv wrt units in pixels
// Output:\ weights shows the weight of each hex tile
void bumphex2derivNMap(
    out vec2 deriv, out vec3 weights,
    sampler2D nmap, vec2 st,
    float rotStrength, float r)
{
    vec2 dSTdx = dFdx(st), dSTdy = dFdy(st);

    // Get triangle info
    float w1, w2, w3;
    ivec2 vertex1, vertex2, vertex3;
    ht_TriangleGrid(w1, w2, w3, vertex1, vertex2, vertex3, st);

    mat2 rot1 = ht_LoadRot2x2(vertex1, rotStrength);
    mat2 rot2 = ht_LoadRot2x2(vertex2, rotStrength);
    mat2 rot3 = ht_LoadRot2x2(vertex3, rotStrength);

    vec2 cen1 = ht_MakeCenST(vertex1);
    vec2 cen2 = ht_MakeCenST(vertex2);
    vec2 cen3 = ht_MakeCenST(vertex3);

    vec2 st1 = mul(st - cen1, rot1) + cen1 + ht_hash(vertex1);
    vec2 st2 = mul(st - cen2, rot2) + cen2 + ht_hash(vertex2);
    vec2 st3 = mul(st - cen3, rot3) + cen3 + ht_hash(vertex3);

    // Fetch input
    vec2 d1 = ht_sampleDeriv(nmap, st1,
        mul(dSTdx, rot1), mul(dSTdy, rot1));
    vec2 d2 = ht_sampleDeriv(nmap, st2,
        mul(dSTdx, rot2), mul(dSTdy, rot2));
    vec2 d3 = ht_sampleDeriv(nmap, st3,
        mul(dSTdx, rot3), mul(dSTdy, rot3));

    d1 = mul(rot1, d1); d2 = mul(rot2, d2); d3 = mul(rot3, d3);

    // produce sine to the angle between the conceptual normal
    // in tangent space and the Z-axis
    vec3 D = vec3(dot(d1, d1), dot(d2, d2), dot(d3, d3));
    vec3 Dw = sqrt(D / (1.0 + D));

    Dw = mix(vec3(1.0), Dw, ht_g_fallOffContrast);	// 0.6
    vec3 W = Dw * pow(vec3(w1, w2, w3), vec3(ht_g_exp));	// 7
    W /= (W.x + W.y + W.z);
    if (r != 0.5) W = ht_Gain3(W, r);

    deriv = W.x * d1 + W.y * d2 + W.z * d3;
    weights = ht_ProduceHexWeights(W.xyz, vertex1, vertex2, vertex3);
}


// Input:\ tex is a texture with color
// Input:\ r increase contrast when r>0.5
// Output:\ color is the blended result
// Output:\ weights shows the weight of each hex tile
void ht_hex2colTex(
    out vec4 color,
    sampler2D tex,
    vec2 st,
    float rotStrength)
{
    vec2 dSTdx = dFdx(st), dSTdy = dFdy(st);

    // Get triangle info
    float w1, w2, w3;
    ivec2 vertex1, vertex2, vertex3;
    ht_TriangleGrid(w1, w2, w3, vertex1, vertex2, vertex3, st);

    mat2 rot1 = ht_LoadRot2x2(vertex1, rotStrength);
    mat2 rot2 = ht_LoadRot2x2(vertex2, rotStrength);
    mat2 rot3 = ht_LoadRot2x2(vertex3, rotStrength);

    vec2 cen1 = ht_MakeCenST(vertex1);
    vec2 cen2 = ht_MakeCenST(vertex2);
    vec2 cen3 = ht_MakeCenST(vertex3);

    vec2 st1 = mul(st - cen1, rot1) + cen1 + ht_hash(vertex1);
    vec2 st2 = mul(st - cen2, rot2) + cen2 + ht_hash(vertex2);
    vec2 st3 = mul(st - cen3, rot3) + cen3 + ht_hash(vertex3);

    // Fetch input
    vec4 c1 = textureGrad(tex, st1, dSTdx*rot1, dSTdy*rot1);
    vec4 c2 = textureGrad(tex, st2, dSTdx*rot2, dSTdy*rot2);
    vec4 c3 = textureGrad(tex, st3, dSTdx*rot3, dSTdy*rot3);

    // use luminance as weight
    const vec3 Lw = vec3(0.299, 0.587, 0.114);
    vec3 Dw = vec3(dot(c1.xyz, Lw), dot(c2.xyz, Lw), dot(c3.xyz, Lw));

    Dw = mix(vec3(1.0), Dw, ht_g_fallOffContrast);	// 0.6
    vec3 W = Dw * pow(vec3(w1, w2, w3), vec3(ht_g_exp));	// 7
    W /= (W.x + W.y + W.z);
    //if (r != 0.5) W = Gain3(W, r);

    color = W.x * c1 + W.y * c2 + W.z * c3;
    //weights = ProduceHexWeights(W.xyz, vertex1, vertex2, vertex3);
}

#define HT_HASH(X) fract(sin(mat2(127.1, 311.7, 269.5, 183.3) * X)*43758.5453)

// Hextiling function optimized for no rotations and to 
// sample and interpolate both color and material vectors
void ht_hex2colTex_optimized(
    in sampler2D color_tex,
    in sampler2D material_tex,
    in vec2 st,
    out vec4 color,
    out vec4 material)
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
    vec2 ddx = dFdx(st), ddy = dFdy(st);

    vec4 c1 = textureGrad(color_tex, st1, ddx, ddy);
    vec4 c2 = textureGrad(color_tex, st2, ddx, ddy);
    vec4 c3 = textureGrad(color_tex, st3, ddx, ddy);

    vec4 m1 = textureGrad(material_tex, st1, ddx, ddy);
    vec4 m2 = textureGrad(material_tex, st2, ddx, ddy);
    vec4 m3 = textureGrad(material_tex, st3, ddx, ddy);

    // Use color's luminance as weighting factor
    const vec3 Lw = vec3(0.299, 0.587, 0.114);
    vec3 Dw = vec3(dot(c1.xyz, Lw), dot(c2.xyz, Lw), dot(c3.xyz, Lw));
    Dw = mix(vec3(1.0), Dw, ht_g_fallOffContrast);
    vec3 W = Dw * pow(weights, vec3(ht_g_exp));
    W /= (W.x + W.y + W.z);

    color = W.x * c1 + W.y * c2 + W.z * c3;
    material = W.x * m1 + W.y * m2 + W.z * m3;
}
