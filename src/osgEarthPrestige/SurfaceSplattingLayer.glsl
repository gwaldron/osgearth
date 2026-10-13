#pragma vp_name Prestige surface coordinates
#pragma vp_function oe_surface_vertex, vertex_view, 0.9

uniform mat4 oe_surface_viewToTexture;
out vec2 oe_surface_uv;
out vec3 oe_surface_viewPosition;

// Derive metric UVs from the rendered (already morphed) regular terrain vertices.
void oe_surface_vertex(inout vec4 vertex)
{
    oe_surface_uv = (oe_surface_viewToTexture * vertex).xy;
    oe_surface_viewPosition = vertex.xyz;
}

[break]
#pragma vp_name Prestige terrain surface material
#pragma vp_location fragment_coloring
#pragma import_defines(OE_USE_PBR, OE_IS_SHADOW_CAMERA, OE_IS_DEPTH_CAMERA, OE_IS_PICK_CAMERA)
#pragma include PBRMaterial.glsl

uniform sampler2D oe_surface_albedo;
uniform sampler2D oe_surface_normal;
uniform sampler2D oe_surface_pbr;
uniform sampler2D oe_surface_ao;
uniform vec4 oe_surface_factors;
uniform int oe_surface_flags;
uniform float oe_surface_normalStrength;
in vec2 oe_surface_uv;
in vec3 oe_surface_viewPosition;
in vec3 vp_Normal;
#ifdef OE_USE_PBR
struct OE_PBR { float displacement, roughness, ao, metal; } oe_pbr;
#endif

// Called by terrain after coverage morph/opacity and before first-layer or framebuffer blending.
void oe_terrain_applyLayerMaterial(inout vec4 texel)
{
#if !defined(OE_IS_SHADOW_CAMERA) && !defined(OE_IS_DEPTH_CAMERA) && !defined(OE_IS_PICK_CAMERA)
    float coverage = texel.a;
    vec4 albedo = texture(oe_surface_albedo, oe_surface_uv);
    if ((oe_surface_flags & 8) != 0)
        albedo.rgb = mix(1.055 * pow(max(albedo.rgb, 0.0), vec3(1.0/2.4)) - 0.055,
            12.92 * albedo.rgb, lessThanEqual(albedo.rgb, vec3(0.0031308)));
    texel.rgb = albedo.rgb;
    // Material alpha never enlarges or erases the compiler's pavement footprint.
    if ((oe_surface_flags & 1) != 0)
    {
        vec3 N = normalize(vp_Normal);
        vec3 p1 = dFdx(oe_surface_viewPosition), p2 = dFdy(oe_surface_viewPosition);
        vec2 t1 = dFdx(oe_surface_uv), t2 = dFdy(oe_surface_uv);
        vec3 q2 = cross(p2, N), q1 = cross(N, p1);
        vec3 T = q2*t1.x + q1*t2.x, B = q2*t1.y + q1*t2.y;
        float magnitude = max(dot(T,T), dot(B,B));
        float scale = inversesqrt(max(magnitude, 1e-20));
        vec3 bump = texture(oe_surface_normal, oe_surface_uv).xyz*2.0-1.0;
        bump.xy *= oe_surface_normalStrength;
        vec3 perturbed = mat3(T*scale, B*scale, N)*bump;
        vp_Normal = normalize(mix(N, normalize(perturbed), coverage));
    }
#ifdef OE_USE_PBR
    vec4 packedValue = oe_pbr_default(oe_surface_factors.x);
    if ((oe_surface_flags & 2) != 0) packedValue = texture(oe_surface_pbr, oe_surface_uv);
    float ao = (oe_surface_flags & 4) != 0 ? texture(oe_surface_ao, oe_surface_uv).r : -1.0;
    vec4 material = oe_pbr_decode(packedValue, oe_surface_factors, ao);
    oe_pbr.roughness = mix(oe_pbr.roughness, material.g, coverage);
    oe_pbr.ao = mix(oe_pbr.ao, material.b, coverage);
    oe_pbr.metal = mix(oe_pbr.metal, material.a, coverage);
#endif
#endif
}
