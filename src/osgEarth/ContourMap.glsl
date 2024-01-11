#pragma vp_function oe_contour_vertex, vertex_view
#pragma import_defines(OE_USE_GL4)

#ifdef OE_USE_GL4
#pragma include RexEngine.GL4.glsl

uint64_t oe_terrain_getElevationHandle();
vec2 oe_terrain_getElevationCoord(in vec2);
out vec4 oe_layer_tilec;
out vec2 oe_elev_coord;
flat out uint64_t oe_elev_tex;

void oe_contour_vertex(inout vec4 not_used)
{
    oe_elev_coord = oe_terrain_getElevationCoord(oe_layer_tilec.st);
    oe_elev_tex = oe_terrain_getElevationHandle();
}
#else
void oe_contour_vertex(inout vec4 not_used) { }
#endif

[break]
#pragma vp_function oe_contour_fragment, fragment_coloring
#pragma import_defines(OE_USE_GL4)

uniform sampler1D oe_contour_xfer;
uniform float oe_contour_min;
uniform float oe_contour_range;

#ifdef OE_USE_GL4

flat in uint64_t oe_elev_tex;
in vec2 oe_elev_coord;

void oe_contour_fragment( inout vec4 color )
{
    if (oe_elev_tex > 0)
    {
        float height = texture(sampler2D(oe_elev_tex), oe_elev_coord).r;
        float height_normalized = (height - oe_contour_min) / oe_contour_range;
        float lookup = clamp(height_normalized, 0.0, 1.0);
        vec4 texel = texture(oe_contour_xfer, lookup);
        color.rgb = mix(color.rgb, texel.rgb, texel.a);
    }
}

#else // OE_USE_GL4

// GL3 implementation:
float oe_terrain_getElevation(in vec2 uv);
in vec4 oe_layer_tilec;

void oe_contour_fragment(inout vec4 color)
{
    float height = oe_terrain_getElevation(oe_layer_tilec.st);
    float height_normalized = (height - oe_contour_min) / oe_contour_range;
    float lookup = clamp(height_normalized, 0.0, 1.0);
    vec4 texel = texture(oe_contour_xfer, lookup);
    color.rgb = mix(color.rgb, texel.rgb, texel.a);
}

#endif // OE_USE_GL4
