#pragma vp_entryPoint oe_overlay_vertex
#pragma vp_location   vertex_view

uniform mat4 oe_overlay_texmatrix;
uniform float oe_overlay_rttLimitZ;

out vec4 oe_overlay_texcoord;

void oe_overlay_vertex(inout vec4 vertexVIEW)
{
    oe_overlay_texcoord = oe_overlay_texmatrix * vertexVIEW;
}


[break]
#pragma vp_entryPoint oe_overlay_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      1.1

#pragma import_defines(OE_IS_PICK_CAMERA)
#pragma import_defines(OE_DISABLE_DRAPING)

uniform sampler2D oe_overlay_tex;
in vec4 oe_overlay_texcoord;

void oe_overlay_fragment(inout vec4 color)
{
#ifdef OE_DISABLE_DRAPING
    return;
#endif

    vec4 texel = textureProj(oe_overlay_tex, oe_overlay_texcoord);

#ifdef OE_IS_PICK_CAMERA
    color = texel;
#else
    color = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a);
#endif
}
