#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "oe_mp_setup_coloring"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0"

uniform float oe_layer_minRange;
uniform float oe_layer_maxRange;

varying vec4 oe_layer_texc;
varying vec4 oe_layer_tilec;
varying float oe_terrain_rangeOpacity;

void oe_mp_setup_coloring(inout vec4 vertexView)
{
    oe_layer_texc  = gl_MultiTexCoord$MP_PRIMARY_UNIT;
    oe_layer_tilec = gl_MultiTexCoord$MP_SECONDARY_UNIT;

    float range = -vertexView.z;

    float rangeSpanSlice = 0.1*(oe_layer_maxRange-oe_layer_minRange);
    float maxFadeSpan    = min(rangeSpanSlice, oe_layer_maxRange*0.1);
    float minFadeSpan    = min(rangeSpanSlice, oe_layer_minRange*0.1);

    oe_terrain_rangeOpacity =
        range > oe_layer_maxRange + maxFadeSpan ? 0.0 :
        range > oe_layer_maxRange               ? 1.0-(range-oe_layer_maxRange)/maxFadeSpan :
        range > oe_layer_minRange + minFadeSpan ? 1.0 :
        range > oe_layer_minRange               ? (range-oe_layer_minRange)/minFadeSpan :
        0.0;
}
