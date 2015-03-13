#version 110

#pragma vp_entryPoint "oe_bumpmap_vertexView"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.5"

varying float oe_bumpmap_range;

void oe_bumpmap_vertexView(inout vec4 vertexView)
{
    oe_bumpmap_range = -vertexView.z;
}
