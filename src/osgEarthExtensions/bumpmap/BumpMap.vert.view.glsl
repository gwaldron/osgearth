#version 110

varying float oe_bumpmap_range;

void oe_bumpmap_vertexView(inout vec4 vertexView)
{
    oe_bumpmap_range = -vertexView.z;
}
