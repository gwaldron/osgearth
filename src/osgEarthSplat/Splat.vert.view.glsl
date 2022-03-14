#pragma vp_entryPoint oe_splat_vertex_view
#pragma vp_location   vertex_view

#pragma include Splat.types.glsl

#pragma import_defines(OE_LANDCOVER_TEX)
#pragma import_defines(OE_LANDCOVER_TEX_MATRIX)

out vec4 oe_layer_tilec;
out float oe_splat_range;
out vec2 oe_splat_covtc;

uniform sampler2D OE_LANDCOVER_TEX;
uniform mat4 OE_LANDCOVER_TEX_MATRIX;

flat out float oe_splat_coverageTexSize;

//uniform mat4 OE_SPLAT_COVERAGE_TEXMAT;   // assigned at runtime

uniform vec3 oe_Camera; // (vp width, vp height, LOD scale)


void oe_splat_vertex_view(inout vec4 VertexVIEW)
{
    // range from camera to vertex
    oe_splat_range = -VertexVIEW.z * oe_Camera.z; // apply LOD scale

    // calculate the coverage sampling coordinates. The texture matrix accounts
    // for any super-sampling that might be in effect for the current LOD.
    oe_splat_covtc = (OE_LANDCOVER_TEX_MATRIX * oe_layer_tilec).st;

    // Precalculate the size of the coverage texture. This is faster than
    // calling textureSize per pixel in the fragment shader.
    oe_splat_coverageTexSize = textureSize(OE_LANDCOVER_TEX, 0).x;
}
