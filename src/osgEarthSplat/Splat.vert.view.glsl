#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_splat_vertex_view
#pragma vp_location   vertex_view
#pragma vp_order      0.5

#pragma include Splat.types.glsl

out vec4 oe_layer_tilec;
out float oe_splat_range;
out vec2 oe_splat_covtc;

uniform sampler2D oe_splat_coverageTex;
flat out float oe_splat_coverageTexSize;

uniform mat4 COVERAGE_TEXTURE_MATRIX;   // assigned at runtime


void oe_splat_vertex_view(inout vec4 VertexVIEW)
{
    // range from camera to vertex
    oe_splat_range = -VertexVIEW.z;

    // calculate the coverage sampling coordinates. The texture matrix accounts
    // for any super-sampling that might be in effect for the current LOD.
    oe_splat_covtc = (COVERAGE_TEXTURE_MATRIX * oe_layer_tilec).st;

    // Precalculate the size of the coverage texture. This is faster than
    // calling textureSize per pixel in the fragment shader.
    oe_splat_coverageTexSize = textureSize(oe_splat_coverageTex, 0).x;
}
