#version 110

#pragma vp_entryPoint "oe_clamp_vertex"
#pragma vp_location   "vertex_view"

#pragma include "GPUClamping.vert.lib.glsl"

uniform float oe_clamp_horizonDistance2;
varying float oe_clamp_alpha;

// prototype; see GPUClamping.vert.lib.glsl
//void oe_getClampedViewVertex(in  vec4  vertView,
//                             out vec4  out_clampedVertView,
//                             out float out_depth);

// clamp a vertex to the ground
void oe_clamp_vertex(inout vec4 vertexView)
{
    // check distance; alpha out if its beyone the horizon distance.
    float dist2 = vertexView.z*vertexView.z;
    oe_clamp_alpha = clamp(oe_clamp_horizonDistance2 - dist2, 0.0, 1.0);

    // if visible, calculate clamping.
    // note: no branch divergence in the vertex shader
    if ( oe_clamp_alpha > 0.0 )
    {
        vec4  clampedVertexView;
        float depth;

        // get the clamped vertex:
        oe_getClampedViewVertex(vertexView, clampedVertexView, depth);
        vertexView = clampedVertexView;

        // if the clamped depth value is near the far plane, suppress drawing
        // to avoid rendering anomalies.
        oe_clamp_alpha = 1.0-step(0.99999, depth);
    }
}
