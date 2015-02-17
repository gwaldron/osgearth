#version 120

#pragma vp_entryPoint "oe_clamp_vertex"
#pragma vp_location   "vertex_view"

#pragma include "GPUClamping.vert.lib.glsl"

attribute vec4 oe_clamp_attrs;
uniform float oe_clamp_altitudeOffset;
uniform float oe_clamp_horizonDistance2;
uniform bool oe_clamp_hasAttrs;
varying float oe_clamp_alpha;

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
        vec4  clampedPoint;
        float depth;
        float move = 0.0f;

        bool applyClampAttrs = (oe_clamp_hasAttrs == true) && (oe_clamp_attrs.a == 1.0);

        if ( applyClampAttrs )
        {
            // roof/extruded point:
            vec4 refPoint = gl_ModelViewMatrix * vec4(oe_clamp_attrs.xyz, 1.0);
            oe_getClampedViewVertex(refPoint, clampedPoint, depth);            
            move = distance(refPoint, clampedPoint);
        }
        else
        {
            // base/ground point:
            oe_getClampedViewVertex(vertexView, clampedPoint, depth);
            vertexView.xyz = clampedPoint.xyz;
        }

        // apply the altitude offset.
        vec3 up;
        oe_getClampingUpVector(up);
        
        vertexView.xyz += up*(move + oe_clamp_altitudeOffset);

        // if the clamped depth value is near the far plane, suppress drawing
        // to avoid rendering anomalies.
        oe_clamp_alpha = 1.0-step(0.99999, depth);
    }
}
