#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_clamp_vertex
#pragma vp_location   vertex_view
#pragma vp_order      0.5

#pragma include GPUClamping.vert.lib.glsl

in vec4 oe_clamp_attrs;     // vertex attribute
in float oe_clamp_height;   // vertex attribute

out float oe_clamp_alpha;

uniform bool oe_clamp_hasAttrs;
uniform bool oe_isGeocentric;
uniform float oe_clamp_altitudeOffset;
uniform float oe_clamp_horizonDistance2;

void oe_clamp_vertex(inout vec4 vertexView)
{
    const float ClampToAnchor = 1.0;

    // check distance; alpha out if its beyone the horizon distance.
    oe_clamp_alpha = oe_isGeocentric ? 
        clamp(oe_clamp_horizonDistance2 - (vertexView.z*vertexView.z), 0.0, 1.0) :
        1.0;

    // if visible, calculate clamping.
    // note: no branch divergence in the vertex shader
    if ( oe_clamp_alpha > 0.0 )
    {
        bool relativeToAnchor = oe_clamp_hasAttrs && (oe_clamp_attrs.a == ClampToAnchor);

        float verticalOffset = oe_clamp_hasAttrs ? oe_clamp_attrs.z : 0.0;

        // if we are using the anchor point, xform it into view space to prepare
        // for clamping. Force Z=0 for anchoring.
        vec4 pointToClamp = relativeToAnchor?
            gl_ModelViewMatrix * vec4(oe_clamp_attrs.xy, 0.0, 1.0) :
            vertexView;

        // find the clamped point.
        vec4  clampedPoint;
        float depth;
        oe_getClampedViewVertex(pointToClamp, clampedPoint, depth);

        float dh = verticalOffset + oe_clamp_altitudeOffset;

        if ( relativeToAnchor )
        {
            // if we are clamping relative to the anchor point, adjust the HAT based on the
            // distance from the anchor point to the terrain. Since distance() is unsigned,
            // we use the vector dot product to calculate whether to adjust up or down.
            float dist = distance(pointToClamp, clampedPoint);
            float dir  = sign(dot(clampedPoint-pointToClamp, vertexView-pointToClamp));
            dh += (dist * dir);
        }
        else
        {
            // if we are clamping to the terrain, the vertex becomes the
            // clamped point
            vertexView.xyz = clampedPoint.xyz;
            dh += oe_clamp_height;
        }
        
        // calculate the up vector along which clamping will occur (in either direction)
        vec3 up;
        oe_getClampingUpVector( up );
        vertexView.xyz += up*dh;

        // if the clamped depth value is near the far plane, suppress drawing
        // to avoid rendering anomalies.
        oe_clamp_alpha = 1.0-step(0.9999, depth);
    }
}
