#version 110

// uniforms from this ClampingTechnique:
uniform sampler2D oe_clamp_depthTex;
uniform mat4 oe_clamp_cameraView2depthClip;

#ifdef SUPPORT_Z
uniform mat4 oe_clamp_depthClip2depthView;
uniform mat4 oe_clamp_depthView2cameraView;
#else
uniform mat4 oe_clamp_depthClip2cameraView;
#endif

uniform float oe_clamp_horizonDistance;
varying float oe_clamp_alphaFactor;

void oe_clamp_vertex(inout vec4 VertexVIEW)
{
    // start by mocing the vertex into view space.
    vec4 v_view_orig = VertexVIEW;

    // if the distance to the vertex is beyond the visible horizon,
    // "hide" the vertex by setting its alpha component to 0.0.
    // if this is the case, there's no point in continuing -- so we 
    // would normally branch here, but since this happens on the fly 
    // the shader engine will run both branches regardless. So keep going.
    float vert_distance = length(v_view_orig.xyz/v_view_orig.w);
    oe_clamp_alphaFactor = clamp(oe_clamp_horizonDistance - vert_distance, 0.0, 1.0 );

    //   transform the vertex into the depth texture's clip coordinates.
    vec4 v_depthClip = oe_clamp_cameraView2depthClip * v_view_orig;

    //   sample the depth map.
    float d = texture2DProj( oe_clamp_depthTex, v_depthClip ).r;

    //   blank it out if it's at the far plane (no terrain visible)
    if ( d > 0.999999 ) { oe_clamp_alphaFactor = 0.0; }

    //   now transform into depth-view space so we can apply the height-above-ground:
    vec4 p_depthClip = vec4(v_depthClip.x, v_depthClip.y, d, 1.0);

#ifdef SUPPORT_Z
    vec4 p_depthView = oe_clamp_depthClip2depthView * p_depthClip;

    // next, apply the vert's Z value for that ground offset.
    // TODO: This calculation is not right!
    //       I think perhaps the model matrix is not earth-aligned.
    p_depthView.z += gl_Vertex.z*gl_Vertex.w/p_depthView.w;

    // then transform the vert back into camera view space.
    VertexVIEW = oe_clamp_depthView2cameraView * p_depthView;
#else
    // transform the depth-clip point back into camera view coords.
    VertexVIEW = oe_clamp_depthClip2cameraView * p_depthClip;
#endif
}