// note: this is an include file

// depth texture captures by the clamping technique
uniform sampler2D oe_clamp_depthTex;

// matrix transforming from view space to depth-texture clip space
uniform mat4 oe_clamp_cameraView2depthClip;

// matrix transform from depth-tecture clip space to view space
uniform mat4 oe_clamp_depthClip2cameraView;

// Given a vertex in view space, clamp it to the "ground" as represented
// by an orthographic depth texture. Return the clamped vertex in view space,
// along with the associated depth value.
void oe_getClampedViewVertex(in vec4 vertView, out vec4 out_clampedVertView, out float out_depth)
{
    // transform the vertex into the depth texture's clip coordinates.
    vec4 vertDepthClip = oe_clamp_cameraView2depthClip * vertView;

    // sample the depth map
    out_depth = texture2DProj( oe_clamp_depthTex, vertDepthClip ).r;

    // now transform into depth-view space so we can apply the height-above-ground:
    vec4 clampedVertDepthClip = vec4(vertDepthClip.x, vertDepthClip.y, out_depth, 1.0);

    // convert back into view space.
    out_clampedVertView = oe_clamp_depthClip2cameraView * clampedVertDepthClip;
}

// Returns a vector indicating the "down" direction.
void oe_getClampingUpVector(out vec3 up)
{
    up = normalize(mat3(oe_clamp_depthClip2cameraView) * vec3(0,0,-1));
}
