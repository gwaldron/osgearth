#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_depthOffset_vertex
#pragma vp_location   vertex_view
#pragma vp_order      0.8

uniform float oe_depthOffset_minBias;
uniform float oe_depthOffset_maxBias;
uniform float oe_depthOffset_minRange;
uniform float oe_depthOffset_maxRange;

void oe_depthOffset_vertex(inout vec4 vertexView)
{
    // calculate range to target:
    float range = length(vertexView.xyz);

    // calculate the depth offset bias for this range:
    float ratio = (clamp(range, oe_depthOffset_minRange, oe_depthOffset_maxRange)-oe_depthOffset_minRange)/(oe_depthOffset_maxRange-oe_depthOffset_minRange);
    float bias = oe_depthOffset_minBias + ratio * (oe_depthOffset_maxBias-oe_depthOffset_minBias);

	// clamp the bias to 1/2 of the range of the vertex. We don't want to 
    // pull the vertex TOO close to the camera and certainly not behind it.
    bias = min(bias, range*0.5);

    //   pull the vertex towards the camera.
    vec3 pullVec = normalize(vertexView.xyz);
    vec3 simVert3 = vertexView.xyz - pullVec*bias;
    vertexView = vec4(simVert3, 1.0);
}