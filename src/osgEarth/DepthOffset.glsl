#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_DepthOffset_vertex
#pragma vp_location   vertex_view
#pragma vp_order      0.8

uniform vec4 oe_DepthOffset_params;

void oe_DepthOffset_vertex(inout vec4 vertexView)
{
    // calculate range to target:
    float range = length(vertexView.xyz);

    // extract params for clarity.
    float minBias = oe_DepthOffset_params[0];
    float maxBias = oe_DepthOffset_params[1];
    float minRange = oe_DepthOffset_params[2];
    float maxRange = oe_DepthOffset_params[3];

    // calculate the depth offset bias for this range:
    float ratio = (clamp(range, minRange, maxRange)-minRange)/(maxRange-minRange);
    float bias = minBias + ratio * (maxBias-minBias);

	// clamp the bias to 1/2 of the range of the vertex. We don't want to 
    // pull the vertex TOO close to the camera and certainly not behind it.
    bias = min(bias, range*0.5);
    bias = min(bias, maxBias);

    // pull the vertex towards the camera.
    vec3 pullVec = normalize(vertexView.xyz);
    vec3 simVert3 = vertexView.xyz - pullVec*bias;
    vertexView = vec4(simVert3, 1.0);
}
