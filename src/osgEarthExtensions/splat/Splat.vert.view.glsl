#version 330

#pragma vp_entryPoint "oe_splat_vertex_view"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.5"

#pragma include "Splat.types.glsl"

out vec4 oe_layer_tilec;
out float oe_splat_range;
out vec2 oe_splat_covtc;

flat out vec3 oe_splat_scaleBias0;
flat out vec3 oe_splat_scaleBias1;
flat out int oe_splat_lodIndex;

uniform mat4 $COVERAGE_TEXMAT_UNIFORM;   // assigned at runtime
uniform float oe_splat_scaleOffset;
uniform vec4 oe_tile_key;


vec3 calcScaleBias(in float lod)
{
    vec3 result = vec3(0);

    float dL = oe_tile_key.z - lod;
    float factor = exp2(dL);
    float invFactor = 1.0/factor;
    result.x = invFactor;

    // For upsampling we need to calculate an offset as well
    if ( factor >= 1.0 )
    {
        vec2 a = floor(oe_tile_key.xy * invFactor);
        vec2 b = a * factor;
        vec2 c = (a+1.0) * factor;
        result.yz = (oe_tile_key.xy-b)/(c-b);
    }

    return result;
}


void oe_splat_vertex_view(inout vec4 VertexVIEW)
{
    // quantize the scale offset so we take the hit in the FS
    float scaleOffset = oe_splat_scaleOffset >= 0.0 ? ceil(oe_splat_scaleOffset) : floor(oe_splat_scaleOffset);

    // range from camera to vertex
    oe_splat_range = -VertexVIEW.z;

    // Choose the best range based on distance to camera.
    oe_splat_range = clamp(oe_splat_range, oe_SplatRanges[0], oe_SplatRanges[RANGE_COUNT-1]);
    
    for(int i=0; i<RANGE_COUNT-1; ++i)
    {
        if ( oe_splat_range >= oe_SplatRanges[i] && oe_splat_range <= oe_SplatRanges[i+1] )
        {            
            float lod0 = oe_SplatLevels[i] + scaleOffset;
            oe_splat_scaleBias0 = calcScaleBias(lod0);

            float lod1 = oe_SplatLevels[i+1] + scaleOffset;
            oe_splat_scaleBias1 = calcScaleBias(lod1);

            oe_splat_lodIndex = i;

            break;
        }
    }

    // calculate the coverage sampling coordinates. The texture matrix accounts
    // for any super-sampling that might be in effect for the current LOD.
    oe_splat_covtc = ($COVERAGE_TEXMAT_UNIFORM * oe_layer_tilec).st;

}
