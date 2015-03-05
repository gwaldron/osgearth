#version 110

#pragma vp_entryPoint "oe_bumpmap_vertexModel"
#pragma vp_location   "vertex_model"
#pragma vp_order      "0.5"
#pragma vp_define     "OE_USE_NORMAL_MAP"

uniform vec4 oe_tile_key;
uniform float oe_bumpmap_scale;

varying vec4 oe_layer_tilec;
varying vec3 oe_Normal;

varying vec2 oe_bumpmap_coords;
varying float oe_bumpmap_range;

#ifdef OE_USE_NORMAL_MAP
uniform mat4 oe_nmap_normalTexMatrix;
varying vec4 oe_bumpmap_normalCoords;
#else
varying float oe_bumpmap_slope;
#endif

vec2 oe_bumpmap_scaleCoords(in vec2 coords, in float targetLOD)
{
    float dL = oe_tile_key.z - targetLOD;
    float factor = exp2(dL);
    float invFactor = 1.0/factor;
    vec2 scale = vec2(invFactor);
    vec2 result = coords * scale;

    // For upsampling we need to calculate an offset as well
    float upSampleToggle = factor >= 1.0 ? 1.0 : 0.0;
    {
        vec2 a = floor(oe_tile_key.xy * invFactor);
        vec2 b = a * factor;
        vec2 c = (a+1.0) * factor;
        vec2 offset = (oe_tile_key.xy-b)/(c-b);
        result += upSampleToggle * offset;
    }

    return result;
}

void oe_bumpmap_vertexModel(inout vec4 VertexMODEL)
{            
    // quantize the scale factor
    float iscale = float(int(oe_bumpmap_scale));

    // scale sampling coordinates to a target LOD.
    const float targetLOD = 13.0;
    oe_bumpmap_coords = oe_bumpmap_scaleCoords(oe_layer_tilec.st, targetLOD) * iscale;

#ifdef OE_USE_NORMAL_MAP
    oe_bumpmap_normalCoords = oe_nmap_normalTexMatrix * oe_layer_tilec;
#else
    // calcluate slope and augment it.
    oe_bumpmap_slope = clamp(2.5*(1.0-oe_Normal.z), 0.0, 1.0);
#endif
}
