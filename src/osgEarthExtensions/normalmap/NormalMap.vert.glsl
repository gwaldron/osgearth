#version 110

uniform vec4 oe_tile_key;
uniform float oe_nmap_scale;
varying vec4 oe_layer_tilec;
varying vec3 oe_Normal;
varying vec2 oe_nmap_coords;
varying float oe_nmap_slope;

vec2 oe_nmap_scaleCoords(in vec2 coords, in float targetLOD)
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

void oe_nmap_vertex(inout vec4 VertexMODEL)
{            
    // quantize the scale factor
    float iscale = float(int(oe_nmap_scale));

    // scale sampling coordinates to a target LOD.
    const float targetLOD = 13.0;
    oe_nmap_coords = oe_nmap_scaleCoords(oe_layer_tilec.st, targetLOD) * iscale;

    // calcluate slope and augment it.
    oe_nmap_slope = clamp(2.5*(1.0-oe_Normal.z), 0.0, 1.0);
}