#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_bumpmap_vertexView
#pragma vp_location   vertex_view
#pragma vp_order      0.5

uniform vec4 oe_tile_key;
uniform float oe_bumpmap_scale;
uniform float oe_bumpmap_baseLOD;

out vec4 oe_layer_tilec;
out vec3 vp_Normal;

out vec2 oe_bumpmap_coords;
out float oe_bumpmap_range;
flat out mat3 oe_bumpmap_normalMatrix;


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

void oe_bumpmap_vertexView(inout vec4 vertexView)
{
    oe_bumpmap_range = -vertexView.z;

    // quantize the scale factor
    float iscale = float(int(oe_bumpmap_scale));

    // scale sampling coordinates to a target LOD.
    oe_bumpmap_coords = oe_bumpmap_scaleCoords(oe_layer_tilec.st, floor(oe_bumpmap_baseLOD)) * iscale;

    // propagate normal matrix to fragment stage
    oe_bumpmap_normalMatrix = gl_NormalMatrix;
}
