#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_normalMapVertex
#pragma vp_location   vertex_view
#pragma vp_order      0.5

#pragma import_defines(OE_TERRAIN_RENDER_NORMAL_MAP)

uniform mat4 oe_tile_normalTexMatrix;
uniform vec2 oe_tile_elevTexelCoeff;

// stage globals
vec4 oe_layer_tilec;

out vec2 oe_normalMapCoords;
out vec3 oe_normalMapBinormal;

void oe_normalMapVertex(inout vec4 unused)
{
#ifndef OE_TERRAIN_RENDER_NORMAL_MAP
    return;
#endif

    // calculate the sampling coordinates for the normal texture
    //oe_normalMapCoords = (oe_tile_normalTexMatrix * oe_layer_tilec).st;
    
    oe_normalMapCoords = oe_layer_tilec.st
        * oe_tile_elevTexelCoeff.x * oe_tile_normalTexMatrix[0][0]
        + oe_tile_elevTexelCoeff.x * oe_tile_normalTexMatrix[3].st
        + oe_tile_elevTexelCoeff.y;

    // send the bi-normal to the fragment shader
    oe_normalMapBinormal = normalize(gl_NormalMatrix * vec3(0,1,0));
}
