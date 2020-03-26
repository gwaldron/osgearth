#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       REX Engine - Init Model Space
#pragma vp_entryPoint oe_rex_init_model
#pragma vp_location   vertex_model
#pragma vp_order      first

// uniforms
uniform vec4 oe_terrain_color;

// outputs
out vec4 vp_Color;
out vec4 oe_layer_tilec;
out vec4 oe_terrain_tessLevel;

out float oe_rex_morphFactor;
flat out int oe_terrain_vertexMarker;

void oe_rex_init_model(inout vec4 vertexModel)
{
    // Texture coordinate for the tile (always 0..1)
    oe_layer_tilec = gl_MultiTexCoord0;

    // Color of the underlying map geometry (untextured)
    vp_Color = oe_terrain_color;

    // initialize:
    oe_rex_morphFactor = 0.0;

    // Extract the vertex type marker
    oe_terrain_vertexMarker = int(oe_layer_tilec.z);

    // Default tessellation level (where applicable)
    oe_terrain_tessLevel = vec4(1);
}


[break]
#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       REX Engine - Init View Space
#pragma vp_entryPoint oe_rex_init_view
#pragma vp_location   vertex_view
#pragma vp_order      first

// outputs
vec3 vp_Normal;
out vec3 oe_UpVectorView;

void oe_rex_init_view(inout vec4 vertexView)
{
    // "up" vector at this vertex in view space, which we will later
    // need in order to elevate the terrain. vp_Normal can change later
    // but UpVectorView will stay the same.
    oe_UpVectorView = vp_Normal;
}
