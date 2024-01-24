#pragma vp_name REX Engine - Init Model Space
#pragma vp_function oe_rex_init_model, vertex_model, first

// uniforms
uniform vec4 oe_terrain_color;
uniform vec4 oe_tile_key_u;

// outputs
out vec4 vp_Color;
out vec4 oe_layer_tilec;
out vec4 oe_terrain_tessLevel;
out float oe_rex_morphFactor;
flat out int oe_terrain_vertexMarker;

// stage globals
vec4 oe_tile_key;

void oe_rex_init_model(inout vec4 vertexModel)
{
    // Texture coordinate for the tile (always 0..1)
    oe_layer_tilec = gl_MultiTexCoord0;

    // Extract the vertex type marker
    //oe_terrain_vertexMarker = int(oe_layer_tilec.z);

    // Doing this instead of the commented-out line above is a workaround
    // for the Mesa driver bug described here:
    // https://gitlab.freedesktop.org/mesa/mesa/-/issues/10482
    oe_terrain_vertexMarker = int(gl_MultiTexCoord0.z);

    // Color of the underlying map geometry (untextured)
    vp_Color = oe_terrain_color;

    // initialize:
    oe_rex_morphFactor = 0.0;

    // tile key
    oe_tile_key = oe_tile_key_u;
    
    // Default tessellation level (where applicable)
    oe_terrain_tessLevel = vec4(1);
}


[break]
#pragma vp_name REX Engine - Init View Space
#pragma vp_function oe_rex_init_view, vertex_view, first

// outputs
out vec3 vp_Normal;
out vec3 oe_UpVectorView;

void oe_rex_init_view(inout vec4 vert_view)
{
    // "up" vector at this vertex in view space, which we will later
    // need in order to elevate the terrain. vp_Normal can change later
    // but UpVectorView will stay the same.
    oe_UpVectorView = vp_Normal;
}
