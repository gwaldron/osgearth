#version 460
#pragma include RexEngine.GL4.glsl
#pragma vp_name REX Engine - Init Model Space
#pragma vp_function oe_rex_init_model, vertex_model, first

// uniforms
uniform vec4 oe_terrain_color;

// outputs
out vec4 vp_Color;
out float oe_rex_morphFactor;
out vec4 oe_terrain_tessLevel;

void oe_rex_init_model(inout vec4 vertexModel)
{
    // instance ID from the DrawElementsIndirect cmd
    // if we go with MDEI, change this to gl_DrawID
    oe_tileID = gl_InstanceID;

    // Color of the underlying map geometry (untextured)
    vp_Color = oe_terrain_color;

    // initialize:
    oe_rex_morphFactor = 0.0;
    
    // Default tessellation level (where applicable)
    oe_terrain_tessLevel = vec4(1);
}


[break]

#version 460
#pragma include RexEngine.GL4.glsl
#pragma vp_name REX Engine - Init View Space
#pragma vp_function oe_rex_init_view, vertex_view, first

// outputs
out vec4 oe_layer_tilec;
out vec3 vp_Normal;
out vec3 oe_UpVectorView;
flat out int oe_terrain_vertexMarker;

// stage globals
vec4 oe_tile_key;
mat4 oe_tile_mvm;

void oe_rex_init_view(inout vec4 vert_view)
{
    oe_tile_mvm = oe_tile[oe_tileID].modelViewMatrix;

    // extract vertex and its marker (in w)
    vec4 vdata = oe_tile[oe_tileID].verts[gl_VertexID];
    vert_view = oe_tile_mvm * vec4(vdata.xyz, 1);

    // assign vertex marker flags
    oe_terrain_vertexMarker = int(vdata.w);

    // extract normal
    vp_Normal = mat3(oe_tile_mvm) * oe_tile[oe_tileID].normals[gl_VertexID].xyz;

    // extract tile UV (global data)
    oe_layer_tilec = vec4(oe_global.uvs[gl_VertexID], 0, 1);

    // the tile key
    oe_tile_key = oe_tile[oe_tileID].tileKey;

    // "up" vector at this vertex in view space, which we will later
    // need in order to elevate the terrain. vp_Normal can change later
    // but UpVectorView will stay the same.
    oe_UpVectorView = vp_Normal;
}
