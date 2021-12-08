#version 460
#pragma include RexEngine.Types.GL4.glsl
#pragma vp_name REX Engine - Init Model Space
#pragma vp_function oe_rex_init_model, vertex_model, first

// uniforms
uniform vec4 oe_terrain_color;

// outputs
out vec4 vp_Color;
//out vec4 oe_layer_tilec;
out vec4 oe_terrain_tessLevel;
out float oe_rex_morphFactor;

void oe_rex_init_model(inout vec4 vertexModel)
{
    // instance ID from the DrawElementsIndirect cmd
    // if we go with MDEI, change this to gl_DrawID
    oe_tileID = gl_InstanceID;

    //// derive texture coordinates, assuming 17 size tile
    //float u = float(gl_VertexID % OE_TILE_SIZE) / float(OE_TILE_SIZE - 1);
    //float v = float(gl_VertexID / OE_TILE_SIZE) / float(OE_TILE_SIZE - 1);
    //oe_layer_tilec = vec4(u, v, 0, 1);

    // Color of the underlying map geometry (untextured)
    vp_Color = oe_terrain_color;

    // initialize:
    oe_rex_morphFactor = 0.0;
    
    // Default tessellation level (where applicable)
    oe_terrain_tessLevel = vec4(1);
}


[break]

#version 460
#pragma include RexEngine.Types.GL4.glsl
#pragma vp_name REX Engine - Init View Space
#pragma vp_function oe_rex_init_view, vertex_view, first

// outputs
out vec4 oe_layer_tilec;
out vec3 vp_Normal;
out vec3 oe_UpVectorView;
flat out int oe_terrain_vertexMarker;

void oe_rex_init_view(inout vec4 vert_view)
{
    // extract vertex and its marker (in w)
    vec4 vdata = tile[oe_tileID].verts[gl_VertexID];
    vert_view = vec4(vdata.xyz, 1);

    // assign vertex marker flags
    oe_terrain_vertexMarker = int(vdata.w);

    // extract normal
    vp_Normal = tile[oe_tileID].normals[gl_VertexID].xyz;

    // extract tile UV (global data)
    oe_layer_tilec = vec4(global.uvs[gl_VertexID], 0, 1);

    // "up" vector at this vertex in view space, which we will later
    // need in order to elevate the terrain. vp_Normal can change later
    // but UpVectorView will stay the same.
    oe_UpVectorView = vp_Normal;
}
