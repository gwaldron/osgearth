#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name REX Engine - Init Model Space
#pragma vp_function oe_rex_init_model, vertex_model, first
#pragma import_defines(OE_INDIRECT)

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
#ifdef OE_INDIRECT
    float u = float(gl_VertexID % 17) / 16.0;
    float v = float(gl_VertexID / 17) / 16.0;
    oe_layer_tilec = vec4(u, v, 1, 1);
#else
    oe_layer_tilec = gl_MultiTexCoord0;
#endif

    // Extract the vertex type marker
    oe_terrain_vertexMarker = int(oe_layer_tilec.z);

    // Color of the underlying map geometry (untextured)
    vp_Color = oe_terrain_color;

    // initialize:
    oe_rex_morphFactor = 0.0;
    
    // Default tessellation level (where applicable)
    oe_terrain_tessLevel = vec4(1);
}


[break]
#version 430

#pragma vp_name REX Engine - Init View Space
#pragma vp_function oe_rex_init_view, vertex_view, first
#pragma import_defines(OE_INDIRECT)

// outputs
vec3 vp_Normal;
out vec3 oe_UpVectorView;

#ifdef OE_INDIRECT
struct Tile {
    vec4 verts[289];
    vec4 normals[289];
    mat4 modelViewMatrix;
    mat4 colorMat;
    int colorIndex;
    float padding[3];
};
layout(binding=0, std430) readonly restrict buffer TileBuffer {
    Tile tile[];
};
#endif

void oe_rex_init_view(inout vec4 vert_view)
{
#ifdef OE_INDIRECT
    int i = gl_InstanceID; // gl_DrawID;
    vert_view = tile[i].verts[gl_VertexID];
    //.vert_view = tile[i].modelViewMatrix * tile[i].verts[gl_VertexID];
    vp_Normal = tile[i].normals[gl_VertexID].xyz;
    //vp_Normal = mat3(tile[i].modelViewMatrix) * tile[i].normals[gl_VertexID].xyz;
#endif

    // "up" vector at this vertex in view space, which we will later
    // need in order to elevate the terrain. vp_Normal can change later
    // but UpVectorView will stay the same.
    oe_UpVectorView = vp_Normal;
}
