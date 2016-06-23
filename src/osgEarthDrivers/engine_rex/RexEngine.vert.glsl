#version $GLSL_VERSION_STR

#pragma vp_name       REX Engine - Vertex
#pragma vp_entryPoint oe_rexEngine_vert
#pragma vp_location   vertex_model
#pragma vp_order      0.0

// uniforms
uniform vec4 oe_terrain_color;

// outputs
out vec4 vp_Color;
out vec3 vp_Normal;

out vec4 oe_layer_texc;
out vec4 oe_layer_tilec;
out vec3 oe_UpVectorView;

out float oe_rex_morphFactor;

void oe_rexEngine_vert(inout vec4 vertexModel)
{
    // Texture coordinate for the tile (always 0..1)
    oe_layer_tilec = gl_MultiTexCoord0;

    // Color of the underlying map geometry (untextured)
    vp_Color = oe_terrain_color;
	
    // "up" vector at this vertex in view space, which we will later
    // need in order to elevate the terrain
	oe_UpVectorView = normalize(gl_NormalMatrix*vp_Normal);

    // initialize:
    oe_rex_morphFactor = 0.0;
}
