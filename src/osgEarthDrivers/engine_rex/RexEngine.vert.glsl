#version 330 compatibility

#pragma vp_name       "REX Engine - Vertex"
#pragma vp_entryPoint "oe_rexEngine_vert"
#pragma vp_location   "vertex_model"
#pragma vp_order      "0.0"
#pragma vp_define     "OE_REX_USE_TERRAIN_COLOR"

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

#ifdef OE_REX_USE_TERRAIN_COLOR
    vp_Color = oe_terrain_color;
#endif
	
	oe_UpVectorView = normalize(gl_NormalMatrix*vp_Normal);

    // initialize:
    oe_rex_morphFactor = 0.0;
}
