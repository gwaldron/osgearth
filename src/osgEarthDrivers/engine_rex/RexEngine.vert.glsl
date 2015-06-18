#version 330 compatibility

#pragma vp_name       "REX Engine - Vertex"
#pragma vp_entryPoint "oe_rexEngine_vert"
#pragma vp_location   "vertex_model"
#pragma vp_order      "0.5"
#pragma vp_define     "OE_REX_USE_TERRAIN_COLOR"

// uniforms
uniform mat4 oe_layer_texMatrix;
uniform vec4 oe_terrain_color;

// outputs
out vec4 oe_layer_texc;
out vec4 oe_layer_tilec;
out vec4 vp_Color;
out vec3 vp_UpVector;
out vec3 vp_Normal;

void oe_rexEngine_vert(inout vec4 vertexModel)
{
    // Texture coordinate for the tile (always 0..1)
    oe_layer_tilec = gl_MultiTexCoord0;

    // Texture coordinate for the color texture (scale,bias)
    oe_layer_texc = oe_layer_texMatrix * oe_layer_tilec;

#ifdef OE_REX_USE_TERRAIN_COLOR
    vp_Color = oe_terrain_color;
#endif
	
	vp_UpVector = vp_Normal;
}
