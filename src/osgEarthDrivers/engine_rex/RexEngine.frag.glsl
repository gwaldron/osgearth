#version 330

#pragma vp_name       "REX Engine - Fragment"
#pragma vp_entryPoint "oe_rexEngine_frag"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.5"
#pragma vp_define     "OE_REX_USE_BLENDING"

uniform sampler2D oe_layer_tex;
uniform int       oe_layer_uid;
uniform int       oe_layer_order;
uniform float     oe_layer_opacity;

in vec4 oe_layer_texc;
in vec4 flerp;

void oe_rexEngine_frag(inout vec4 color)
{
    float applyImagery = oe_layer_uid >= 0 ? 1.0 : 0.0;

#if 1
    vec4 texel = mix(color, texture2D(oe_layer_tex, oe_layer_texc.st), applyImagery);
    texel.a = mix(texel.a, texel.a*oe_layer_opacity, applyImagery);
#else
    // Poor man's LOD blending.
    vec4 t0 = texture(oe_layer_tex, oe_layer_texc.st, 1.0);
    vec4 t1 = texture(oe_layer_tex, oe_layer_texc.st);
    vec4 texel = mix(color, mix(t0, t1, 1.0-flerp.x), applyImagery);
#endif

    float firstLayer = oe_layer_order == 0 ? 1.0 : 0.0;

#ifdef OE_REX_USE_BLENDING
    color = mix(texel, texel*texel.a + color*(1.0-texel.a), firstLayer);
#else
    color = texel;
#endif
	//color.xyz = (flerp.xyz);
	//return;
	//if (flerp.x>0.95)
	//{
	//	color.xyz = vec3(1,0,0);
	//}
	//color.xyz = vec3(0,1,0);
	
	//if (oe_tile_key.z == 10)
	//{
	//	color.xyz = vec3(1,0,0)*flerp.x;
	//}
	//else if (oe_tile_key.z == 11)
	//{
	//	color.xyz = vec3(0,1,0)*flerp.x;
	//}
	//else if (oe_tile_key.z == 12)
	//{
	//	color.xyz = vec3(0,0,1)*flerp.x;
	//}
	//else if (oe_tile_key.z == 13)
	//{
	//	color.xyz = vec3(1,1,0)*flerp.x;
	//}
	//else if (oe_tile_key.z == 14)
	//{
	//	color.xyz = vec3(1,0,1)*flerp.x;
	//}
	//else if (oe_tile_key.z == 15)
	//{
	//	color.xyz = vec3(0,1,1)*flerp.x;
	//}
}
