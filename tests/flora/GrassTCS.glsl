#version 400 compatibility

#pragma vp_entryPoint "oe_grass_configureTess"
#pragma vp_location   "tess_control"

layout(vertices = 3) out;

uniform float oe_grass_density = 1.0;

uniform sampler2D oe_layer_tex;
uniform mat4 oe_layer_texMatrix;
in vec4 oe_layer_tilec;

                
void oe_grass_configureTess()
{
	if (gl_InvocationID == 0)
	{
        // Testing: masking
        vec3 rgb = texture(oe_layer_tex, (oe_layer_texMatrix*oe_layer_tilec).st).rgb;
        
        if ( rgb.b < 0.3 )
        {
            gl_TessLevelOuter[0] = oe_grass_density;
            gl_TessLevelOuter[1] = oe_grass_density;
            gl_TessLevelOuter[2] = oe_grass_density;
            gl_TessLevelInner[0] = oe_grass_density;
        }
        else
        {
            gl_TessLevelOuter[0] = 0;
            gl_TessLevelOuter[1] = 0;
            gl_TessLevelOuter[2] = 0;
            gl_TessLevelInner[0] = 0;
        }
	}
}
