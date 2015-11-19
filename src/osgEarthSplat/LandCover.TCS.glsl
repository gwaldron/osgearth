#version 400

/**
 * TCS that assigns a patch grid density.
 */
 
#pragma vp_name       LandCover tessellation control shader
#pragma vp_entryPoint oe_landcover_configureTess
#pragma vp_location   tess_control

layout(vertices=3) out;

uniform float oe_landcover_density;

// MAIN ENTRY POINT                
void oe_landcover_configureTess()
{
	if (gl_InvocationID == 0)
	{
        float d = oe_landcover_density;

        gl_TessLevelOuter[0] = d;
        gl_TessLevelOuter[1] = d;
        gl_TessLevelOuter[2] = d;
        gl_TessLevelInner[0] = d;
	}
}
