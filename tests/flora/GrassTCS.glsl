#version 400 compatibility

/**
 * Flora TCS that distributes data EVERYWHERE with no filter.
 */

#pragma vp_entryPoint "oe_grass_configureTess"
#pragma vp_location   "tess_control"

layout(vertices = 3) out;

uniform float oe_grass_density;
                
void oe_grass_configureTess()
{
	if (gl_InvocationID == 0)
	{
        gl_TessLevelOuter[0] = oe_grass_density;
        gl_TessLevelOuter[1] = oe_grass_density;
        gl_TessLevelOuter[2] = oe_grass_density;
        gl_TessLevelInner[0] = oe_grass_density;
    }
}
