#version 400 compatibility

/**
 * Flora TCS that distributes data EVERYWHERE with no filter.
 */

#pragma vp_entryPoint "oe_flora_configureTess"
#pragma vp_location   "tess_control"

layout(vertices = 3) out;

uniform float oe_flora_density;
                
void oe_flora_configureTess()
{
	if (gl_InvocationID == 0)
	{
        gl_TessLevelOuter[0] = oe_flora_density;
        gl_TessLevelOuter[1] = oe_flora_density;
        gl_TessLevelOuter[2] = oe_flora_density;
        gl_TessLevelInner[0] = oe_flora_density;
    }
}
