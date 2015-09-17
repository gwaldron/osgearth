#version 400 compatibility

/**
 * Flora TCS that distributes data based on a Land Use code sampler.
 */
 
#pragma vp_entryPoint "oe_flora_configureTess"
#pragma vp_location   "tess_control"

layout(vertices = 3) out;

uniform float oe_flora_density;

// Land Use Codes from the GLOBCOVER ESA dataset
#define LC_FOREST_LOW  40
#define LC_FOREST_HIGH 100

// Land Use sampler+matrix
uniform sampler2D landUseTex;
uniform mat4      landUseTexMatrix;

in vec4 oe_layer_tilec;

// MAIN ENTRY POINT                
void oe_flora_configureTess()
{
	if (gl_InvocationID == 0)
	{
        //TODO: sample at each corner and adjust tess levels individually
        
        float code = textureLod(landUseTex, (landUseTexMatrix * oe_layer_tilec).st, 0).r;

        if ( code >= LC_FOREST_LOW && code <= LC_FOREST_HIGH )
        {
            gl_TessLevelOuter[0] = oe_flora_density;
            gl_TessLevelOuter[1] = oe_flora_density;
            gl_TessLevelOuter[2] = oe_flora_density;
            gl_TessLevelInner[0] = oe_flora_density;
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
