#version 400

/**
 * TCS that assigns a patch grid density.
 */
 
#pragma vp_name       LandCover tessellation control shader
#pragma vp_entryPoint oe_landcover_configureTess
#pragma vp_location   tess_control

layout(vertices=3) out;

uniform float oe_landcover_density;

// per-vertex tile coordinates
vec4 oe_layer_tilec;

// SDK function to sample the coverage data
int oe_landcover_getBiomeIndex(in vec4);

// SDK function to load per-vertex data
void VP_LoadVertex(in int);

// MAIN ENTRY POINT                
void oe_landcover_configureTess()
{
	if (gl_InvocationID == 0)
	{
        float d = oe_landcover_density;

        VP_LoadVertex(0);
        if ( oe_landcover_getBiomeIndex(oe_layer_tilec) >= 0 ) {
            d = oe_landcover_density;
        }
        else {
            VP_LoadVertex(1);
            if ( oe_landcover_getBiomeIndex(oe_layer_tilec) >= 0 ) {
                d = oe_landcover_density;
                VP_LoadVertex(0);
            }
            else {
                VP_LoadVertex(2);
                if ( oe_landcover_getBiomeIndex(oe_layer_tilec) >= 0 ) {
                    d = oe_landcover_density;
                    VP_LoadVertex(0);
                }
            }
        }

        gl_TessLevelOuter[0] = d;
        gl_TessLevelOuter[1] = d;
        gl_TessLevelOuter[2] = d;
        gl_TessLevelInner[0] = d;
	}
}
