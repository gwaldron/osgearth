#version 400

#pragma vp_name       REX Engine TCS
#pragma vp_entryPoint oe_terrain_tcs
#pragma vp_location   tess_control
#pragma vp_order      0

layout(vertices=3) out;

// per-vertex tile coordinates
vec4 oe_layer_tilec;

uniform float dd;

// MAIN ENTRY POINT                
void oe_terrain_tcs()
{
    if (gl_InvocationID == 0)
    {
        float d = 1;

        gl_TessLevelOuter[0] = d;
        gl_TessLevelOuter[1] = d;
        gl_TessLevelOuter[2] = d;
        gl_TessLevelInner[0] = d;
    }
}
