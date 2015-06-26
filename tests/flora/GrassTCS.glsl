#version 400 compatibility

#pragma vp_entryPoint "oe_grass_configureTess"
#pragma vp_location   "tess_control"

layout(vertices = 3) out;

uniform float oe_grass_density = 1.0;

in vec4 oe_layer_tilec;

uniform sampler2D floraColor;
uniform mat4      floraMatrix;


#define USE_LAND_USE

#ifdef USE_LAND_USE
uniform sampler2D landUseTex;
uniform mat4      landUseTexMatrix;

float getLandUseCode()
{
    return textureLod(landUseTex, (landUseTexMatrix * oe_layer_tilec).st, 0).r;
}
#endif


const mat3 toYUV = mat3(0.299, 0.587, 0.114, -0.14713, -0.28886, 0.436, 0.615, -0.51499, -0.10001);

uniform float oe_grass_coverage;

                
void oe_grass_configureTess()
{
	if (gl_InvocationID == 0)
	{
        float d;
        
#ifdef USE_LAND_USE    
        d = 2.0;
        float code = getLandUseCode();
        if ( code >= 40.0 && code < 50 )
#endif
        {
            // Try to see how "green" the ground is, and threshold the flora based on that
            vec3 green = toYUV * vec3(0,1,0);
            vec3 color = toYUV * texture(floraColor, (floraMatrix*oe_layer_tilec).st).rgb;
            d = distance(color, green);
        }
        
        if ( d <= oe_grass_coverage )
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
