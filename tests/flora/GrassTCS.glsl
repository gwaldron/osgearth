#version 400 compatibility

#pragma vp_entryPoint "oe_grass_configureTess"
#pragma vp_location   "tess_control"

layout(vertices = 3) out;

uniform float oe_grass_density = 1.0;

uniform sampler2D floraColor;
uniform mat4      floraMatrix;

in vec4 oe_layer_tilec;

const mat3 toYUV = mat3(0.299, 0.587, 0.114, -0.14713, -0.28886, 0.436, 0.615, -0.51499, -0.10001);

uniform float oe_grass_coverage;
                
void oe_grass_configureTess()
{
	if (gl_InvocationID == 0)
	{
        // Try to see how "green" the ground is, and threshold the flora based on that
        vec3 green = toYUV * vec3(0,1,0);
        vec3 color = toYUV * texture(floraColor, (floraMatrix*oe_layer_tilec).st).rgb;
        float d = distance(color, green);
        
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
