#version 400

/**
 * Flora TCS that distributes data based on a Land Use code sampler.
 */
 
#pragma vp_entryPoint "oe_flora_configureTess"
#pragma vp_location   "tess_control"

layout(vertices = 3) out;

uniform float oe_flora_density;

// Land Use Codes from the GLOBCOVER ESA dataset

#define GC_FOREST_DECIDUOUS_SPARSE 40
#define GC_FOREST_DECIDUOUS_MEDIUM 50
#define GC_FOREST_DECIDUOUS_DENSE  60
#define GC_FOREST_NEEDLED_SPARSE   70
#define GC_FOREST_NEEDLED_MEDIUM   90
#define GC_FOREST_NEEDLED_DENSE    80
#define GC_FOREST_MIXED           100

#define GC_GRASSLAND_SPARSE       110
#define GC_GRASSLAND_DENSE        120

#define GC_SHRUBLAND_DENSE        130
#define GC_SHRUBLAND_MEDIUM       140
#define GC_SHURBLAND_SPARSE       150

#define GC_SWAMPLAND_SPARSE       160
#define GC_SWAMPLAND_DENSE        170
#define GC_SWAMPLAND_MEDIUM       180

#define GC_URBAN                  190
#define GC_DESERT                 200
#define GC_WATER                  210
#define GC_TUNDRA                 220
#define GC_NODATA                 230


// Land Use sampler+matrix (set these in your earth file)
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
        
        if ( code >= GC_FOREST_DECIDUOUS_SPARSE && code <= GC_FOREST_MIXED )
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
