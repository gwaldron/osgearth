#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_fog_vertex
#pragma vp_location   vertex_view

uniform int oe_fog_algo;

varying float oe_fogFactor;

void oe_fog_vertex(inout vec4 vertexVIEW)
{
    float z = length( vertexVIEW.xyz );

	// linear fog
	if (oe_fog_algo == 0)
	{
	  oe_fogFactor = clamp((gl_Fog.end - z) / (gl_Fog.end - gl_Fog.start), 0.0, 1.0);
	}
	// exp fog
	else if (oe_fog_algo == 1)
	{	
	  oe_fogFactor = clamp(exp( -gl_Fog.density * z ), 0.0, 1.0);
	}	
	else
	// exp2 fog
	{
        const float LOG2 = 1.442695;
        oe_fogFactor = clamp(exp2( -gl_Fog.density * gl_Fog.density * z * z * LOG2 ), 0.0, 1.0);
	}	
}