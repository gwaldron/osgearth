#version 410

#pragma vp_name       LandCover TES Shader
#pragma vp_entryPoint oe_landcover_tessellate
#pragma vp_location   tess_eval

// osgEarth terrain is always CCW winding
layout(triangles, fractional_odd_spacing, ccw) in;

// Internal helpers:
void VP_Interpolate3();
void VP_EmitVertex();

float VP_Interpolate3(float a, float b, float c) 
{
    return dot(gl_TessCoord.xyz, vec3(a,b,c));
}

vec2 VP_Interpolate3(vec2 a, vec2 b, vec2 c) 
{
    return vec2(dot(gl_TessCoord.xyz, vec3(a.x,b.x,c.x)),
	dot(gl_TessCoord.xyz, vec3(a.y,b.y,c.y)));
}

vec3 VP_Interpolate3(vec3 a, vec3 b, vec3 c) 
{
    return vec3(dot(gl_TessCoord.xyz, vec3(a.x,b.x,c.x)),
	dot(gl_TessCoord.xyz, vec3(a.y,b.y,c.y)),
	dot(gl_TessCoord.xyz, vec3(a.z,b.z,c.z)));
}

vec4 VP_Interpolate3(vec4 a, vec4 b, vec4 c) 
{
    return vec4(dot(gl_TessCoord.xyz, vec3(a.x,b.x,c.x)),
				dot(gl_TessCoord.xyz, vec3(a.y,b.y,c.y)),
				dot(gl_TessCoord.xyz, vec3(a.z,b.z,c.z)),
				dot(gl_TessCoord.xyz, vec3(a.w,b.w,c.w)));
}

                
vec3 vp_Normal;

// simplest possible pass-though:
void oe_landcover_tessellate()
{
    VP_Interpolate3();
    // Must re-normalize the normal vector since interpolation was linear?
	//vp_Normal = normalize(vp_Normal);
    VP_EmitVertex();
}
