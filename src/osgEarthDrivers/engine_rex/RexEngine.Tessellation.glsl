#version 400
#pragma vp_name REX Engine TCS
#pragma vp_entryPoint oe_rex_tessellate
#pragma vp_location tess_control
#pragma vp_order first

layout(vertices=3) out;

// feeds the TCS outputer (below)
in vec4 oe_terrain_tessLevel;

// contains [viewport.x, viewport.y, LOD scale]
uniform vec3 oe_Camera;

uniform float oe_terrain_sse; // screen space error (pixels)

// inspired by:
// http://codeflow.org/entries/2010/nov/07/opengl-4-tessellation/

// convert model position to viewport coordinates.
vec2 toPixels(in vec4 model)
{    
    vec4 clip = gl_ModelViewProjectionMatrix * model;
    return (clip.xy/clip.w) * (oe_Camera.xy*0.5);
}

// calculate a tesselation level based on the maximum edge length uniform.
float tessLevel(in vec2 v0, in vec2 v1)
{
    return clamp(distance(v0,v1)/oe_terrain_sse, 1.0, 64.0);
}

void oe_rex_tessellate()
{
    if (gl_InvocationID == 0)
    {
        vec2 ss0 = toPixels(gl_in[0].gl_Position);
        vec2 ss1 = toPixels(gl_in[1].gl_Position);
        vec2 ss2 = toPixels(gl_in[2].gl_Position);

        float e0 = tessLevel(ss1, ss2);
        float e1 = tessLevel(ss2, ss0);
        float e2 = tessLevel(ss0, ss1);

        oe_terrain_tessLevel = vec4(e0, e1, e2, (e0+e1+e2)/3.0);
    }
}


[break]

#version 400
#pragma vp_name       REX Engine TCS
#pragma vp_entryPoint oe_rex_TCS
#pragma vp_location   tess_control
#pragma vp_order      last

layout(vertices=3) out;

in vec4 oe_terrain_tessLevel;

// MAIN ENTRY POINT
void oe_rex_TCS()
{
    if (gl_InvocationID == 0)
    {
        gl_TessLevelOuter[0] = oe_terrain_tessLevel[0];
        gl_TessLevelOuter[1] = oe_terrain_tessLevel[1];
        gl_TessLevelOuter[2] = oe_terrain_tessLevel[2];
        gl_TessLevelInner[0] = oe_terrain_tessLevel[3];
    }
}


[break]
#version 400

#pragma vp_name       REX Engine TES
#pragma vp_entryPoint oe_rex_TES
#pragma vp_location   tess_eval

// osgEarth terrain is always CCW winding
layout(triangles, equal_spacing, ccw) in;
//layout(triangles, fractional_even_spacing, ccw) in;

// Internal helpers:
void VP_Interpolate3();
void VP_EmitVertex();

float VP_Interpolate3(float a, float b, float c) 
{
    return dot(gl_TessCoord.xyz, vec3(a,b,c));
}

vec2 VP_Interpolate3(vec2 a, vec2 b, vec2 c) 
{
    return vec2(
        dot(gl_TessCoord.xyz, vec3(a.x,b.x,c.x)),
        dot(gl_TessCoord.xyz, vec3(a.y,b.y,c.y)));
}

vec3 VP_Interpolate3(vec3 a, vec3 b, vec3 c) 
{
    return vec3(
        dot(gl_TessCoord.xyz, vec3(a.x,b.x,c.x)),
        dot(gl_TessCoord.xyz, vec3(a.y,b.y,c.y)),
        dot(gl_TessCoord.xyz, vec3(a.z,b.z,c.z)));
}

vec4 VP_Interpolate3(vec4 a, vec4 b, vec4 c) 
{
    return vec4(
        dot(gl_TessCoord.xyz, vec3(a.x,b.x,c.x)),
        dot(gl_TessCoord.xyz, vec3(a.y,b.y,c.y)),
        dot(gl_TessCoord.xyz, vec3(a.z,b.z,c.z)),
        dot(gl_TessCoord.xyz, vec3(a.w,b.w,c.w)));
}

vec3 oe_UpVectorView;
vec3 vp_Normal;

void oe_rex_TES()
{
    VP_Interpolate3();
    
    // Must re-normalize the normal vector since interpolation was linear?
    //vp_Normal = normalize(vp_Normal);
    //oe_UpVectorView = normalize(oe_UpVectorView);

    VP_EmitVertex();
}

