#pragma vp_name REX Engine TCS
#pragma vp_function oe_rex_TCS, tess_control, last

layout(vertices=3) out;

uniform float oe_terrain_tess;
uniform float oe_terrain_tess_range;

// temporary: use lifemap texture from earth file
//#pragma oe_use_shared_layer(LIFEMAP_TEX, LIFEMAP_MAT);

varying vec4 oe_layer_tilec;
varying vec4 vp_Vertex;
varying vec3 vp_Normal;

void VP_LoadVertex(in int);
float oe_terrain_getElevation();

float remap_unit(in float value, in float lo, in float hi)
{
    return clamp((value - lo) / (hi - lo), 0.0, 1.0);
}

void oe_rex_TCS()
{
    if (gl_InvocationID == 0)
    {
#if 1
        // iterator backward so we end up loading vertex 0
        float d[3];
        vec3 v[3];
        for (int i = 2; i >= 0; --i)
        {
            VP_LoadVertex(i);
            v[i] = (gl_ModelViewMatrix * (vp_Vertex + vec4(vp_Normal * oe_terrain_getElevation(), 0.0))).xyz;
            d[i] = oe_terrain_tess;
        }

        float max_dist = oe_terrain_tess_range;
        float min_dist = oe_terrain_tess_range / 6.0;

        vec3 m12 = 0.5*(v[1] + v[2]);
        vec3 m20 = 0.5*(v[2] + v[0]);
        vec3 m01 = 0.5*(v[0] + v[1]);

        float f12 = remap_unit(-m12.z, max_dist, min_dist);
        float f20 = remap_unit(-m20.z, max_dist, min_dist);
        float f01 = remap_unit(-m01.z, max_dist, min_dist);

        float e0 = max(1.0, max(d[1], d[2]) * f12);
        float e1 = max(1.0, max(d[2], d[0]) * f20);
        float e2 = max(1.0, max(d[0], d[1]) * f01);

        float e3 = max(e0, max(e1, e2));

        gl_TessLevelOuter[0] = e0;
        gl_TessLevelOuter[1] = e1;
        gl_TessLevelOuter[2] = e2;
        gl_TessLevelInner[0] = e3;
#else

        gl_TessLevelOuter[0] = oe_terrain_tess;
        gl_TessLevelOuter[1] = oe_terrain_tess;
        gl_TessLevelOuter[2] = oe_terrain_tess;
        gl_TessLevelInner[0] = oe_terrain_tess;
#endif
    }
}


[break]
#pragma vp_name REX Engine TES
#pragma vp_function oe_rex_TES, tess_eval

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

void oe_rex_TES()
{
    VP_Interpolate3();
    VP_EmitVertex();
}

