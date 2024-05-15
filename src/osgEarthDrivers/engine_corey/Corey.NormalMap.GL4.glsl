#pragma include RexEngine.GL4.glsl
#pragma vp_function oe_rex_normalMapVS, vertex_view, 0.5

#pragma import_defines(OE_TERRAIN_RENDER_NORMAL_MAP)

//out vec3 oe_normal_binormal;

#ifdef OE_TERRAIN_RENDER_NORMAL_MAP
vec2 oe_terrain_getNormalCoords();
flat out uint64_t oe_normal_handle;
out vec2 oe_normal_uv;
#endif

void oe_rex_normalMapVS(inout vec4 unused)
{
#ifdef OE_TERRAIN_RENDER_NORMAL_MAP
    oe_normal_handle = 0;
    int index = oe_tile[oe_tileID].normalIndex;
    if (index >= 0)
    {
        oe_normal_uv = oe_terrain_getNormalCoords();
        oe_normal_handle = oe_terrain_tex[index];
    }
#endif
}


[break]
#pragma include RexEngine.GL4.glsl
#pragma vp_function oe_rex_normalMapFS, fragment_coloring, 0.1

#pragma import_defines(OE_TERRAIN_RENDER_NORMAL_MAP)
#pragma import_defines(OE_DEBUG_NORMALS)
#pragma import_defines(OE_DEBUG_CURVATURE)

in vec3 vp_Normal;
in vec3 oe_UpVectorView;

vec4 oe_terrain_getNormalAndCurvature(in uint64_t, in vec2); // SDK

#ifdef OE_TERRAIN_RENDER_NORMAL_MAP
flat in uint64_t oe_normal_handle;
in vec2 oe_normal_uv;
#endif

// stage global
mat3 oe_normalMapTBN;

void oe_rex_normalMapFS(inout vec4 color)
{
    vp_Normal = oe_UpVectorView;

    vec3 binormal = normalize(gl_NormalMatrix * vec3(0, 1, 0));
    vec3 tangent = normalize(cross(binormal, oe_UpVectorView));
    oe_normalMapTBN = mat3(tangent, binormal, oe_UpVectorView);

#ifdef OE_TERRAIN_RENDER_NORMAL_MAP
    if (oe_normal_handle > 0)
    {
        vec4 N = oe_terrain_getNormalAndCurvature(oe_normal_handle, oe_normal_uv);
        vp_Normal = normalize( oe_normalMapTBN*N.xyz );
    }
#endif

#ifdef OE_DEBUG_CURVATURE
    // visualize curvature quantized:
    color.rgba = vec4(0, 0, 0, 1);
    float curvature = N.w;
    if (curvature > 0.0) color.r = curvature;
    if (curvature < 0.0) color.g = -curvature;
#endif

#ifdef OE_DEBUG_NORMALS
    // visualize normals:
    color.rgb = (N.xyz + 1.0)*0.5;
#endif
}
