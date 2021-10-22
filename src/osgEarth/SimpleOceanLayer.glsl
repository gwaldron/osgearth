#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_ocean_VS
#pragma vp_location vertex_view

#pragma import_defines(OE_OCEAN_TEXTURE)
#pragma import_defines(OE_OCEAN_TEXTURE_LOD)
#pragma import_defines(OE_OCEAN_MASK_MATRIX)

uniform float oe_ocean_seaLevel;

#ifdef OE_OCEAN_TEXTURE
out vec2 oe_ocean_texCoord;
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD); // from SDK
#endif

#ifdef OE_OCEAN_MASK_MATRIX
out vec2 oe_ocean_maskCoord;
uniform mat4 OE_OCEAN_MASK_MATRIX ;
#endif

vec4 oe_layer_tilec;
vec3 oe_UpVectorView;   // stage global

void oe_ocean_VS(inout vec4 vertexView)
{
    // move the surface to the new sea level:
    vertexView.xyz += oe_UpVectorView * oe_ocean_seaLevel;

#ifdef OE_OCEAN_TEXTURE
    oe_ocean_texCoord = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, OE_OCEAN_TEXTURE_LOD);
#endif

    // if masking, calculate the mask coordinates
#ifdef OE_OCEAN_MASK_MATRIX
    oe_ocean_maskCoord = (OE_OCEAN_MASK_MATRIX * oe_layer_tilec).st;
#endif
}


[break]

#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_ocean_FS
#pragma vp_location fragment_coloring

#pragma import_defines(OE_OCEAN_TEXTURE)
#pragma import_defines(OE_OCEAN_MASK)
#pragma import_defines(OE_OCEAN_USE_BATHYMETRY)

float oe_terrain_getElevation();

//in float oe_layer_opacity; // from VisibleLayer

uniform vec4 oe_ocean_color;
uniform float oe_ocean_seaLevel;

#ifdef OE_OCEAN_TEXTURE
in vec2 oe_ocean_texCoord;
uniform sampler2D OE_OCEAN_TEXTURE ;
#endif

#ifdef OE_OCEAN_MASK
in vec2 oe_ocean_maskCoord;
uniform sampler2D OE_OCEAN_MASK ;
#endif

// fragment stage global PBR parameters.
struct OE_PBR {
    float roughness;
    float ao;
    float metal;
    float brightness;
    float contrast;
} oe_pbr;

// remaps a value from [vmin..vmax] to [0..1] clamped
float oe_ocean_remap(float val, float vmin, float vmax, float r0, float r1)
{
    float vr = (clamp(val, vmin, vmax)-vmin)/(vmax-vmin);
    return r0 + vr * (r1-r0);
}

// entry point.
void oe_ocean_FS(inout vec4 color)
{
    float alpha = 1.0;

#ifdef OE_OCEAN_USE_BATHYMETRY
    const float lowF = -100.0;
    const float hiF = -10.0;

    float elevation = oe_terrain_getElevation();
    alpha = oe_ocean_remap(elevation, oe_ocean_seaLevel+lowF, oe_ocean_seaLevel+hiF, 1.0, 0.0);
#endif

#ifdef OE_OCEAN_MASK
    float mask = texture(OE_OCEAN_MASK, oe_ocean_maskCoord).a;
    alpha *= mask;
#endif

    color = vec4(oe_ocean_color.rgb, alpha*oe_ocean_color.a);
    
    oe_pbr.roughness = 0.3;
    oe_pbr.ao = 1.0;

#ifdef OE_OCEAN_TEXTURE
    color *= texture(OE_OCEAN_TEXTURE, oe_ocean_texCoord);
#endif
}
