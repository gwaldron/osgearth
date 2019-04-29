#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint ocean_VS
#pragma vp_location vertex_view

#pragma import_defines(OE_OCEAN_TEXTURE)
#pragma import_defines(OE_OCEAN_TEXTURE_LOD)
#pragma import_defines(OE_OCEAN_MASK_MATRIX)

uniform float ocean_maxAltitude;
uniform float ocean_seaLevel;
uniform mat4 osg_ViewMatrixInverse;

vec3 oe_UpVectorView;   // stage global

out float ocean_visibility; // [0..1]

#ifdef OE_OCEAN_TEXTURE
out vec2 ocean_texCoord;
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD); // from SDK
#endif

#ifdef OE_OCEAN_MASK_MATRIX
out vec2 ocean_maskCoord;
uniform mat4 OE_OCEAN_MASK_MATRIX ;
#endif

vec4 oe_layer_tilec;

void ocean_VS(inout vec4 vertexView)
{
    // calculate the visibility based on the max altitude:
    vec3 eye = osg_ViewMatrixInverse[3].xyz;
    float eyeAlt = max(length(eye) - 6371000.0, 0.0);
    float lowAlt = ocean_maxAltitude;
    float highAlt = lowAlt*1.5;
    ocean_visibility = clamp((highAlt-eyeAlt) / (highAlt-lowAlt), 0.0, 1.0);

    // move the surface to the new sea level:
    vertexView.xyz += oe_UpVectorView * ocean_seaLevel;

#ifdef OE_OCEAN_TEXTURE
    ocean_texCoord = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, OE_OCEAN_TEXTURE_LOD);
#endif

    // if masking, calculate the mask coordinates
#ifdef OE_OCEAN_MASK_MATRIX
    ocean_maskCoord = (OE_OCEAN_MASK_MATRIX * oe_layer_tilec).st;
#endif
}


[break]

#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint ocean_FS
#pragma vp_location fragment_coloring

#pragma import_defines(OE_OCEAN_TEXTURE)
#pragma import_defines(OE_OCEAN_MASK)
#pragma import_defines(OE_OCEAN_USE_BATHYMETRY)

float oe_terrain_getElevation();

in float ocean_visibility; // [0..1] => [invisible..visible]
in float oe_layer_opacity; // from VisibleLayer

uniform vec4 ocean_color;
uniform float ocean_seaLevel;

#ifdef OE_OCEAN_TEXTURE
in vec2 ocean_texCoord;
uniform sampler2D OE_OCEAN_TEXTURE ;
#endif

#ifdef OE_OCEAN_MASK
in vec2 ocean_maskCoord;
uniform sampler2D OE_OCEAN_MASK ;
#endif

// remaps a value from [vmin..vmax] to [0..1] clamped
float ocean_remap(float val, float vmin, float vmax, float r0, float r1)
{
    float vr = (clamp(val, vmin, vmax)-vmin)/(vmax-vmin);
    return r0 + vr * (r1-r0);
}

// entry point.
void ocean_FS(inout vec4 color)
{
    float alpha = 1.0;

#ifdef OE_OCEAN_USE_BATHYMETRY
    const float lowF = -100.0;
    const float hiF = -10.0;

    float elevation = oe_terrain_getElevation();
    alpha = ocean_remap(elevation, ocean_seaLevel+lowF, ocean_seaLevel+hiF, 1.0, 0.0);
#endif

#ifdef OE_OCEAN_MASK
    float mask = texture(OE_OCEAN_MASK, ocean_maskCoord).a;
    alpha *= mask;
#endif

    color = vec4(ocean_color.rgb, alpha*ocean_visibility*oe_layer_opacity*ocean_color.a);

#ifdef OE_OCEAN_TEXTURE
    color *= texture(OE_OCEAN_TEXTURE, ocean_texCoord);
#endif
}
