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
