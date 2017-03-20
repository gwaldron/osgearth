#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint ocean_FS
#pragma vp_location fragment_coloring
#pragma import_defines(OE_OCEAN_MASK, OE_OCEAN_BATHYMETRY)

float oe_terrain_getElevation();

in float ocean_visibility; // [0..1] => [invisible..visible]

uniform vec4 ocean_color;

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

#ifdef OE_OCEAN_BATHYMETRY
    const float lowF = -100.0;
    const float hiF = -10.0;
    const float seaLevel = 0.0;

    float elevation = oe_terrain_getElevation();
    float alpha = ocean_remap(elevation, seaLevel+lowF, seaLevel+hiF, 1.0, 0.0);
#endif

#ifdef OE_OCEAN_MASK
    float mask = texture(OE_OCEAN_MASK, ocean_maskCoord).a;
    alpha *= mask;
#endif

    color = vec4(ocean_color.rgb, alpha*ocean_visibility*ocean_color.a);
}
