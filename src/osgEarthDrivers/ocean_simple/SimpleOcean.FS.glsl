#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       SimpleOcean Proxy FS
#pragma vp_entryPoint oe_ocean_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      0.6

#pragma import_defines(OE_SIMPLE_OCEAN_USE_TEXTURE)
#pragma import_defines(OE_SIMPLE_OCEAN_USE_MASK)

// clamps a value to the vmin/vmax range, then re-maps it to the r0/r1 range:
float ocean_remap( float val, float vmin, float vmax, float r0, float r1 )  
{  
    float vr = (clamp(val, vmin, vmax)-vmin)/(vmax-vmin);  
    return r0 + vr * (r1-r0);  
}  

#ifdef OE_SIMPLE_OCEAN_USE_TEXTURE
uniform sampler2D ocean_surface_tex;          // surface texture
in vec4 ocean_surface_tex_coord;         // surface texture coords
#endif

uniform float ocean_seaLevel;                 // sea level offset
uniform float ocean_lowFeather;               // offset from sea level at which to start feathering out
uniform float ocean_highFeather;              // offset from sea level at which to stop feathering out
uniform vec4  ocean_baseColor;                // base ocean color before processing
uniform float ocean_max_range;                // maximum visible distance of the ocean
uniform float ocean_fade_range;               // distance over which to fade in the ocean
uniform float oe_ocean_alpha;                 // The ocean alpha

// inputs from vertex stage
in float ocean_v_msl;                    // elevation (MSL) of camera
in float ocean_v_range;                  // distance from camera to current vertex


#ifdef OE_SIMPLE_OCEAN_USE_MASK

uniform sampler2D ocean_data;
in vec4 ocean_mask_tex_coord;

// Ocean mask version:
void oe_ocean_fragment(inout vec4 color)  
{  
    color = ocean_baseColor;  

    // amplify the range's effect on alpha when the camera elevation gets low
    float rangeFactor = ocean_remap( ocean_v_msl, -10000.0, 10000.0, 10.0, 1.0 );  

    // affect alpha based on the distance from the camera
    float rangeEffect = ocean_remap( 
       ocean_v_range, 
       ocean_max_range - ocean_fade_range, ocean_max_range * rangeFactor, 
       1.0, 0.0);  
    
#ifdef OE_SIMPLE_OCEAN_USE_TEXTURE
    // start with the surface texture.
    float a1 = texture(ocean_surface_tex, ocean_surface_tex_coord.xy).r;  
    float a2 = -texture(ocean_surface_tex, ocean_surface_tex_coord.zw).r;  
    const float contrast = 1.0;
    float brightness = 1.0 + 0.5*(a1+a2);
    color = clamp(((color-0.5)*contrast + 0.5) * brightness, 0.0, 1.0);
    color.a = ocean_baseColor.a; 
    color.a = max(color.a, ocean_baseColor.a);
#endif // OE_SIMPLE_OCEAN_USE_TEXTURE

    // effect of the terrain mask [0..1] in the alpha component.
    float maskEffect = texture(ocean_data, ocean_mask_tex_coord.xy).a;  

    // color it
    color = vec4( color.rgb, maskEffect * rangeEffect * color.a * oe_ocean_alpha );  

    //    color = vec4( maskEffect, 0, 0, 1 );   // debugging
}

#else // no mask

in float ocean_terrainHeight; // terrain height at vertex in proxy Map

// Proxy layer version:
void oe_ocean_fragment(inout vec4 color)  
{  
    color = ocean_baseColor;  

    // amplify the range's effect on alpha when the camera elevation gets low
    float rangeFactor = ocean_remap( ocean_v_msl, -10000.0, 10000.0, 10.0, 1.0 );  

    // affect alpha based on the distance from the camera
    float rangeEffect = ocean_remap( 
        ocean_v_range, 
        ocean_max_range - ocean_fade_range, ocean_max_range * rangeFactor, 
        1.0, 0.0);  

#ifdef OE_SIMPLE_OCEAN_USE_TEXTURE
    // start with the surface texture.
    float a1 = texture(ocean_surface_tex, ocean_surface_tex_coord.xy).r;  
    float a2 = -texture(ocean_surface_tex, ocean_surface_tex_coord.zw).r;  
    const float contrast = 1.0;
    float brightness = 1.0 + 0.5*(a1+a2);
    color = clamp(((color-0.5)*contrast + 0.5) * brightness, 0.0, 1.0);
    color.a = ocean_baseColor.a; 
    color.a = max(color.a, ocean_baseColor.a);
#endif // OE_SIMPLE_OCEAN_USE_TEXTURE

    // heightfield's effect on alpha [0..1]
    float terrainEffect = ocean_remap( ocean_terrainHeight, ocean_seaLevel+ocean_lowFeather, ocean_seaLevel+ocean_highFeather, 1.0, 0.0 );   

    // color it
    color = vec4( color.rgb, terrainEffect * rangeEffect * oe_ocean_alpha * color.a );  

   //      color = vec4( terrainEffect, 0, 0, 1 );   // debugging
}

#endif
