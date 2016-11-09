#version $GLSL_VERSION_STR

#pragma vp_name       Shadowing Fragment Shader
#pragma vp_entryPoint oe_shadow_fragment
#pragma vp_location   fragment_lighting
#pragma vp_order      0.9

uniform sampler2DArray oe_shadow_map;
uniform float          oe_shadow_color;
uniform float          oe_shadow_blur;

in vec3 vp_Normal; // stage global
in vec4 oe_shadow_coord[$OE_SHADOW_NUM_SLICES];


#define OE_SHADOW_NUM_SAMPLES 16

const vec2 oe_shadow_samples[16] = vec2[](
    vec2( -0.942016, -0.399062 ), vec2( 0.945586, -0.768907 ), vec2( -0.094184, -0.929389 ), vec2( 0.344959, 0.293878 ),
    vec2( -0.915886, 0.457714 ), vec2( -0.815442, -0.879125 ), vec2( -0.382775, 0.276768 ), vec2( 0.974844, 0.756484 ),
    vec2( 0.443233, -0.975116 ), vec2( 0.53743, -0.473734 ), vec2( -0.264969, -0.41893 ), vec2( 0.791975, 0.190909 ),
    vec2( -0.241888, 0.997065 ), vec2( -0.8141, 0.914376 ), vec2( 0.199841, 0.786414 ), vec2( 0.143832, -0.141008 )
);

float oe_shadow_rand(vec2 co)
{
   return fract(sin(dot(co.xy, vec2(12.9898,78.233))) * 43758.5453);
}

vec2 oe_shadow_rot(vec2 p, float a)
{
    vec2 sincos = vec2(sin(a), cos(a));
    return vec2(dot(p, vec2(sincos.y, -sincos.x)), dot(p, sincos.xy));
}

// slow PCF sampling.
float oe_shadow_multisample(in vec3 c, in float refvalue, in float blur)
{
    float shadowed = 0.0;
    float randomAngle = 6.283185 * oe_shadow_rand(c.xy);
    for(int i=0; i<OE_SHADOW_NUM_SAMPLES; ++i)
    {
        vec2 off = oe_shadow_rot(oe_shadow_samples[i], randomAngle);
        vec3 pc = vec3(c.xy + off*blur, c.z);
        float depth = texture(oe_shadow_map, pc).r;
        
        if (depth < 1.0 && depth < refvalue )
        {
           shadowed += 1.0;
        }
    }
    return 1.0-(shadowed/OE_SHADOW_NUM_SAMPLES);
}

void oe_shadow_fragment(inout vec4 color)
{
    float alpha = color.a;
    float factor = 1.0;

    // pre-pixel biasing to reduce moire/acne
    const float b0 = 0.001;
    const float b1 = 0.01;
    vec3 L = normalize(gl_LightSource[0].position.xyz);
    vec3 N = normalize(vp_Normal);
    float costheta = clamp(dot(L,N), 0.0, 1.0);
    float bias = b0*tan(acos(costheta));

    float depth;

    // loop over the slices:
    for(int i=0; i<$OE_SHADOW_NUM_SLICES && factor > 0.0; ++i)
    {
        vec4 c = oe_shadow_coord[i];
        vec3 coord = vec3(c.x, c.y, float(i));

        // TODO: This causes an NVIDIA error (DUI_foreachId) - disable for now.
        if ( oe_shadow_blur > 0.0 )
        {
            factor = min(factor, oe_shadow_multisample(coord, c.z-bias, oe_shadow_blur));
        }
        else
        {
            //float depth = texture(oe_shadow_map, coord).r;
            depth = texture(oe_shadow_map, coord).r;
            if ( depth < 1.0 && depth < c.z-bias )
                factor = 0.0;
        }
    }

    vec3 colorInFullShadow = color.rgb * oe_shadow_color;
    color = vec4( mix(colorInFullShadow, color.rgb, factor), alpha );

    //color = vec4(factor, 1.0);
}
