#version 430

// local work matrix
layout(local_size_x=1, local_size_y=1, local_size_z=1) in;

// output image binding
layout(binding=0, rgba8) uniform image3D oe_wind_tex;

#define SAFETY_BAILOUT 4096 // prevent GPU lockup in case of internal error

// https://tinyurl.com/y6see7zz
#define MAX_WIND_SPEED 50.0 // meters per second

// keep me vec4-aligned
struct WindData {
    vec4 position;
    vec3 direction;
    float speed;
};

layout(binding=0, std430) readonly buffer BufferData {
    WindData wind[];
};

uniform mat4 oe_wind_texToViewMatrix; // from texture space to view space
uniform mat4 osg_ViewMatrixInverse; // from view space to world space (OSG)
uniform float osg_FrameTime; // from OSG
uniform sampler2D oe_noise_tex; // repeating noise texture
uniform float oe_wind_power = 1.0; // wind power factor

// scale factor for sampling the noise texture with spherical coords
// note, 6378 = 1km-wide noise texture at the equator.
#define NOISE_TEX_SCALE 48000.0

void main()
{
    // coords is texture space:
    vec4 pixel_ndc = vec4(
        float(gl_WorkGroupID.x) / float(gl_NumWorkGroups.x-1),
        float(gl_WorkGroupID.y) / float(gl_NumWorkGroups.y-1),
        float(gl_WorkGroupID.z) / float(gl_NumWorkGroups.z-1),
        1);

    vec3 totalDirection = vec3(0);

    int i;
    for(i=0; wind[i].speed >= 0.0; ++i)
    {
        // base wind speed:
        float speed = wind[i].speed * oe_wind_power;
        float time_factor = clamp(speed * 0.0025, 0.01, 0.1);
        float time = osg_FrameTime * time_factor;

        // convert the pixel coordinates to world coordinates.
        vec4 pixel_view = oe_wind_texToViewMatrix * pixel_ndc;
        vec4 pixel_world = osg_ViewMatrixInverse * pixel_view;
        
        // convert from world coordinates to spherical coordinates.
        // avoid doing any computations in the shader on the texcoords before
        // using them for sampling, or you risk precision loss.
        float pixel_phi = atan(pixel_world.y, pixel_world.x); // azimuth (lon)
        float pixel_theta = acos(pixel_world.z); // inclination (lat)
        vec2 noise_uv = vec2(pixel_phi, pixel_theta) * NOISE_TEX_SCALE + time;

        // sample the scrolling noise texture to add some randomness; the
        // sampling factor is based on the wind speed. Sample in the XZ plane
        // which is parallel to the ground in most cases.
        vec4 noise = texture(oe_noise_tex, noise_uv);
        speed *= mix(0.0, 2.0, noise[3]);

        if (wind[i].position.w == 1) // point wind
        {
            // point wind:
            vec4 windView = wind[i].position;
            vec4 pixelView = oe_wind_texToViewMatrix * pixel_ndc;
            pixelView.xyz /= pixelView.w;
            vec3 dir = pixelView.xyz - windView.xyz;

            // speed attenuation
            speed = max(speed - length(dir), 0.0); // linearly interpolates for low-res textures
            totalDirection += normalize(dir) * speed;
        }
        else // directional wind
        {
            totalDirection += wind[i].direction * speed;
        }
    }

    float totalSpeed = length(totalDirection);

    vec4 pixel;
    // RGB holds normalized wind direction
    pixel.rgb = 0.5*(normalize(totalDirection)+1.0);

    // A holds normalized wind speed
    pixel.a = min(totalSpeed, MAX_WIND_SPEED) / MAX_WIND_SPEED;

    imageStore(oe_wind_tex, ivec3(gl_WorkGroupID), pixel);
}
