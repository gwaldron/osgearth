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

// matrix that transforms from texture space back to camera view space
uniform mat4 oe_wind_texToViewMatrix;

//const float one_over_log_10 = 1.0/log(10.0);
//float log10(in float x) {
//    return log(x) * one_over_log_10;
//}

void main()
{
    // coords is texture space:
    vec4 pixelNDC = vec4(
        float(gl_WorkGroupID.x) / float(gl_NumWorkGroups.x-1),
        float(gl_WorkGroupID.y) / float(gl_NumWorkGroups.y-1),
        float(gl_WorkGroupID.z) / float(gl_NumWorkGroups.z-1),
        1);

    vec3 totalDirection = vec3(0);

    int i;
    for(i=0; wind[i].speed >= 0.0 && i<SAFETY_BAILOUT; ++i)
    {
        if (wind[i].position.w == 1)
        {
            // point wind:
            vec4 windView = wind[i].position;
            vec4 pixelView = oe_wind_texToViewMatrix * pixelNDC;
            pixelView.xyz /= pixelView.w;
            vec3 dir = pixelView.xyz - windView.xyz;

            // speed attenuation
            float speed = wind[i].speed;
            speed = max(speed - length(dir), 0.0); // linearly interpolates for low-res textures
            //speed = max(speed - 10.0*log10(dot(dir,dir)), 0.0); // good, but doesn't linearly interpolate!
            totalDirection += normalize(dir) * speed;
        }
        else
        {
            // directional wind:
            totalDirection += wind[i].direction * wind[i].speed;
        }
    }

    vec4 pixel;

    // RGB holds normalized wind direction
    pixel.rgb = 0.5*(normalize(totalDirection)+1.0);

    // A holds normalized wind speed
    //pixel.a = length(totalDirection) / MAX_WIND_SPEED;
    pixel.a = min(length(totalDirection), MAX_WIND_SPEED) / MAX_WIND_SPEED;

    if (i==SAFETY_BAILOUT)
        pixel = vec4(1,0,0,1);

    // basic testing
    //vec2 uv = vec2(float(pixelCoords.x)/63.0, float(pixelCoords.y)/63.0);
    //pixel.rg = uv;

    imageStore(oe_wind_tex, ivec3(gl_WorkGroupID), pixel);
}
