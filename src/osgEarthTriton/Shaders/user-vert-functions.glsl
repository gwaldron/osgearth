
float user_get_depth( in vec3 worldPos )
{
    return 1000.0;
}

#ifdef DOUBLE_PRECISION

dvec3 user_horizon(in dvec3 intersect)
{
    return intersect;
}

#else

vec3 user_horizon(in vec3 intersect)
{
    return intersect;
}

#endif

/* You may use this hook to set any varying parameters you need for the user-functions.glsl fragment program.
   provided are the ocean vertex in world, eye, and projected coordinates. */
void user_intercept(in vec3 worldPosition, in vec3 localPosition, in vec4 eyePosition, in vec4 projectedPosition)
{

}

#if __VERSION__ > 140
    out float oe_LogDepth_logz;
#else
    varying float oe_LogDepth_logz;
#endif
uniform mat4 trit_projection;

// Provides a point to override the final value of gl_Position.
// Useful for implementing logarithmic depth buffers etc.
vec4 overridePosition(in vec4 position)
{
    if(trit_projection[3][3] == 0)
    {
        mat4 clip2view = inverse(trit_projection);
        vec4 farPoint = clip2view * vec4(0,0,1,1);
        float FAR = -farPoint.z/farPoint.w;
    
        const float C = 0.001;
        float FC = 1.0/log(FAR*C + 1);
        oe_LogDepth_logz = log(position.w*C + 1)*FC;
        position.z = (2*oe_LogDepth_logz - 1)*position.w;
    }
    else
    {
        oe_LogDepth_logz = -1;
    }
    return position;
}
