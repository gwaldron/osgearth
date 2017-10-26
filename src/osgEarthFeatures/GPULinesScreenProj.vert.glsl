#version $GLSL_VERSION_STR

#pragma vp_name GPU Lines Screen Projected
#pragma vp_entryPoint oe_GPULinesProj_VS_CLIP
#pragma vp_location vertex_clip

uniform vec2 oe_ViewportSize;

in vec3 oe_GPULines_prev;
in vec3 oe_GPULines_next;
in float oe_GPULines_width;

void oe_GPULinesProj_VS_CLIP(inout vec4 currClip)
{
    vec4 prevClip = gl_ModelViewProjectionMatrix * vec4(oe_GPULines_prev, 1.0);
    vec4 nextClip = gl_ModelViewProjectionMatrix * vec4(oe_GPULines_next, 1.0);

    vec2 currUnit = 0.5*(1.0+currClip.xy/currClip.w);
    vec2 prevUnit = 0.5*(1.0+prevClip.xy/prevClip.w);
    vec2 nextUnit = 0.5*(1.0+nextClip.xy/nextClip.w);

    float thickness = abs(oe_GPULines_width);
    float len = thickness;
    float orientation = sign(oe_GPULines_width);

    // starting point uses (next - current)
    vec2 dir = vec2(0.0);

    if (currUnit == prevUnit)
    {
        dir = normalize(nextUnit - currUnit);
    }
    
    // ending point uses (current - previous)
    else if (currUnit == nextUnit)
    {
        dir = normalize(currUnit - prevUnit);
    }

    // middle? join
    else
    {        
        vec2 dirA = normalize(currUnit - prevUnit);
        vec2 dirB = normalize(nextUnit - currUnit);
        vec2 tangent = normalize(dirA+dirB);
        vec2 perp = vec2(-dirA.y, dirA.x);
        vec2 miter = vec2(-tangent.y, tangent.x);
        dir = tangent;
        len = thickness / dot(miter, perp);
        len = clamp(len, -thickness*2.0, thickness*2.0);

        // simpler version, not as nice:
        //dir = normalize(currUnit - prevUnit);
    }

    // calculate the normal vector in pixels
    vec2 normalPixels = vec2(-dir.y, dir.x) * len/2.0;

    // and convert to unit space:
    vec2 normalUnit = normalPixels / oe_ViewportSize;

    // and from that make a clip-coord offset vector
    vec4 offset = vec4(normalUnit*orientation*currClip.w, 0.0, 0.0);
    currClip += offset;
}
