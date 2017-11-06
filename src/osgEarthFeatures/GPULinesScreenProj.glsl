#version $GLSL_VERSION_STR

#pragma vp_name GPU Lines Screen Projected
#pragma vp_entryPoint oe_GPULinesProj_VS_CLIP
#pragma vp_location vertex_clip
#pragma import_defines(OE_GPULINES_STIPPLE_PATTERN)

uniform vec2 oe_ViewportSize;

in vec3 oe_GPULines_prev;
in vec3 oe_GPULines_next;
in float oe_GPULines_width;

#ifdef OE_GPULINES_STIPPLE_PATTERN
flat out vec2 oe_GPULines_rv;
#endif

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
    float ar = oe_ViewportSize.x/oe_ViewportSize.y;

    vec2 dir = vec2(0.0);

    // We will use this to calculate stippling data:
    vec2 fragDir = dir;

    // The following vertex comparisons must be done in model 
    // space because the equivalency gets mashed after projection.

    // starting point uses (next - current)
    if (gl_Vertex.xyz == oe_GPULines_prev)
    {
        dir = normalize(nextUnit - currUnit);
    }
    
    // ending point uses (current - previous)
    else if (gl_Vertex.xyz == oe_GPULines_next)
    {
        dir = normalize(currUnit - prevUnit);
    }

    // middle? join
    else
    {
        vec2 dirA = normalize(currUnit - prevUnit);
        vec2 dirB = normalize(nextUnit - currUnit);
        if (dot(dirA,dirB) < -0.99)
        {
            dir = normalize(currUnit-prevUnit);
        }
        else
        {
            vec2 tangent = normalize(dirA+dirB);
            vec2 perp = vec2(-dirA.y, dirA.x);
            vec2 miter = vec2(-tangent.y, tangent.x);
            dir = tangent;
            len = thickness / dot(miter, perp);
            len = clamp(len, -thickness*4.0, thickness*4.0);
        }
        fragDir = dirB;
    }

    // calculate the extrusion vector in pixels
    vec2 extrudePixels = vec2(-dir.y, dir.x) * len/2.0;

    // and convert to unit space:
    vec2 extrudeUnit = extrudePixels / oe_ViewportSize;

    // and from that make a clip-coord offset vector
    vec4 offset = vec4(extrudeUnit*orientation*currClip.w, 0.0, 0.0);
    currClip += offset;

#ifdef OE_GPULINES_STIPPLE_PATTERN
    // line creation is done. Now, calculate a rotation angle
    // for use by out fragment shader to do GPU stippling. The rotation
    // vector "rotates" the fragment coordinate onto the X axis so
    // linear stippling can be applied along the direction of the line.
    // Note: this depends on the GLSL "provoking vertex" being at the 
    // beginning of the line segment!

    // aspect ratio correction (since stippling happens in viewport coords)
    fragDir.x *= ar;

    // flip the vector so stippling always proceedes from left to right
    // regardless of the direction of the segment
    fragDir = normalize(fragDir.x < 0? -fragDir : fragDir);

    // calculate the rotation angle that will project the
    // fragment coord onto the X-axis for stipple pattern sampling.
    float way = sign(cross(vec3(1, 0, 0), vec3(fragDir, 0)).z);
    float angle = acos(dot(vec2(1, 0), fragDir)) * way;

    // send it to the fragment shader.
    oe_GPULines_rv = vec2(cos(angle), sin(angle));
#endif
}


[break]

#version $GLSL_VERSION_STR

#pragma vp_name GPU Lines Screen Projected FS
#pragma vp_entryPoint oe_GPULinesProj_Stippler_FS
#pragma vp_location fragment_coloring
#pragma import_defines (OE_GPULINES_STIPPLE_PATTERN, OE_GPULINES_STIPPLE_FACTOR)

flat in vec2 oe_GPULines_rv;
void oe_GPULinesProj_Stippler_FS(inout vec4 color)
{
#ifdef OE_GPULINES_STIPPLE_PATTERN

    const int pattern = OE_GPULINES_STIPPLE_PATTERN;
    const int factor = OE_GPULINES_STIPPLE_FACTOR;

    // coordinate of the fragment, shifted to 0:
    vec2 coord = gl_FragCoord.xy - 0.5;
                
    // rotate the frag coord onto the X-axis so we can sample the 
    // stipple pattern:
    vec2 coordProj =
        mat2(oe_GPULines_rv.x, -oe_GPULines_rv.y,
             oe_GPULines_rv.y,  oe_GPULines_rv.x)
        * coord;

    // sample the stippling pattern (16-bits repeating)
    //int ci = int(mod(coordProj.x, 16));
    int ci = int(mod(coordProj.x, 16 * factor)) / factor;
    if ((pattern & (1 << ci)) == 0)
        discard; 
#endif
}
