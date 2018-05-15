#version $GLSL_VERSION_STR
#pragma vp_name GPU Lines Screen Projected Model
#pragma vp_entryPoint oe_GPULinesProj_VS_MODEL
#pragma vp_location vertex_model
#pragma import_defines(OE_LINES_USE_LIMITS)

#ifdef OE_LINES_USE_LIMITS
uniform vec2 oe_GPULines_limits;
flat out int oe_GPULines_draw;
#endif

void oe_GPULinesProj_VS_MODEL(inout vec4 unused)
{
#ifdef OE_LINES_USE_LIMITS
    oe_GPULines_draw = 1;
    int first = int(oe_GPULines_limits[0]);
    int last = int(oe_GPULines_limits[1]);
    if (gl_VertexID < first || (last > 0 && gl_VertexID > last))
    {
        oe_GPULines_draw = 0;
    }
#endif
}



[break]

#version $GLSL_VERSION_STR
#pragma vp_name GPU Lines Screen Projected Clip
#pragma vp_entryPoint oe_GPULinesProj_VS_CLIP
#pragma vp_location vertex_clip
//#pragma import_defines(OE_LINES_STIPPLE_PATTERN)
//#pragma import_defines(OE_LINES_WIDTH)
#pragma import_defines(OE_LINES_USE_LIMITS)
#pragma import_defines(OE_GPU_CLAMPING)
#pragma import_defines(OE_LINES_ANTIALIAS)

uniform vec2 oe_ViewportSize;

uniform float oe_GL_LineWidth;
uniform int oe_GL_LineStipplePattern;

in vec3 oe_GPULines_prev;
in vec3 oe_GPULines_next;

#ifdef OE_LINES_USE_LIMITS
flat out int oe_GPULines_draw;
#endif

//#ifdef OE_LINES_STIPPLE_PATTERN
flat out vec2 oe_GPULines_rv;
//#endif

#ifdef OE_GPU_CLAMPING
// see GPUClamping.vert.glsl
vec3 oe_clamp_viewSpaceClampingVector;
#endif


#ifdef OE_LINES_ANTIALIAS
out float oe_GPULines_lateral;
#else
float oe_GPULines_lateral;
#endif

void oe_GPULinesProj_VS_CLIP(inout vec4 currClip)
{
#ifdef OE_LINES_USE_LIMITS
    if (oe_GPULines_draw == 0)
        return;
#endif

#ifdef OE_GPU_CLAMPING
    vec4 prevView = gl_ModelViewMatrix * vec4(oe_GPULines_prev, 1.0);
    prevView.xyz += oe_clamp_viewSpaceClampingVector;
    vec4 prevClip = gl_ProjectionMatrix * prevView;

    vec4 nextView = gl_ModelViewMatrix * vec4(oe_GPULines_next, 1.0);
    nextView.xyz += oe_clamp_viewSpaceClampingVector;
    vec4 nextClip = gl_ProjectionMatrix * nextView;
#else
    vec4 prevClip = gl_ModelViewProjectionMatrix * vec4(oe_GPULines_prev, 1.0);
    vec4 nextClip = gl_ModelViewProjectionMatrix * vec4(oe_GPULines_next, 1.0);
#endif

    // transform into pixel space
    vec2 currPixel = ((currClip.xy/currClip.w)+1.0) * 0.5*oe_ViewportSize;
    vec2 prevPixel = ((prevClip.xy/prevClip.w)+1.0) * 0.5*oe_ViewportSize;
    vec2 nextPixel = ((nextClip.xy/nextClip.w)+1.0) * 0.5*oe_ViewportSize;

    float thickness = oe_GL_LineWidth;

#ifdef OE_LINES_ANTIALIAS
    thickness += 2.0;
#endif

    float len = thickness;

    // even-indexed verts are negative, odd-indexed are positive
    oe_GPULines_lateral = (gl_VertexID & 0x01) == 0? -1.0 : 1.0;

    vec2 dir = vec2(0.0);

    // We will use this to calculate stippling data:
    vec2 stippleDir;

    // The following vertex comparisons must be done in model 
    // space because the equivalency gets mashed after projection.

    // starting point uses (next - current)
    if (gl_Vertex.xyz == oe_GPULines_prev)
    {
        //dir = normalize(nextUnit - currUnit);
        dir = normalize(nextPixel - currPixel);
        stippleDir = dir;
    }
    
    // ending point uses (current - previous)
    else if (gl_Vertex.xyz == oe_GPULines_next)
    {
        //dir = normalize(currUnit - prevUnit);
        dir = normalize(currPixel - prevPixel);
        stippleDir = dir;
    }

    // middle? join
    else
    {
        vec2 dirA = normalize(currPixel - prevPixel);
        vec2 dirB = normalize(nextPixel - currPixel);

        // Edge case: segment that doubles back on itself:
        if (dot(dirA,dirB) < -0.99)
        {
            dir = dirA;
        }

        // Normal case - create a mitered corner:
        else
        {
            vec2 tangent = normalize(dirA+dirB);
            vec2 perp = vec2(-dirA.y, dirA.x);
            vec2 miter = vec2(-tangent.y, tangent.x);
            dir = tangent;
            len = thickness / dot(miter, perp);

            // limit the length of a mitered corner, to prevent unsightly spikes
            const float limit = 2.0;
            if (len > thickness*limit)
            {
                len = thickness;
                dir = dirB;
            }
        }
        stippleDir = dirB;
    }

    // calculate the extrusion vector in pixels
    // note: seems like it should be len/2, BUT we are in [-1..1] space
    vec2 extrudePixel = vec2(-dir.y, dir.x) * (len - 0.5);

    // and convert to unit space:
    vec2 extrudeUnit = extrudePixel / oe_ViewportSize;

    // and from that make a clip-coord offset vector
    vec2 offset = extrudeUnit*oe_GPULines_lateral*currClip.w;
    currClip.xy += offset;

    if (oe_GL_LineStipplePattern != 0xffff)
    {
        // Line creation is done. Now, calculate a rotation angle
        // for use by out fragment shader to do GPU stippling. 
        // This "rotates" the fragment coordinate onto the X axis so that
        // we can apply stippling along the direction of the line.
        // Note: this depends on the GLSL "provoking vertex" being at the 
        // beginning of the line segment!

        // flip the vector so stippling always proceedes from left to right
        // regardless of the direction of the segment
        stippleDir = normalize(stippleDir.x < 0? -stippleDir : stippleDir);

        // calculate the rotation angle that will project the
        // fragment coord onto the X-axis for stipple pattern sampling.
        float way = sign(cross(vec3(1, 0, 0), vec3(stippleDir, 0)).z);
        float angle = acos(dot(vec2(1, 0), stippleDir)) * way;

        // quantize the rotation angle to mitigate precision problems
        // when connecting segments with slightly different vectors
        const float pi = 3.14159265359;
        const float q = pi/8.0;
        angle = floor(angle/q) * q;

        // send it to the fragment shader.
        oe_GPULines_rv = vec2(cos(angle), sin(angle));
    }
}


[break]

#version $GLSL_VERSION_STR

#pragma vp_name GPU Lines Screen Projected FS
#pragma vp_entryPoint oe_GPULinesProj_Stippler_FS
#pragma vp_location fragment_coloring
//#pragma import_defines(OE_LINES_STIPPLE_PATTERN)
//#pragma import_defines(OE_LINES_STIPPLE_FACTOR)
#pragma import_defines(OE_LINES_USE_LIMITS)
#pragma import_defines(OE_LINES_ANTIALIAS)

uniform int oe_GL_LineStippleFactor;
uniform int oe_GL_LineStipplePattern;

flat in vec2 oe_GPULines_rv;

#ifdef OE_LINES_USE_LIMITS
flat in int oe_GPULines_draw;
#endif

#ifdef OE_LINES_ANTIALIAS
in float oe_GPULines_lateral;
#endif

void oe_GPULinesProj_Stippler_FS(inout vec4 color)
{
#ifdef OE_LINES_USE_LIMITS
    if (oe_GPULines_draw == 0)
        discard;
#endif

    if (oe_GL_LineStipplePattern != 0xffff)
    {
        // coordinate of the fragment, shifted to 0:
        vec2 coord = (gl_FragCoord.xy - 0.5);

        // rotate the frag coord onto the X-axis so we can sample the 
        // stipple pattern:
        vec2 coordProj =
            mat2(oe_GPULines_rv.x, -oe_GPULines_rv.y,
                 oe_GPULines_rv.y,  oe_GPULines_rv.x)
            * coord;

        // sample the stippling pattern (16-bits repeating)
        int ci = int(mod(coordProj.x, 16 * oe_GL_LineStippleFactor)) / oe_GL_LineStippleFactor;
        int pattern16 = 0xffff & (oe_GL_LineStipplePattern & (1 << ci));
        if (pattern16 == 0)
            discard; 

        // uncomment to debug stipple direction vectors
        //color.b = 0;
        //color.r = oe_GPULines_rv.x;
        //color.g = oe_GPULines_rv.y;
    }

#ifdef OE_LINES_ANTIALIAS
    // anti-aliasing
    float L = abs(oe_GPULines_lateral);
    color.a *= L > 0.5 ? (1.0-(2*(L-.5)))*(1.0-(2*(L-.5))) : 1.0;
#endif
}
