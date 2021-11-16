#version $GLSL_VERSION_STR
#pragma vp_name GPU Lines Screen Projected Model
#pragma vp_entryPoint oe_LineDrawable_VS_VIEW
#pragma vp_location vertex_view
#pragma vp_order last

uniform vec2 oe_LineDrawable_limits;
flat out int oe_LineDrawable_draw;

// Input attributes for adjacent points
in vec3 oe_LineDrawable_prev;
in vec3 oe_LineDrawable_next;

// Shared stage globals
vec4 oe_LineDrawable_prevView;
vec4 oe_LineDrawable_nextView;

void oe_LineDrawable_VS_VIEW(inout vec4 currView)
{
    oe_LineDrawable_draw = 1;
    int first = int(oe_LineDrawable_limits[0]);
    int last = int(oe_LineDrawable_limits[1]);
    if (first >= 0)
    {
        if (gl_VertexID < first || (last > 0 && gl_VertexID > last))
        {
            oe_LineDrawable_draw = 0;
        }
    }

    // Compute the change in the view vertex so that we can apply the same
    // delta to the prev and next vectors. (An example would be if the verts
    // were GPU-clamped or otherwise permuted in another shader component.)
    vec4 originalView = gl_ModelViewMatrix * gl_Vertex;
    vec4 deltaView = currView - originalView;

    // calculate prev/next points in post-transform view space:
    oe_LineDrawable_prevView = gl_ModelViewMatrix * vec4(oe_LineDrawable_prev,1) + deltaView;
    oe_LineDrawable_nextView = gl_ModelViewMatrix * vec4(oe_LineDrawable_next,1) + deltaView;
}



[break]

#version $GLSL_VERSION_STR
#pragma vp_name GPU Lines Screen Projected Clip
#pragma vp_entryPoint oe_LineDrawable_VS_CLIP
#pragma vp_location vertex_clip
#pragma import_defines(OE_LINE_SMOOTH)

// Set by the InstallCameraUniform callback
uniform vec3 oe_Camera;

// Set by GLUtils methods
uniform float oe_GL_LineWidth;
uniform int oe_GL_LineStipplePattern;

// Input attributes for adjacent points
in vec3 oe_LineDrawable_prev;
in vec3 oe_LineDrawable_next;

flat out int oe_LineDrawable_draw;
flat out vec2 oe_LineDrawable_rv;

// Shared stage globals
vec4 oe_LineDrawable_prevView;
vec4 oe_LineDrawable_nextView;

#ifdef OE_LINE_SMOOTH
out float oe_LineDrawable_lateral;
#else
float oe_LineDrawable_lateral;
#endif

//#define OE_LINE_QUANTIZE_JOINS

void oe_LineDrawable_VS_CLIP(inout vec4 currClip4)
{
    if (oe_LineDrawable_draw == 0)
        return;

    // Transform the prev and next points in clip space.
    vec2 currClip = currClip4.xy;
    vec2 prevClip = (gl_ProjectionMatrix * oe_LineDrawable_prevView).xy;
    vec2 nextClip = (gl_ProjectionMatrix * oe_LineDrawable_nextView).xy;

#ifdef OE_LINE_SMOOTH
    float thickness = floor(oe_GL_LineWidth + 1.0);
#else
    float thickness = max(0.5, floor(oe_GL_LineWidth));
#endif

    float len = thickness;
    int code = (gl_VertexID+2) & 3; // gl_VertexID % 4
    bool isStart = code <= 1;
    bool isRight = code==0 || code==2;

    oe_LineDrawable_lateral = isRight? -1.0 : 1.0;

    vec2 dir = vec2(0.0);
    vec2 stipple_dir;

    // The following vertex comparisons must be done in model 
    // space because the equivalency gets mashed after projection.

    // starting point uses (next - current)
    if (gl_Vertex.xyz == oe_LineDrawable_prev)
    {
        dir = normalize(nextClip - currClip);
        stipple_dir = dir;
    }
    
    // ending point uses (current - previous)
    else if (gl_Vertex.xyz == oe_LineDrawable_next)
    {
        dir = normalize(currClip - prevClip);
        stipple_dir = dir;
    }

    else
    {
        vec2 dirIn  = normalize(currClip - prevClip);
        vec2 dirOut = normalize(nextClip - currClip);

        if (dot(dirIn,dirOut) < -0.999999)
        {
            dir = isStart? dirOut : dirIn;
        }
        else
        {
            vec2 tangent = normalize(dirIn+dirOut);
            vec2 perp = vec2(-dirIn.y, dirIn.x);
            vec2 miter = vec2(-tangent.y, tangent.x);
            dir = tangent;
            len = thickness / dot(miter, perp);

            // limit the length of a mitered corner, to prevent unsightly spikes
            const float limit = 2.0;
            if (len > thickness*limit)
            {
                len = thickness;
                dir = isStart? dirOut : dirIn;
            }
        }
        stipple_dir = dirOut;
    }

    // calculate the extrusion vector in pixels
    // note: seems like it should be len/2, BUT remember we are in [-w..w] space
    vec2 extrudePixel = vec2(-dir.y, dir.x) * len;

    // and convert to unit space:
    vec2 extrudeUnit = extrudePixel / oe_Camera.xy;
        
    // calculate the offset in clip space and apply it.
    vec2 offset = extrudeUnit * oe_LineDrawable_lateral;
    currClip4.xy += (offset * currClip4.w);

#ifdef OE_LINE_QUANTIZE_JOINS
    if (oe_GL_LineStipplePattern != 0xffff)
    {
        // Calculate the rotation angle that will project the
        // fragment coord onto the X-axis for stipple pattern sampling.
        // Note: this depends on the GLSL "provoking vertex" being at the 
        // beginning of the line segment!
        float way = sign(cross(vec3(1, 0, 0), vec3(-dir, 0)).z);
        float angle = acos(dot(vec2(1, 0), -dir)) * way;

        // Quantize the rotation angle to mitigate precision problems
        // when connecting segments with slightly different vectors
        const float pi = 3.14159265359;
        const float q = pi / 8.0;
        angle = floor(angle / q) * q;
        stipple_dir = vec2(cos(angle), sin(angle));
    }
#endif

    oe_LineDrawable_rv = stipple_dir;
}


[break]

#version $GLSL_VERSION_STR

#pragma vp_name GPU Lines Screen Projected FS
#pragma vp_entryPoint oe_LineDrawable_Stippler_FS
#pragma vp_location fragment_coloring
#pragma import_defines(OE_LINE_SMOOTH)

uniform int oe_GL_LineStippleFactor;
uniform int oe_GL_LineStipplePattern;

flat in vec2 oe_LineDrawable_rv;
flat in int oe_LineDrawable_draw;

#ifdef OE_LINE_SMOOTH
in float oe_LineDrawable_lateral;
#endif

void oe_LineDrawable_Stippler_FS(inout vec4 color)
{
    if (oe_LineDrawable_draw == 0)
        discard;

    if (oe_GL_LineStipplePattern != 0xffff)
    {
        // coordinate of the fragment, shifted to 0:
        vec2 coord = (gl_FragCoord.xy - 0.5);

        // rotate the frag coord onto the X-axis to sample the stipple pattern linearly
        vec2 rv = normalize(oe_LineDrawable_rv);
        vec2 coordProj = mat2(rv.x, -rv.y, rv.y, rv.x) * coord;

        // sample the stippling pattern (16-bits repeating)
        int ci = int(mod(coordProj.x, 16.0 * float(oe_GL_LineStippleFactor))) / oe_GL_LineStippleFactor;
        int pattern16 = 0xffff & (oe_GL_LineStipplePattern & (1 << ci));
        if (pattern16 == 0)
            discard;
    }

#ifdef OE_LINE_SMOOTH
    // anti-aliasing
    float L = abs(oe_LineDrawable_lateral);
    color.a = color.a * smoothstep(0.0, 1.0, 1.0-(L*L));
#endif
    
}
