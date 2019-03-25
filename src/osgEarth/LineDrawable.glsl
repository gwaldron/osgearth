#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT
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

    // clamp the current vertex to the near clip plane (or at least to Z=0)
    // to prevent clip space coordinate freakouts! (only in perspective camera)
    if (currView.z > 0.0 && gl_ProjectionMatrix[3][3] == 0.0)
    {
        if (oe_LineDrawable_prevView != currView)
        {
            vec3 v = currView.xyz-oe_LineDrawable_prevView.xyz;
            float r = -oe_LineDrawable_prevView.z / v.z;
            currView.xyz = oe_LineDrawable_prevView.xyz + v*r;
        }
        else
        {
            vec3 v = oe_LineDrawable_nextView.xyz-currView.xyz;
            float r = currView.z / -v.z;
            currView.xyz += v*r;
        }
    }
}



[break]

#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT
#pragma vp_name GPU Lines Screen Projected Clip
#pragma vp_entryPoint oe_LineDrawable_VS_CLIP
#pragma vp_location vertex_clip
#pragma import_defines(OE_LINE_SMOOTH)

// Set by the InstallViewportUniform callback
uniform vec2 oe_ViewportSize;

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

void oe_LineDrawable_VS_CLIP(inout vec4 currClip)
{
    if (oe_LineDrawable_draw == 0)
        return;

    // Transform the prev and next points in clip space.
    vec4 prevClip = gl_ProjectionMatrix * oe_LineDrawable_prevView;
    vec4 nextClip = gl_ProjectionMatrix * oe_LineDrawable_nextView;

    // Transform all points into pixel space
    vec2 prevPixel = ((prevClip.xy/prevClip.w)+1.0) * 0.5*oe_ViewportSize;
    vec2 currPixel = ((currClip.xy/currClip.w)+1.0) * 0.5*oe_ViewportSize;
    vec2 nextPixel = ((nextClip.xy/nextClip.w)+1.0) * 0.5*oe_ViewportSize;

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

    // We will use this to calculate stippling data:
    vec2 stippleDir;

    // The following vertex comparisons must be done in model 
    // space because the equivalency gets mashed after projection.

    // starting point uses (next - current)
    if (gl_Vertex.xyz == oe_LineDrawable_prev)
    {
        dir = normalize(nextPixel - currPixel);
        stippleDir = dir;
    }
    
    // ending point uses (current - previous)
    else if (gl_Vertex.xyz == oe_LineDrawable_next)
    {
        dir = normalize(currPixel - prevPixel);
        stippleDir = dir;
    }

    else
    {
        vec2 dirIn  = normalize(currPixel - prevPixel);
        vec2 dirOut = normalize(nextPixel - currPixel);

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

        stippleDir = dirOut;
    }

    // calculate the extrusion vector in pixels
    // note: seems like it should be len/2, BUT remember we are in [-w..w] space
    vec2 extrudePixel = vec2(-dir.y, dir.x) * len;

    // and convert to unit space:
    vec2 extrudeUnit = extrudePixel / oe_ViewportSize;
        
    // calculate the offset in clip space and apply it.
    vec2 offset = extrudeUnit * oe_LineDrawable_lateral * currClip.w;
    currClip.xy += offset;

    // prepare for stippling:
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
        stippleDir = normalize(stippleDir.x < 0.0 ? -stippleDir : stippleDir);

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
        oe_LineDrawable_rv = vec2(cos(angle), sin(angle));
    }
}


[break]

#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

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

        // rotate the frag coord onto the X-axis so we can sample the 
        // stipple pattern:
        vec2 coordProj =
            mat2(oe_LineDrawable_rv.x, -oe_LineDrawable_rv.y,
                 oe_LineDrawable_rv.y,  oe_LineDrawable_rv.x)
            * coord;

        // sample the stippling pattern (16-bits repeating)
        int ci = int(mod(coordProj.x, 16.0 * float(oe_GL_LineStippleFactor))) / oe_GL_LineStippleFactor;
        int pattern16 = 0xffff & (oe_GL_LineStipplePattern & (1 << ci));
        if (pattern16 == 0)
            discard; 

        // uncomment to debug stipple direction vectors
        //color.b = 0;
        //color.r = oe_LineDrawable_rv.x;
        //color.g = oe_LineDrawable_rv.y;
    }

#ifdef OE_LINE_SMOOTH
    // anti-aliasing
    float L = abs(oe_LineDrawable_lateral);
    color.a = color.a * smoothstep(0.0, 1.0, 1.0-(L*L));
#endif
}
