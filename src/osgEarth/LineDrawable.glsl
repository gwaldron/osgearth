#version $GLSL_VERSION_STR
#pragma vp_name GPU Lines Screen Projected Model
#pragma vp_entryPoint oe_LineDrawable_VS_VIEW
#pragma vp_location vertex_view
#pragma vp_order last

uniform vec2 oe_LineDrawable_limits;
flat out int oe_LineDrawable_draw;

// change in view vertex over the course of the shader pipeline
vec4 oe_LineDrawable_viewDelta;

void oe_LineDrawable_VS_VIEW(inout vec4 vertexView)
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

    // record the change in the view vertex so that we can apply the same
    // delta to the prev and next vectors.
    vec4 originalView = gl_ModelViewMatrix * gl_Vertex;
    oe_LineDrawable_viewDelta = vertexView - originalView;
}



[break]

#version $GLSL_VERSION_STR
#pragma vp_name GPU Lines Screen Projected Clip
#pragma vp_entryPoint oe_LineDrawable_VS_CLIP
#pragma vp_location vertex_clip
#pragma import_defines(OE_LINES_ANTIALIAS)

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

// Change in main vertex (calculated in oe_LineDrawable_VS_VIEW)
vec4 oe_LineDrawable_viewDelta;

#ifdef OE_LINES_ANTIALIAS
out float oe_LineDrawable_lateral;
#else
float oe_LineDrawable_lateral;
#endif

void oe_LineDrawable_VS_CLIP(inout vec4 currClip)
{
    if (oe_LineDrawable_draw == 0)
        return;

    // compute the prev and next points in clip space.
    // we apply the "view detla" to account for any other shaders that 
    // might have altered the main vertex up to this point.
    vec4 prevView = gl_ModelViewMatrix * vec4(oe_LineDrawable_prev, 1.0);
    prevView += oe_LineDrawable_viewDelta;
    vec4 prevClip = gl_ProjectionMatrix * prevView;

    vec4 nextView = gl_ModelViewMatrix * vec4(oe_LineDrawable_next, 1.0);
    nextView += oe_LineDrawable_viewDelta;
    vec4 nextClip = gl_ProjectionMatrix * nextView;

    // transform into pixel space
    vec2 currPixel = ((currClip.xy/currClip.w)+1.0) * 0.5*oe_ViewportSize;
    vec2 prevPixel = ((prevClip.xy/prevClip.w)+1.0) * 0.5*oe_ViewportSize;
    vec2 nextPixel = ((nextClip.xy/nextClip.w)+1.0) * 0.5*oe_ViewportSize;

#ifdef OE_LINES_ANTIALIAS
    float thickness = floor(oe_GL_LineWidth + 2.0);
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

    // and from that make a clip-coord offset vector
    vec2 offset = extrudeUnit * oe_LineDrawable_lateral * currClip.w;
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
        oe_LineDrawable_rv = vec2(cos(angle), sin(angle));
    }
}


[break]

#version $GLSL_VERSION_STR

#pragma vp_name GPU Lines Screen Projected FS
#pragma vp_entryPoint oe_LineDrawable_Stippler_FS
#pragma vp_location fragment_coloring
#pragma import_defines(OE_LINES_ANTIALIAS)

uniform int oe_GL_LineStippleFactor;
uniform int oe_GL_LineStipplePattern;

flat in vec2 oe_LineDrawable_rv;
flat in int oe_LineDrawable_draw;

#ifdef OE_LINES_ANTIALIAS
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
        int ci = int(mod(coordProj.x, 16 * oe_GL_LineStippleFactor)) / oe_GL_LineStippleFactor;
        int pattern16 = 0xffff & (oe_GL_LineStipplePattern & (1 << ci));
        if (pattern16 == 0)
            discard; 

        // uncomment to debug stipple direction vectors
        //color.b = 0;
        //color.r = oe_LineDrawable_rv.x;
        //color.g = oe_LineDrawable_rv.y;
    }

#ifdef OE_LINES_ANTIALIAS
    // anti-aliasing
    float L = abs(oe_LineDrawable_lateral);
    color.a *= L > 0.5 ? (1.0-(2*(L-.5)))*(1.0-(2*(L-.5))) : 1.0;
#endif
}
