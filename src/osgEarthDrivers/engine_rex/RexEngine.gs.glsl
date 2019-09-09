#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT


#if 0 // currently unused - triangle discard implemented on CPU instead

#pragma vp_name       REX Engine - GS
#pragma vp_entryPoint oe_rexEngine_gs
#pragma vp_location   geometry

// Vertex Markers:
#define VERTEX_MARKER_DISCARD  1
#define VERTEX_MARKER_GRID     2
#define VERTEX_MARKER_PATCH    4
#define VERTEX_MARKER_BOUNDARY 8
#define VERTEX_MARKER_SKIRT    16

layout(triangles)      in;
layout(triangle_strip) out;
layout(max_vertices=3) out;

void VP_LoadVertex(in int);
void VP_EmitModelVertex();

in vec4 oe_layer_tilec;

void oe_rexEngine_gs(void)
{
    for(int i=0; i < 3; ++i )
    {
        VP_LoadVertex(i);
        if ( int(oe_layer_tilec.z) == VERTEX_MARKER_DISCARD )
            return;
    }

    for(int i=0; i < 3; ++i )
    {
        VP_LoadVertex(i);
        gl_Position = gl_in[i].gl_Position;
        VP_EmitModelVertex();
    }
    EndPrimitive();
}

#endif
