#pragma vp_name       REX Engine - GS
#pragma vp_entryPoint oe_rex_GS
#pragma vp_location   geometry

layout(triangles)in;
layout(triangle_strip, max_vertices=3) out;

void VP_LoadVertex(in int);
void VP_EmitModelVertex();

void oe_rex_GS(void)
{
    for(int i=0; i < 3; ++i )
    {
        VP_LoadVertex(i);
        gl_Position = gl_in[i].gl_Position;
        VP_EmitModelVertex();
    }
    EndPrimitive();
}
