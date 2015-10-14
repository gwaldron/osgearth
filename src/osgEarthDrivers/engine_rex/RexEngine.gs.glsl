#version 330 compatibility

#pragma vp_name       REX Engine - GS
#pragma vp_entryPoint oe_rexEngine_gs
#pragma vp_location   geometry

layout(triangles)      in;
layout(triangle_strip) out;
layout(max_vertices=3) out;

void VP_LoadVertex(in int);
void VP_EmitModelVertex();

in vec4 oe_layer_tilec;

void oe_rexEngine_gs(void)
{
    float masked = 1.0;
    for(int i=0; i < 3; ++i )
    {
        VP_LoadVertex(i);
        masked = masked * oe_layer_tilec.z;
    }

    if (masked > 0.0)
    {
        for(int i=0; i < 3; ++i )
        {
            VP_LoadVertex(i);
            gl_Position = gl_in[i].gl_Position;
            VP_EmitModelVertex();
        }
        EndPrimitive();
    }
}
