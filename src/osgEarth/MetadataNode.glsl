#pragma vp_entryPoint oe_metadata_vert
#pragma vp_location   vertex_view

#version 430

uniform int  oe_index_uniform = 0;
in uint      oe_index_attr;
uint         oe_index_objectid;         // Stage global containing the Object ID.

struct Instance
{
    uint objectID;
    uint visible;
};

layout(binding = 0, std430) buffer Instances {
    Instance instances[];
};

flat out uint out_visible;

void oe_metadata_vert(inout vec4 VertexVIEW)
{
    int index = -1;
    if (oe_index_attr > 0)
    {
        index = int(oe_index_attr-1);
    }
    else if (oe_index_uniform > 0)
    {
        index = int(oe_index_uniform - 1);
    }
    // This is a big ole hack but if the oe_index_objectid is already set (via instancing) then assume it is the index
    else if (oe_index_objectid > 0)
    {
        index = int(oe_index_objectid - 1);
    }

    if (index >= 0)
    {
        uint objectID = instances[index].objectID;
        oe_index_objectid = objectID;
        out_visible = instances[index].visible;
    }
    else
    {
        oe_index_objectid = 0;
        out_visible = 1;
    }
}


[break]
#pragma vp_entryPoint oe_metadata_frag
#pragma vp_location   fragment_coloring
#pragma vp_order      first

#version 430

flat in uint out_visible;

void oe_metadata_frag(inout vec4 color)
{
    if (out_visible == 0) discard;
}
