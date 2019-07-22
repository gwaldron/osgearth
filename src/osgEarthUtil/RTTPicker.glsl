#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_pick_encodeObjectID
#pragma vp_location   vertex_clip

//#define USE_BITWISE_MATH
        
// Vertex stage global containing the Object ID; set in ObjectIndex shader.
uint oe_index_objectid;

// output encoded oid to fragment shader
flat out vec4 oe_pick_encoded_objectid;

// whether color already contains oid (written by another RTT camera)
flat out int oe_pick_color_contains_objectid;

void oe_pick_encodeObjectID(inout vec4 vertex)
{
    // the color will contain an encoded object ID for draped features.
    oe_pick_color_contains_objectid = (oe_index_objectid == 1u) ? 1 : 0;

    if ( oe_pick_color_contains_objectid == 0 )
    {
        // encode the objectID as a vec4 (color)
#ifdef USE_BITWISE_MATH
        uint u0 = (oe_index_objectid & 0xff000000u) >> 24u;
        uint u1 = (oe_index_objectid & 0x00ff0000u) >> 16u;
        uint u2 = (oe_index_objectid & 0x0000ff00u) >> 8u;
        uint u3 = (oe_index_objectid & 0x000000ffu);
#else
        uint u0 = (oe_index_objectid / 16777216u);
        uint u1 = (oe_index_objectid / 65536u) - (u0 * 256u);
        uint u2 = (oe_index_objectid / 256u) - (u1 * 256u) - (u0 * 65536u);
        uint u3 = (oe_index_objectid) - (u2 * 256u) - (u1 * 65536u) - (u0 * 16777216u);
#endif
        oe_pick_encoded_objectid = vec4(float(u0), float(u1), float(u2), float(u3)) / 255.0;
    }
}


[break]


#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_pick_renderEncodedObjectID
#pragma vp_location   fragment_output
#pragma vp_order      last

flat in vec4 oe_pick_encoded_objectid;
flat in int oe_pick_color_contains_objectid;
        
out vec4 fragColor;

void oe_pick_renderEncodedObjectID(inout vec4 color)
{
    if ( oe_pick_color_contains_objectid == 1 )
        fragColor = color;
    else
        fragColor = oe_pick_encoded_objectid;
}
