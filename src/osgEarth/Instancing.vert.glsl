#version 130
#extension GL_EXT_gpu_shader4 : enable
#extension GL_ARB_draw_instanced: enable

#pragma vp_entryPoint "oe_di_setInstancePosition"
#pragma vp_location   "vertex_model"
#pragma vp_order      "0.0"

uniform sampler2D oe_di_postex;
uniform vec2      oe_di_postex_size;

// Stage-global containing object ID
uint oe_index_objectid;

void oe_di_setInstancePosition(inout vec4 VertexMODEL)
{ 
    float index = float(4 * gl_InstanceID) / oe_di_postex_size.x;
    float s = fract(index);
    float t = floor(index)/oe_di_postex_size.y; 
    float step = 1.0 / oe_di_postex_size.x;  // step from one vec4 to the next 

    vec4 m0 = texture2D(oe_di_postex, vec2(           s, t));
    vec4 m1 = texture2D(oe_di_postex, vec2(    step + s, t)); 
    vec4 m2 = texture2D(oe_di_postex, vec2(2.0*step + s, t)); 
    vec4 m3 = texture2D(oe_di_postex, vec2(3.0*step + s, t));

    // decode the ObjectID from the last column:
    
    oe_index_objectid = uint(m3[0]) + (uint(m3[1]) << 8u) + (uint(m3[2]) << 16u) + (uint(m3[3]) << 24u);
    
    // rebuild positioning matrix and transform the vert. (Note, the matrix is actually
    // transposed so we have to reverse the multiplication order.)
    VertexMODEL = VertexMODEL * mat4(m0, m1, m2, vec4(0,0,0,1));
}
