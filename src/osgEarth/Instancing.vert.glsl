#version 120
#extension GL_EXT_gpu_shader4 : enable
#extension GL_ARB_draw_instanced: enable

#pragma vp_entryPoint "oe_di_setInstancePosition"
#pragma vp_location   "vertex_model"

uniform sampler2D oe_di_postex;
uniform vec2 oe_di_postex_size;

void oe_di_setInstancePosition(inout vec4 VertexMODEL)
{ 
    float index = float(4 * gl_InstanceID) / oe_di_postex_size.x;
    float s = fract(index);
    float t = floor(index)/oe_di_postex_size.y; 
    float step = 1.0 / oe_di_postex_size.x;  // step from one vec4 to the next 
    vec4 m0 = texture2D(oe_di_postex, vec2(s, t));
    vec4 m1 = texture2D(oe_di_postex, vec2(s+step, t)); 
    vec4 m2 = texture2D(oe_di_postex, vec2(s+step+step, t)); 
    vec4 m3 = texture2D(oe_di_postex, vec2(s+step+step+step, t));
    VertexMODEL = VertexMODEL * mat4(m0, m1, m2, m3);
}
