#extension GL_EXT_gpu_shader4 : enable
#extension GL_ARB_draw_instanced: enable

#pragma vp_entryPoint oe_draw_instanced_attribute_VS_MODEL
#pragma vp_location   vertex_model
#pragma vp_order      0.0

in vec3 oe_DrawInstancedAttribute_position;
in vec4 oe_DrawInstancedAttribute_rotation;
in vec3 oe_DrawInstancedAttribute_scale;

vec3 vp_Normal;

vec3 rotateQuatPt(vec4 q, vec3 v)
{
    vec3 u = q.xyz;
    float s = q.w;
    return 2 * dot(u, v) * u + (s *s - dot(u, u)) * v + 2 * s * cross(u, v);
}

void oe_draw_instanced_attribute_VS_MODEL(inout vec4 currVertex)
{
    vec3 result = oe_DrawInstancedAttribute_scale * currVertex.xyz;
    result = rotateQuatPt(oe_DrawInstancedAttribute_rotation, result);
    result = result + oe_DrawInstancedAttribute_position;
    currVertex.xyz = result;
    // Transform normal by transpose of inverse transformation
    vp_Normal = vp_Normal / oe_DrawInstancedAttribute_scale;
    vp_Normal = rotateQuatPt(oe_DrawInstancedAttribute_rotation, vp_Normal);
    vp_Normal = normalize(vp_Normal);
}
