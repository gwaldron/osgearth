#version $GLSL_VERSION_STR

#pragma vp_name       Shadowing Vertex Shader
#pragma vp_entryPoint oe_shadow_vertex
#pragma vp_location   vertex_view


uniform mat4 oe_shadow_matrix[$OE_SHADOW_NUM_SLICES];

out vec4 oe_shadow_coord[$OE_SHADOW_NUM_SLICES];

void oe_shadow_vertex(inout vec4 VertexVIEW)
{
    for(int i=0; i < $OE_SHADOW_NUM_SLICES; ++i)
    {
        oe_shadow_coord[i] = oe_shadow_matrix[i] * VertexVIEW;
    }
}
