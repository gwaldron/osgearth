#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       Phong Lighting Vertex Stage
#pragma vp_entryPoint oe_phong_vertex
#pragma vp_location   vertex_view

#pragma import_defines(OE_LIGHTING)


out vec3 oe_phong_vertexView3;

void oe_phong_vertex(inout vec4 VertexVIEW)
{
#ifndef OE_LIGHTING
    return;
#endif

    oe_phong_vertexView3 = VertexVIEW.xyz / VertexVIEW.w;
}
