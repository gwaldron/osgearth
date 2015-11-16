#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_mp_vertModel
#pragma vp_location   vertex_model
#pragma vp_order      first

out vec4 oe_layer_texc;
out vec4 oe_layer_tilec;

out vec3 oe_UpVectorView;
out float oe_mp_terrainElev; // internal
in vec4 oe_terrain_attr;     // internal

void oe_mp_vertModel(inout vec4 vertexModel)
{
    oe_layer_texc  = gl_MultiTexCoord$MP_PRIMARY_UNIT;
    oe_layer_tilec = gl_MultiTexCoord$MP_SECONDARY_UNIT;

    oe_UpVectorView = gl_NormalMatrix * oe_terrain_attr.xyz;

    // internal variable to support the oe_terrain_getElevation() SDK method
    // in the fragment shader stage
    oe_mp_terrainElev = oe_terrain_attr[3];
}
