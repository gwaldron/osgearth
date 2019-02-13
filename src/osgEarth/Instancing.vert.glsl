#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#extension GL_EXT_gpu_shader4 : enable
#extension GL_ARB_draw_instanced: enable

#pragma vp_entryPoint oe_di_setInstancePosition
#pragma vp_location   vertex_model
#pragma vp_order      0.0

uniform samplerBuffer oe_di_postex_TBO;

// Stage-global containing object ID
uint oe_index_objectid;
vec3 vp_Normal;
vec4 oe_vertexModel;
mat4 oe_modelMatrix;
mat3 oe_normalMatrix;
mat4 oe_modelViewMatrix;
//mat3 oe_modelLocalRotation;
uniform mat4 osg_ViewMatrix;
uniform mat4 osg_ViewMatrixInverse;

void oe_di_setInstancePosition(inout vec4 VertexMODEL)
{ 
    int index = 4 * gl_InstanceID;

    vec4 m0 = texelFetch(oe_di_postex_TBO, index);
    vec4 m1 = texelFetch(oe_di_postex_TBO, index+1); 
    vec4 m2 = texelFetch(oe_di_postex_TBO, index+2); 
    vec4 m3 = texelFetch(oe_di_postex_TBO, index+3);

    // decode the ObjectID from the last column:
    
    oe_index_objectid = uint(m3[0]) + (uint(m3[1]) << 8u) + (uint(m3[2]) << 16u) + (uint(m3[3]) << 24u);
    
    // rebuild positioning matrix and transform the vert.
    // (note: mat4 ctor is column-first order)
    mat4 xform = mat4(m0.x, m1.x, m2.x, 0, m0.y, m1.y, m2.y, 0, m0.z, m1.z, m2.z, 0, m0.w, m1.w, m2.w, 1);

    VertexMODEL = xform * VertexMODEL;

    // rotate the normal vector in the same manner.
    oe_normalMatrix = mat3(xform);
    vp_Normal = oe_normalMatrix * vp_Normal;

    oe_modelMatrix = xform;

    oe_normalMatrix = oe_normalMatrix * gl_NormalMatrix;

    //vec4 r0 = texelFetch(oe_di_postex_TBO, index + 4);
    //vec4 r1 = texelFetch(oe_di_postex_TBO, index + 5);
    //vec4 r2 = texelFetch(oe_di_postex_TBO, index + 6);
    //oe_modelLocalRotation = mat3(r0.x, r1.x, r2.x, r0.y, r1.y, r2.y, r0.z, r1.z, r2.z);
}

