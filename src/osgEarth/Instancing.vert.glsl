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
mat4 oe_instanceModelMatrix;
mat3 oe_instanceNormalMatrix;

vec3 vp_Normal;

void oe_di_setInstancePosition(inout vec4 VertexMODEL)
{ 
    int index = 4 * gl_InstanceID;

    // read the raw data from the buffer
    vec4 m0 = texelFetch(oe_di_postex_TBO, index);
    vec4 m1 = texelFetch(oe_di_postex_TBO, index+1); 
    vec4 m2 = texelFetch(oe_di_postex_TBO, index+2); 
    vec4 m3 = texelFetch(oe_di_postex_TBO, index+3);

    // decode the ObjectID from the last column:    
    oe_index_objectid = uint(m3[0]) + (uint(m3[1]) << 8u) + (uint(m3[2]) << 16u) + (uint(m3[3]) << 24u);
    
    // rebuild positioning matrix and transform the vert.
    // (note: mat4 ctor is column-first order)
    oe_instanceModelMatrix = mat4(m0.x, m1.x, m2.x, 0, m0.y, m1.y, m2.y, 0, m0.z, m1.z, m2.z, 0, m0.w, m1.w, m2.w, 1);

    // transform the model vert so it's ready for gl_ModelViewMatrix
    VertexMODEL = oe_instanceModelMatrix * VertexMODEL;

    // matrix that transforms vectors based on the instance xform.
    // Note, if you use this in a later stage make sure you also
    // apply the actual normal matrix, like: 
    // vec a = gl_NormalMatrix * oe_instanceNormalMatrix * rawVector;
    oe_instanceNormalMatrix = mat3(oe_instanceModelMatrix);

    // Transform the normal
    vp_Normal = oe_instanceNormalMatrix * vp_Normal;
}

