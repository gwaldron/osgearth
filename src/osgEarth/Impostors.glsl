#version 330
#pragma vp_name Impostor Vertex Shader
#pragma vp_entryPoint oe_Impostor_VS
#pragma vp_location vertex_view
#pragma import_defines(IMPOSTOR_DEBUG)

uniform float impostorSize;
out vec2 imposterTC;
out vec2 rawTC;
vec3 vp_Normal;
vec4 oe_vertexModel;
mat4 oe_modelMatrix;    // from Instancing.glsl
mat3 oe_normalMatrix;   // from Instancing.glsl
mat3 oe_modelLocalRotation;
uniform mat4 osg_ViewMatrixInverse;
uniform mat4 osg_ViewMatrix;

#define USE_INSTANCING

// given the octahedral vector, create a local coordinate system
// in which to create the billboard plane. This returns the offset
// vector for one of the billboard corners, whichever one is
// represented by offset.
// vector points from camera to object; "uv" is [-1..1]
vec3 projectOffset(in vec3 octahedralVector, in vec2 uv)
{
    // Derive the reference frame of the original viewer that was
    // used to capture the impostor: FRU=forward/right/up vectors.
    vec3 F = normalize(-octahedralVector);
    vec3 R = normalize(cross(F, vec3(0,0,1)));
    vec3 U = normalize(cross(R, F));
    vec3 newX = R * uv.x * impostorSize;
    vec3 newZ = U * uv.y * impostorSize;
    return newX + newZ;
}

// transform from 2D frame-grid coordinates to a 3D octahedral vector
// Assume normalized input on +Z hemisphere. Output is [0..1]
vec2 vec3_to_hemioct(in vec3 v)
{
    // Project the hemisphere onto the hemi-octahedron, and then into the xy plane
    vec2 p = v.xy * (1.0 / (abs(v.x) + abs(v.y) + v.z));
    // Rotate and scale the center diamond to the unit square
    return (vec2(p.x + p.y, p.x - p.y)+1.0)*0.5; // [0..1]
    //"    return vec2(p.x + p.y, p.x - p.y); // [-1..1]
}

// transform from 3D octahedral vector to a 3D frame-grid coordinate
// Input is [0..1]
vec3 hemioct_to_vec3(in vec2 t)
{
    vec2 e = t*2.0-1.0;
    // Rotate and scale the unit square back to the center diamond
    vec2 temp = vec2(e.x + e.y, e.x - e.y) * 0.5;
    vec3 v = vec3(temp, 1.0 - abs(temp.x) - abs(temp.y));
    return normalize(v);
}

void oe_Impostor_VS(inout vec4 vertexView)
{
    // number of frames in each dimension - make this a define or something
    const float numFrames = 12.0;

    // calcuate the billboard corner offset vector for the current vertex:
    vec2 offset;
    int seq = gl_VertexID & 0x03; // mod(4)
    if      (seq == 0) offset = vec2(-1,-1);
    else if (seq == 1) offset = vec2( 1,-1);
    else if (seq == 2) offset = vec2(-1, 1);
    else               offset = vec2( 1, 1);

    // get grid vector is the vector from the model to the camera
    // that will get mapped to 2D octahedral space. I tried to do 
    // this in view space but it didn't work - need to revisit that.
    // Don't like the inverse. Can we ditch it?
#ifdef USE_INSTANCING
    mat4 instanceToView = gl_ModelViewMatrix * oe_modelMatrix;
    mat4 viewToInstance = inverse(instanceToView);
#else
    mat4 viewToInstance = inverse(gl_ModelViewMatrix);
#endif

    vec3 pivotOffset = vec3(0, 0, impostorSize);
    //mat4 viewToInstance = inverse(gl_ModelViewMatrix * oe_modelMatrix);
    vec4 cameraInInstanceFrame = viewToInstance * vec4(0,0,0,1);
    vec3 octahedralVector = normalize(cameraInInstanceFrame.xyz - pivotOffset);
    
    //octahedralVector = oe_modelLocalRotation * octahedralVector;

    // transform to 2D frame-grid space
    vec2 octo = vec3_to_hemioct(octahedralVector);
    // quantize to the nearest frame
    octo = floor(octo*numFrames)/numFrames;
    // calculate 1/2 the size of one frame:
    float t = (0.5/numFrames);

    // establish the texture coordinates:
    imposterTC = vec2(t)+octo+(offset*t);

    // use the octahedral vector to establish a reference frame for
    // creating the billboard in model space, then xform that into
    // view space and create the billboard by offsetting the vertices:
    // (oe_normalMatrix comes from the Instancing shader)
    vec3 quantizedVector = hemioct_to_vec3(octo);

#if 1
    vec3 vertexOffset = projectOffset(quantizedVector, offset) + pivotOffset;
#else
    vec3 projectedOffset = projectOffset(quantizedVector, offset); // offset projected into object space
    vec3 vertexOffset = projectedOffset + pivotOffset;
    vertexOffset = normalize(cameraInInstanceFrame.xyz-vertexOffset);
    vertexOffset += projectedOffset;
    //vertexOffset -= gl_Vertex.xyz; // impostor vertex is always 0,0,0 anyway
    vertexOffset += pivotOffset;
#endif

#ifdef USE_INSTANCING
    vertexOffset = mat3(instanceToView) * vertexOffset;
#else
    vertexOffset = gl_NormalMatrix*vertexOffset;
#endif

    vertexView.xyz += vertexOffset;

    rawTC = (offset+1.0)*0.5; // debugging
}



//................................................................
[break]

#version 330
#pragma vp_entryPoint oe_Impostor_FS
#pragma vp_location fragment_coloring
#pragma import_defines(IMPOSTOR_DEBUG)

uniform sampler2D imposterTex;
in vec2 imposterTC;
in vec2 rawTC;

void oe_Impostor_FS(inout vec4 color)
{
    vec4 texel = texture(imposterTex, imposterTC);
    color = texel;

#ifdef IMPOSTOR_DEBUG
    // debugging
    if (any(greaterThan(abs(rawTC*2.0-1.0), vec2(.99)))) color=vec4(1);
    if (rawTC.t > .99) color=vec4(1,0,1,1);
#endif

    if (color.a < 0.15)
        discard;
}
