#version 110

uniform vec4 oe_tile_key;
varying vec4 oe_layer_tilec;

vec3 oe_Normal;

uniform mat4 oe_nmap_normalMatrix;
varying vec2 oe_nmap_normalCoords;
varying mat3 oe_nmap_TBN;

uniform float shmoo;

void oe_nmap_vertex(inout vec4 VertexMODEL)
{
    //TODO: fix this matrix
    oe_nmap_normalCoords = (oe_nmap_normalMatrix * oe_layer_tilec).st;

    //TODO: non-hard-code this
    
    const float samples = 127.0; // i.e., floor(texture_size/2.0)
    oe_nmap_normalCoords *= (samples-1.0)/samples;
    oe_nmap_normalCoords += 0.5/(samples-1.0);

    // form the matrix that will transform a normal vector from
    // tangent space to model space in the fragment shader.
    vec3 B = vec3(0,1,0);
    vec3 N = gl_Normal;
    vec3 T = normalize(cross(B,N));

   // B = normalize(gl_NormalMatrix * B);
   // N = normalize(gl_NormalMatrix * N);
   // T = normalize(gl_NormalMatrix * T);
    oe_nmap_TBN = gl_NormalMatrix * mat3(T, B, N);
}