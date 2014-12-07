#version 110

uniform vec4 oe_tile_key;
varying vec4 oe_layer_tilec;

uniform vec4 oe_nmap_normalMatrix;
varying vec2 oe_nmap_normalCoords;

void oe_nmap_vertex(inout vec4 VertexMODEL)
{
    //TODO: fix this matrix
    oe_nmap_normalCoords = oe_layer_tilec.st; //(oe_nmap_normalMatrix * oe_layer_tilec).st;

    //TODO: non-hard-code this
    oe_nmap_normalCoords *= 256.0/257.0;
    oe_nmap_normalCoords += 0.5/257.0;
}