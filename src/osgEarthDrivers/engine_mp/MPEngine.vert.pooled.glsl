#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

uniform mat4 oe_layer_texMatrix;
varying vec4 oe_layer_texc;
varying vec4 oe_layer_tilec;
varying vec3 oe_Normal;

uniform sampler2D oe_tile_elevationTex;
uniform mat4 oe_tile_elevationTexMatrix;

void oe_mp_pooled_vert(inout vec4 vertexModel)
{
    // Texture coordinate for the tile (always 0..1)
    oe_layer_tilec = gl_MultiTexCoord$MP_PRIMARY_UNIT;

    // Texture coordinate for the color texture (scale,bias)
    oe_layer_texc = oe_layer_texMatrix * oe_layer_tilec;

    // Sample the elevation texture and move the vertex accordingly.
    vec4 elevc = oe_tile_elevationTexMatrix * oe_layer_tilec;
    float elev = texture2D(oe_tile_elevationTex, elevc.st).r;
    vertexModel.xyz = vertexModel.xyz + oe_Normal*elev;
}
