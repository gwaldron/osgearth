#version 450

#pragma include Decals.h.glsl

layout(local_size_x = OE_TILES_PER_THREAD_GROUP, local_size_y = OE_TILES_PER_THREAD_GROUP) in;


// test for intersection between decal and frustum
// by testing the decal's BS against all four frustum planes
bool intersects(in Decal decal, in Frustum frustum)
{
    float radius = length(vec3(decal.hx, decal.hy, decal.hz));
    vec3 posVS = (decal.mvm * vec4(0, 0, 0, 1)).xyz; // pos in view space
    for (int i = 0; i < 4; ++i)
    {
        if (dot(frustum.planes[i].xyz, posVS) - frustum.planes[i].w < -radius)
            return false;
    }
    return true;
}

void main()
{
    if (gl_GlobalInvocationID.x >= u_numTiles.x || gl_GlobalInvocationID.y >= u_numTiles.y)
        return;

    uint tile = gl_GlobalInvocationID.y * u_numTiles.x + gl_GlobalInvocationID.x;
    Frustum frustum = frustums[tile];

    uint total = uint(oe_decals[0].textureIndex); // first decal just holds the count
    uint found = 0;

    for (uint i = 1; i < total + 1 && found < MAX_DECALS_PER_TILE; ++i)
    {
        if (intersects(oe_decals[i], frustum))
        {
            oe_decalTiles[tile].indices[found++] = i;
        }
    }

    oe_decalTiles[tile].count = found;
}
