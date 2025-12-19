#extension GL_ARB_gpu_shader_int64 : enable

#pragma vp_function oe_applyDecals, fragment, last

#pragma include Decals.h.glsl

in vec3 vp_VertexView;

// Decal bindless texture arena
layout(binding = OE_BINDING_DECAL_TEXTURES, std430) readonly buffer DecalTextures
{
    uint64_t oe_decalTextures[];
};

// returns the index of the tile containing gl_FragCoord.xy
int tileIndex()
{
    // compute the tile index for this fragment:
    int vpHeight = (u_viewport[3] - u_viewport[1]);
    ivec2 tileCoord = ivec2(gl_FragCoord.x - u_viewport[0], vpHeight - gl_FragCoord.y - u_viewport[1]) / int(u_pixelsPerTile);
    return tileCoord.y * u_numTiles.x + tileCoord.x;
}


// Find the tile containing the current fragment, and apply all decals in that tile
// that intersect.
void oe_applyDecals(inout vec4 color)
{
    DecalTile tile = oe_decalTiles[tileIndex()];

    for (int i = 0; i < tile.count; ++i)
    {
        Decal decal = oe_decals[tile.indices[i]];

        vec3 local = (decal.mvmInverse * vec4(vp_VertexView, 1.0)).xyz; // vertex projected into decal space
        vec3 bbox = vec3(decal.hx, decal.hy, decal.hz);

        if (all(lessThanEqual(abs(local), bbox)))
        {
            vec2 uv = (local.xy / bbox.xy + vec2(1.0)) * vec2(0.5);
            int ti = decal.textureIndex;
            vec4 tex = ti >= 0 ? texture(sampler2D(oe_decalTextures[ti]), uv) : vec4(1, 0, 0, 1);
            color.rgb = mix(color.rgb, tex.rgb, tex.a * decal.opacity * (1.0-u_debugTiles));
        }
    }

    // debugging overlay to show tile density
    float ramp = clamp(float(tile.count) / 5.0, 0.0, 1.0);
    vec3 debugColor = vec3(0, ramp, ramp);
    color.rgb = mix(color.rgb, debugColor, clamp(float(tile.count), 0, 1) * u_debugTiles * 0.5);
}
