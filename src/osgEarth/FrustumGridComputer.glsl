#version 450
// inspired by https://github.com/zimengyang/ForwardPlus_Vulkan

#pragma include FrustumGrid.h.glsl


#define TILES_PER_THREAD_GROUP 16

layout(local_size_x = TILES_PER_THREAD_GROUP, local_size_y = TILES_PER_THREAD_GROUP) in;


vec4 screenToView(in vec4 screen)
{
    // screen to clip:
    vec2 uv = screen.xy / vec2(u_viewport[2], u_viewport[3]);
    uv.y = 1.0 - uv.y; // flip Y
    vec4 clip = vec4(uv * 2.0 - 1.0, screen.z, screen.w);

    // clip to view:
    vec4 view = u_invProjMatrix * clip;
    view /= view.w;
    return view;
}

vec4 createPlane(in vec3 p0, in vec3 p1, in vec3 p2)
{
    vec4 plane;
    vec3 v0 = p1 - p0;
    vec3 v1 = p2 - p0;
    plane.xyz = normalize(cross(v0, v1));
    plane.w = dot(plane.xyz, p0);
    return plane;
}

// We need to run this whenever the screen size changes.
// If we decide we need Z, it will need to run any time the near/far changes.
void main()
{
    if (gl_GlobalInvocationID.x >= u_numTiles.x || gl_GlobalInvocationID.y >= u_numTiles.y)
        return;

    uint tile = gl_GlobalInvocationID.y * u_numTiles.x + gl_GlobalInvocationID.x;

    // screen space corners of tile.
    const float z = 0.0;
    vec4 UL = vec4(gl_GlobalInvocationID.xy * u_pixelsPerTile, z, 1.0);
    vec4 UR = vec4((vec2(gl_GlobalInvocationID.x + 1, gl_GlobalInvocationID.y    ) * u_pixelsPerTile), z, 1.0);
    vec4 LL = vec4((vec2(gl_GlobalInvocationID.x,     gl_GlobalInvocationID.y + 1) * u_pixelsPerTile), z, 1.0);
    vec4 LR = vec4((vec2(gl_GlobalInvocationID.x + 1, gl_GlobalInvocationID.y + 1) * u_pixelsPerTile), z, 1.0);

    // convert to view space:
    vec3 UL_VS = screenToView(UL).xyz;
    vec3 UR_VS = screenToView(UR).xyz;
    vec3 LL_VS = screenToView(LL).xyz;
    vec3 LR_VS = screenToView(LR).xyz;

    // and make the bounding planes:
    const vec3 eye = vec3(0);

    Frustum f;
    f.planes[0] = createPlane(eye, LL_VS, UL_VS); // left plane
    f.planes[1] = createPlane(eye, UR_VS, LR_VS); // right plane
    f.planes[2] = createPlane(eye, UL_VS, UR_VS); // top plane
    f.planes[3] = createPlane(eye, LR_VS, LL_VS); // bottom plane

    frustums[tile] = f;
}
