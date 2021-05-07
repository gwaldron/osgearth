#version 430
#pragma include Splat.GroundCover.Types.glsl

layout(local_size_x=1, local_size_y=1, local_size_z=1) in;


uniform vec3 oe_VisibleLayer_ranges;
uniform vec3 oe_Camera;

uniform int oe_gc_numCommands; // total number of draw commands
uniform int oe_pass; // compute pass (0=cull, 1=sort)


uniform float HARD_CODED_MAX_SSE = 100.0; // pixels

void cull()
{
    uint i = gl_GlobalInvocationID.x;

    // initialize to -1, meaning the instance will be ignored.
    instance[i].drawId = -1;

    // Not a valid instance because it wasn't collected by the generator CS.
    // We have to check this b/c we're dispatching the maximum possible
    // number of instances. Instead, we could read back the results of the 
    // Generator and use that as a count. TODO: investigate.
    if (i >= instanceHeader.count)
        return;

    // Which tile did this come from
    uint tileNum = instance[i].tileNum;

    // Bring into view space:
    vec4 vertex_view = tile[tileNum].modelViewMatrix * instance[i].vertex;

    // range culling:
    //const float HARD_CODED_MODEL_RANGE = 400.0;

    float range = -vertex_view.z;
    float maxRange = oe_VisibleLayer_ranges[1] / oe_Camera.z;

    // distance culling:
    if (range >= maxRange)
        return;

    // frustum culling:
    vec4 clipLL = gl_ProjectionMatrix * (vertex_view - vec4(0.5*instance[i].width, 0, 0, 0));
    clipLL.xy /= clipLL.w;
    if (clipLL.x > 1.0 || clipLL.y > 1.0)
        return;

    vec4 clipUR = gl_ProjectionMatrix * (vertex_view + vec4(0.5*instance[i].width, instance[i].height, 0, 0));
    clipUR.xy /= clipUR.w;
    if (clipUR.x < -1.0 || clipUR.y < -1.0)
        return;


    // Model versus billboard selection:
    bool chooseModel = instance[i].modelId >= 0;

    // If model, make sure we're within the SSE limit:
    vec2 pixelSizeRatio = vec2(1);
    if (chooseModel)
    {
        vec2 pixelSize = 0.5*(clipUR.xy-clipLL.xy) * oe_Camera.xy;
        pixelSizeRatio = pixelSize/vec2(HARD_CODED_MAX_SSE);
        if (all(lessThan(pixelSizeRatio, vec2(1))))
            chooseModel = false;
        //if (all(lessThan(pixelSize, vec2(HARD_CODED_MAX_SSE))))
        //    chooseModel = false;
    }

    // chose a billboard, but there isn't one; bail.
    if (chooseModel == false && instance[i].sideSampler == 0)
        return;

    // Sort into DrawCommands based on range. anything beyond the
    // legal model range goes into the billboard command (cmd[0])
    //float modelRange = HARD_CODED_MODEL_RANGE / oe_Camera.z;

    // If we're out of model range and there is no billboard image, reject
    //if (range > modelRange && instance[i].sideIndex < 0)
        //return;

    // Experimental range-based thinning
    //float ratio = (range/maxRange);
    //if (ratio > 0.35) {
    //    ratio = 10 - 10*ratio;
    //    if (instance[i].instanceId % uint(ratio) == 0)
    //        return;
    //}

    // "DrawID" is the index of the DrawElementsIndirect command we will
    // use to draw this instance. Command[0] is the billboard group; all
    // others are unique 3D models.
    instance[i].drawId = chooseModel ? instance[i].modelId + 1 : 0;

    instance[i].pixelSizeRatio = min(pixelSizeRatio.x, pixelSizeRatio.y);
    
    // for each command FOLLOWING the one we just picked,
    // increment the baseInstance number. We should end up with
    // the correct baseInstance for each command by the end of the cull pass.
    for (uint drawId = instance[i].drawId + 1; drawId < oe_gc_numCommands; ++drawId)
    {
        atomicAdd(cmd[drawId].baseInstance, 1);
    }
}

// Sorts each valid instance into the appropriate draw command
void sort()
{
    uint i = gl_GlobalInvocationID.x;

    int drawId = instance[i].drawId;
    if (drawId >= 0)
    {
        // find the index of the first instance for this bin:
        uint cmdStartIndex = cmd[drawId].baseInstance;

        // bump the instance count for this command; the new instanceCount
        // is also the index within the command:
        uint instanceIndex = atomicAdd(cmd[drawId].instanceCount, 1);

        // copy to the right place in the render list
        uint index = cmdStartIndex + instanceIndex;
        render[index] = instance[i];
    }
}

void main()
{
    if (oe_pass == 0)
        cull();
    else
        sort();
}
