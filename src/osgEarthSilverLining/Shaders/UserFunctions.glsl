// Hook to override computation of fog on clouds
// fogExponent is squared to determine the blending of clouds with the background
// fogBlend is used for blending fogColor with the cloud color.
void overrideBillboardFog(in vec3 eyePosition, inout vec4 fogColor, inout float fogExponent, inout float fogBlend, inout float fogFade)
{

}

// Cloud puffs are drawn additively, so dark = more transparent. This allows you to influence the color used for a given
// puff. This color is multiplied with the puff texture in the fragment shader, and nothing more.
// By default the fogged color is multiplied by the cloudFade, voxelFade, and fogFade, which apply alpha effects on the
// cloud layer as a whole, on the individual puff (ie while clouds are growing in real time,) and from fog (we simulate atmospheric
// scattering by blending the puff into the sky behind it based on distance.)
// modelPos is the position of the puff vertex, before billboarding has been applied, and rotated into a Y-is-up system. So, the altitude
// of this vertex will be in modelPos.y.
void overrideBillboardColor(in vec3 eyePosition, in vec4 modelPos, in vec4 foggedColor, in float cloudFade, in float voxelFade, in float fogFade, inout vec4 finalColor)
{

}

// Override the vertex color of a point on the sky box. The position in world space (relative to the camera) is given, as is
// our sky color before and after applying fog.
void overrideSkyColor(in vec3 vertexPos, in float fogDensity, in vec4 preFogColor, inout vec4 finalColor)
{

}


// Overrides colors of cirrus and cirrocumulus clouds
void overrideCirrusLighting(in vec3 lightColor, in vec3 fogColor, in float fogBlend, in float alpha, inout vec4 finalColor)
{

}

// Override the star colors.
void overrideStars(in vec4 worldPos, in float magnitude, in vec4 starColor, in vec4 fogColor, in float fogDensity, inout vec4 finalColor)
{

}

// override the color calculation for the Stratus clouds.
// finalColor = vec4(color.x, color.y, color.z, color.w * fadeAndDisplacementFactor);
void overrideStratocumulusColor(inout vec4 finalColor)
{

}

// osgEarth

#if __VERSION__ > 140
    out float oe_LogDepth_clipz;
#else
    varying float oe_LogDepth_clipz;
#endif
uniform mat4 osg_ProjectionMatrix;

vec4 oe_LogDepth_vert(in vec4 clip)
{
    if (osg_ProjectionMatrix[3][3] == 0) // perspective only
    {
        mat4 clip2view = inverse(osg_ProjectionMatrix);
        vec4 farPoint = clip2view * vec4(0,0,1,1);
        float FAR = -farPoint.z / farPoint.w;

        const float C = 0.001;
        float FC = 1.0 / log(FAR*C + 1);
        oe_LogDepth_clipz = log(max(1e-6, clip.w*C + 1.0))*FC;
        clip.z = (2.0*oe_LogDepth_clipz - 1.0)*clip.w;
    }
    else
    {
        oe_LogDepth_clipz = -1.0;
    }
    return clip;
}

// Provides a point to override the final value of gl_Position.
// Useful for implementing logarithmic depth buffers etc.
vec4 overridePosition(in vec4 position)
{
    return oe_LogDepth_vert(position);
    //return position;
}