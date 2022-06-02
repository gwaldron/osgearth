#pragma vp_name Wire Lines Vertex Shader Model
#pragma vp_entryPoint oe_WireLine_VS_MODEL
#pragma vp_location vertex_model

uniform float oe_WireDrawable_radius;

// Set by the InstallCameraUniform callback
uniform vec3 oe_Camera;

out vec3 vp_Normal;
out vec4 vp_Color;

// Scale the wire geometry so it covers at least 1 pixel, and scale
// the alpha (as coverage) to compensate. We assume that the wire is
// horizontal because we don't want the alpha value to change as the
// orientation changes.
//
// Given a point v = [x, y , z, 1] in eye space and projection matrix P,
// P * v = [xc, yc, zc, w]; yc / w = -(P[1][2] * z + p[1][1] * y) / z;
// the derivative diff(yc, y) gives the clip space change in height
// with respect to a change in eye space y. diff(yc, y) = -p[1][1] / z,
// or p[1][1] / w.
//
// Clip space goes from -1 to 1, so the pixel height of an object with
// a height h is screen_height * p11 * h / (2 * w).

void oe_WireLine_VS_MODEL(inout vec4 curVertex)
{
    vec4 clipVertex = gl_ModelViewProjectionMatrix * curVertex;
    // Multiply by 2 to get diameter.
    float pixSize = (2.0 * oe_WireDrawable_radius) * oe_Camera.y * gl_ProjectionMatrix[1][1]
        / (2.0 * clipVertex.w);
    float scale = 1.0;
    if (pixSize < 1.0)
    {
        scale = 1.0 / pixSize;
        vp_Color.a = pixSize;
    }
    curVertex.xyz += vp_Normal * oe_WireDrawable_radius * scale;
}
