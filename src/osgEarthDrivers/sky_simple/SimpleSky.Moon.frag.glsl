#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

in vec4 moon_TexCoord;
in float moon_Lighting;
uniform sampler2D moonTex;

out vec4 out_FragColor;

void main( void ) 
{
    if (gl_ProjectionMatrix[3][3] != 0.0)
        discard;
    vec4 color = texture(moonTex, moon_TexCoord.st);
    out_FragColor = vec4(color.rgb*moon_Lighting, color.a);
}
