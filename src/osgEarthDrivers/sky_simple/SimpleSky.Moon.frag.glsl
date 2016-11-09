#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

in vec4 moon_TexCoord;
uniform sampler2D moonTex;

out vec4 out_FragColor;

void main( void ) 
{ 
   out_FragColor = texture(moonTex, moon_TexCoord.st);
}
