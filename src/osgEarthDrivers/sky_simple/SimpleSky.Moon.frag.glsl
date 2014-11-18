#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

varying vec4 moon_TexCoord;
uniform sampler2D moonTex;

void main( void ) 
{ 
   gl_FragColor = texture2D(moonTex, moon_TexCoord.st);
}
