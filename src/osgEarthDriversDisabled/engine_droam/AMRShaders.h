/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

// --------------------------------------------------------------------------

static char source_xyzToLatLonHeight[] =

"vec3 xyz_to_lat_lon_height(in vec3 xyz) \n"
"{ \n"
"   float X = xyz.x; \n"
"   float Y = xyz.y; \n"
"   float Z = xyz.z; \n"
"   float _radiusEquator = 6378137.0; \n"
"   float _radiusPolar   = 6356752.3142; \n"
"   float flattening = (_radiusEquator-_radiusPolar)/_radiusEquator;\n"
"   float _eccentricitySquared = 2*flattening - flattening*flattening;\n"
"   float p = sqrt(X*X + Y*Y);\n"
"   float theta = atan(Z*_radiusEquator , (p*_radiusPolar));\n"
"   float eDashSquared = (_radiusEquator*_radiusEquator - _radiusPolar*_radiusPolar)/(_radiusPolar*_radiusPolar);\n"
"   float sin_theta = sin(theta);\n"
"   float cos_theta = cos(theta);\n"
"\n"
"   float latitude = atan( (Z + eDashSquared*_radiusPolar*sin_theta*sin_theta*sin_theta), (p - _eccentricitySquared*_radiusEquator*cos_theta*cos_theta*cos_theta) );\n"
"   float longitude = atan(Y,X);\n"
"   float sin_latitude = sin(latitude);\n"
"   float N = _radiusEquator / sqrt( 1.0 - _eccentricitySquared*sin_latitude*sin_latitude);\n"
"   float height = p/cos(latitude) - N;\n"
"   return vec3(longitude, latitude, height);\n"
"}\n";

static char source_geodeticToXYZ[] =

"vec3 geodeticToXYZ(in vec3 geodetic) \n"
"{ \n"
"  float RADIUS_EQUATOR = 6378137.0; \n"
"  float RADIUS_POLAR   = 6356752.3142; \n"
"  float FLATTENING     = (RADIUS_EQUATOR-RADIUS_POLAR)/RADIUS_EQUATOR; \n"
"  float ECC2           = (2.0*FLATTENING) - (FLATTENING*FLATTENING); \n"
"\n"
"  float lat = geodetic.y; \n"
"  float lon = geodetic.x; \n"
"  float alt = geodetic.z; \n"
"  float sinLat = sin(lat); \n"
"  float cosLat = cos(lat); \n"
"  float n = RADIUS_EQUATOR / sqrt( 1.0 - ECC2*sinLat*sinLat ); \n"
"  float x = (n+alt)*cosLat*cos(lon); \n"
"  float y = (n+alt)*cosLat*sin(lon); \n"
"  float z = (n*(1.0 - ECC2) + alt) * sinLat; \n"
"  return vec3(x,y,z); \n"
"} \n";

static char source_rotVecToGeodetic[] =

"vec3 rotVecToGeodetic(in vec3 r) \n"
"{ \n"
"  float latitude = -asin(r.y); \n"
"  float longitude = (r.x*r.x + r.z*r.z < 0.0005 ) ? 0.0 : atan2(r.x,r.z); \n"
"  return vec3( longitude, latitude, 0.0 ); \n"
"} \n";

// --------------------------------------------------------------------------



// --------------------------------------------------------------------------

static char source_slerp[] =

"vec3 slerp(in vec3 p0, in vec3 p1, in float t) \n"
"{ \n"
"   float theta = acos( dot(p0,p1) ); \n"
"   vec3 s = ( (p0*sin(1.0-t)*theta) + p1*sin(t*theta) ) / sin(theta); \n"
"   return s * ( length(p0)+length(p1) ) * 0.5; \n"
"} \n";

// --------------------------------------------------------------------------

static char source_fnormal[] = 

"vec3 fnormal(void)\n"
"{\n"
"    //Compute the normal \n"
"    vec3 normal = gl_NormalMatrix * gl_Normal; \n"
"    normal = normalize(normal); \n"
"    return normal; \n"
"}\n";

// --------------------------------------------------------------------------

static char source_directionalLight[] = 

"void directionalLight(in int i, \n"
"                      in vec3 normal, \n"
"                      inout vec4 ambient, \n"
"                      inout vec4 diffuse, \n"
"                      inout vec4 specular) \n"
"{ \n"
"   float nDotVP;         // normal . light direction \n"
"   float nDotHV;         // normal . light half vector \n"
"   float pf;             // power factor \n"
" \n"
"   nDotVP = max(0.0, dot(normal, normalize(vec3 (gl_LightSource[i].position)))); \n"
"   nDotHV = max(0.0, dot(normal, vec3 (gl_LightSource[i].halfVector))); \n"
" \n"
"   if (nDotVP == 0.0) \n"
"   { \n"
"       pf = 0.0; \n"
"   } \n"
"   else \n"
"   { \n"
"       pf = pow(nDotHV, gl_FrontMaterial.shininess); \n"
" \n"
"   } \n"
"   ambient  += gl_LightSource[i].ambient; \n"
"   diffuse  += gl_LightSource[i].diffuse * nDotVP; \n"
"   specular += gl_LightSource[i].specular * pf; \n"
"} \n";

// --------------------------------------------------------------------------

static char source_vertShaderMain_slerpMethod[] =

"uniform vec3 c0, c1, c2; \n"
"uniform vec2 t0, t1, t2; \n"
"uniform vec3 n0, n1, n2; \n"
"varying vec2 texCoord0; \n"
"\n"
"void main (void) \n"
"{ \n"
"   // interpolate vert form barycentric coords \n"
"   float u = gl_Vertex.x; \n"
"   float v = gl_Vertex.y; \n"
"   float w = gl_Vertex.z; // 1-u-v  \n"
"   vec3 interpNormal = normalize(n0*u + n1*v + n2*w); \n"
"   vec3 geodetic = rotVecToGeodetic( interpNormal ); \n"
"   vec4 outVertex = vec4( geodeticToXYZ(geodetic), gl_Vertex.w ); \n"
"   gl_Position = gl_ModelViewProjectionMatrix * outVertex; \n"
"\n"
"   // set up the tex coords for the frad shader: \n"
"   u = gl_MultiTexCoord0.s; \n"
"   v = gl_MultiTexCoord0.t; \n"
"   w = 1.0 - u - v; \n"
"   texCoord0 = t0*u + t1*v + t2*w; \n"
"} \n";

static char source_vertShaderMain_latLonMethod[] =

"uniform vec3 c0, c1, c2; \n"
"uniform vec2 t0, t1, t2; \n"
"varying vec2 texCoord0; \n"
"\n"
"void main (void) \n"
"{ \n"
"   // interpolate vert form barycentric coords \n"
"   float u = gl_Vertex.x; \n"
"   float v = gl_Vertex.y; \n"
"   float w = gl_Vertex.z; // 1-u-v  \n"
"   vec3 outCoord3 = c0*u + c1*v + c2*w; \n"
"   vec4 outVert4 = vec4( lonLatAlt_to_XYZ( outCoord3 ), gl_Vertex.w ); \n"
"   gl_Position = gl_ModelViewProjectionMatrix * outVert4; \n"
"\n"
"   // set up the tex coords for the frad shader: \n"
"   u = gl_MultiTexCoord0.s; \n"
"   v = gl_MultiTexCoord0.t; \n"
"   w = 1.0 - u - v; \n"
"   texCoord0 = t0*u + t1*v + t2*w; \n"
"} \n";

static char source_vertShaderMain_geocentricMethod[] =

"uniform vec3 v0, v1, v2; \n"
"uniform vec2 t0, t1, t2; \n"
"varying vec2 texCoord0; \n"
"\n"
"void main (void) \n"
"{ \n"
"   // interpolate vert form barycentric coords \n"
"   float u = gl_Vertex.x; \n"
"   float v = gl_Vertex.y; \n"
"   float w = gl_Vertex.z; // 1-u-v  \n"
"   vec3 outVert3 = v0*u + v1*v + v2*w; \n"
"   float h = length(v0)*u + length(v1)*v + length(v2)*w; // interpolate height \n"
"   vec4 outVert4 = vec4( normalize(outVert3) * h, gl_Vertex.w ); \n"
"   gl_Position = gl_ModelViewProjectionMatrix * outVert4; \n"
"\n"
"   // set up the tex coords for the frad shader: \n"
"   u = gl_MultiTexCoord0.s; \n"
"   v = gl_MultiTexCoord0.t; \n"
"   w = 1.0 - u - v; \n"
"   texCoord0 = t0*u + t1*v + t2*w; \n"
"} \n";

static char source_vertShaderMain_flatMethod[] =

"uniform vec3 v0, v1, v2; \n"
"uniform vec2 t0, t1, t2; \n"
"varying vec2 texCoord0; \n"
"\n"
"void main (void) \n"
"{ \n"
"   // interpolate vert form barycentric coords \n"
"   float u = gl_Vertex.x; \n"
"   float v = gl_Vertex.y; \n"
"   float w = gl_Vertex.z; // 1-u-v  \n"
"   vec4 outVert4 = vec4( v0*u + v1*v + v2*w, gl_Vertex.w); \n"
"   gl_Position = gl_ModelViewProjectionMatrix * outVert4; \n"
"\n"
"   // set up the tex coords for the frad shader: \n"
"   u = gl_MultiTexCoord0.s; \n"
"   v = gl_MultiTexCoord0.t; \n"
"   w = 1.0 - u - v; \n"
"   texCoord0 = t0*u + t1*v + t2*w; \n"
"} \n";

// --------------------------------------------------------------------------

char source_fragShaderMain[] = 

"varying vec2 texCoord0; \n"
"uniform sampler2D tex0; \n"
"\n"
"void main (void) \n"
"{ \n"
"    gl_FragColor = texture2D( tex0, texCoord0 ); \n"
"} \n";

