/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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

char xyz_to_lat_lon_height_source[] =
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



char fnormal_source[] = 
"vec3 fnormal(void)\n"
"{\n"
"    //Compute the normal \n"
"    vec3 normal = gl_NormalMatrix * gl_Normal; \n"
"    normal = normalize(normal); \n"
"    return normal; \n"
"}\n";

char directionalLight_source[] = 
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


static char vert_shader_source[] =
"uniform vec3 p0, p1, p2, n0, n1, n2; \n"
"\n"
"void main (void) \n"
"{ \n"
"   // interpolate vert form barycentric coords \n"
"   float u = gl_Vertex.y; \n"
"   float v = gl_Vertex.z; \n"
"   float w = gl_Vertex.x; // 1-u-v  \n"
"   vec4 outVert = vec4( p0*w + p1*u + p2*v, gl_Vertex.w );  \n"
"   //gl_Normal = n0+w + n1*u + n2*v;  \n"
"   gl_Position = gl_ModelViewProjectionMatrix * outVert; \n"
"   //gl_Vertex = outVert; \n"
"   //gl_Position = ftransform(); \n"
"   //vec4 glenn = gl_Vertex + vec4(p0,1); \n"
"   //gl_Position = gl_ModelViewProjectionMatrix * glenn; \n"
"\n"
//"   gl_Position = outVert; \n" //gl_ModelViewProjectionMatrix * outVert; \n"
"} \n";

char frag_shader_source[] = 
"void main (void) \n"
"{ \n"
"    gl_FragColor = vec4(1.0,1.0,1.0,1.0); \n"
"} \n";

