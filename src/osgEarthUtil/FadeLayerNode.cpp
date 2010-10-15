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

#include <osgEarthUtil/FadeLayerNode>
#include <osgEarthUtil/Common>
#include <osgEarth/Registry>
#include <osg/Program>
#include <osgUtil/CullVisitor>
#include <OpenThreads/ScopedLock>
#include <sstream>
#include <list>


using namespace osgEarth;
using namespace osgEarthUtil;
using namespace OpenThreads;


char vert_source[] =
                    "varying vec2 texCoords[4];\n"
                    "uniform bool osgEarth_lightingEnabled; \n"
                    "uniform bool osgEarth_lightsEnabled[8];\n"
                    "\n"
                    "\n"
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
                    "} \n"
                    "\n"
                    "vec3 fnormal(void)\n"
                    "{\n"
                    "    //Compute the normal \n"
                    "    vec3 normal = gl_NormalMatrix * gl_Normal;\n"
                    "    normal = normalize(normal);\n"
                    "    return normal;\n"
                    "}\n"
                    "\n"
                    "void main (void)\n"
                    "{\n"
                    "    if (osgEarth_lightingEnabled)\n"
                    "    {\n"
                    "    vec4 ambient = vec4(0.0); \n"
                    "    vec4 diffuse = vec4(0.0); \n"
                    "    vec4 specular = vec4(0.0); \n"
                    " \n"
                    "    vec3 normal = fnormal(); \n"
                    " \n"
                    "    for (int i = 0; i < 8; i++)\n"
                    "    {\n"
                    "      if (osgEarth_lightsEnabled[i]) directionalLight(i, normal, ambient, diffuse, specular); \n"
                    "    }\n"

                    "    vec4 color = gl_FrontLightModelProduct.sceneColor + \n"
                    "                 ambient  * gl_FrontMaterial.ambient + \n"
                    "                 diffuse  * gl_FrontMaterial.diffuse + \n"
                    "                 specular * gl_FrontMaterial.specular; \n"
                    " \n"
                    "    gl_FrontColor = color; \n"
                    "   }\n"
                    " else\n"
                    " {\n"
                    " gl_FrontColor = gl_Color;\n"
                    " }\n"
                    "    gl_Position = ftransform();\n"
                    "\n"
                    //Pass the tex coords along
                    "	 texCoords[0] = gl_MultiTexCoord0.st;\n"
                    "    texCoords[1] = gl_MultiTexCoord1.st;\n"
                    "    texCoords[2] = gl_MultiTexCoord2.st;\n"
                    "    texCoords[3] = gl_MultiTexCoord3.st;\n"
                    "}\n";


char frag_source[] =
                    "uniform int   osgearth_imagelayer_count;\n"
                    "uniform float osgearth_imagelayer_opacity[4]; \n"
                    "uniform bool osgearth_imagelayer_enabled[4]; \n"
                    "uniform sampler2D osgEarth_Layer0_unit;\n"
                    "uniform sampler2D osgEarth_Layer1_unit;\n"
                    "uniform sampler2D osgEarth_Layer2_unit;\n"
                    "uniform sampler2D osgEarth_Layer3_unit;\n"
                    "varying vec2 texCoords[4];\n"
                    "\n"
                    "void main( void )\n"
                    "{\n"
                    //Collect the colors into an array so we can index them in the upcoming loop
                    "  vec4 colors[4];\n"
                    //Initialize the colors                    
                    "  colors[0] = texture2D(osgEarth_Layer0_unit, texCoords[0]);\n"
                    "  colors[1] = texture2D(osgEarth_Layer1_unit, texCoords[1]);\n"
                    "  colors[2] = texture2D(osgEarth_Layer2_unit, texCoords[2]);\n"
                    "  colors[3] = texture2D(osgEarth_Layer3_unit, texCoords[3]);\n"
                    
                    "  vec3 color = vec3(1,1,1);\n"
                    "  int numLayersOn = 0;\n"
                    "  float maxAlpha = 0.0;\n"
                    "  for(int i=0; i<osgearth_imagelayer_count; i++) \n"
                    "  {\n"
                    "     if (osgearth_imagelayer_enabled[i])\n"
                    "     {\n"
                    "       vec4 texel = colors[i];\n"
                    "       float alpha = texel.a * osgearth_imagelayer_opacity[i];\n"
                    "       color = mix(color, texel.rgb, alpha);\n"
                    "       numLayersOn++;\n"
                    "       if (maxAlpha < alpha) maxAlpha = alpha;\n"
                    "     }\n"
                    "  }\n"
                    "  if (numLayersOn == 0) maxAlpha = 1.0;\n"
                    "  gl_FragColor = gl_Color * vec4(color, maxAlpha); \n"                  
                    "}\n";                    


FadeLayerNode::FadeLayerNode( Map* map, const MapNodeOptions& mapOptions) :
_map( map ),
_mapNodeOptions(mapOptions)
{
    if (_mapNodeOptions.getTerrainOptions().compositingTechnique() != TerrainOptions::COMPOSITING_MULTIPASS &&
        osgEarth::Registry::instance()->getCapabilities().supportsGLSL() )
	{
		osg::Program* program = new osg::Program;
		_vertShader = new osg::Shader( osg::Shader::VERTEX, vert_source );
		_fragShader = new osg::Shader( osg::Shader::FRAGMENT, frag_source );

		program->addShader( _vertShader.get() );
		program->addShader( _fragShader.get() );

		getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
		getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);
		getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	}
}

void FadeLayerNode::traverse(osg::NodeVisitor& nv)
{
    osg::Group::traverse(nv);
}