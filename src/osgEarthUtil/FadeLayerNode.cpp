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

#include <osgEarthUtil/FadeLayerNode>
#include <osgEarthUtil/Common>
#include <osg/Program>
#include <osgUtil/CullVisitor>
#include <OpenThreads/ScopedLock>
#include <sstream>
#include <list>


using namespace osgEarth;
using namespace osgEarthUtil;
using namespace OpenThreads;


char vert_source[] ="varying vec2 texCoord0;\n"
                    "varying vec2 texCoord1;\n"
                    "varying vec2 texCoord2;\n"
                    "varying vec2 texCoord3;\n"
                    "uniform bool lightingEnabled; \n"
                    "uniform bool light0Enabled; \n"
                    "uniform bool light1Enabled; \n"
                    "uniform bool light2Enabled; \n"
                    "uniform bool light3Enabled; \n"
                    "uniform bool light4Enabled; \n"
                    "uniform bool light5Enabled; \n"
                    "uniform bool light6Enabled; \n"
                    "uniform bool light7Enabled; \n"
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
                    "    if (lightingEnabled)\n"
                    "    {\n"
                    "    vec4 ambient = vec4(0.0); \n"
                    "    vec4 diffuse = vec4(0.0); \n"
                    "    vec4 specular = vec4(0.0); \n"
                    " \n"
                    "    vec3 normal = fnormal(); \n"
                    " \n"
                    "    if (light0Enabled) directionalLight(0, normal, ambient, diffuse, specular); \n"
                    "    if (light1Enabled) directionalLight(1, normal, ambient, diffuse, specular); \n"
                    "    if (light2Enabled) directionalLight(2, normal, ambient, diffuse, specular); \n"
                    "    if (light3Enabled) directionalLight(3, normal, ambient, diffuse, specular); \n"
                    "    if (light4Enabled) directionalLight(4, normal, ambient, diffuse, specular); \n"
                    "    if (light5Enabled) directionalLight(5, normal, ambient, diffuse, specular); \n"
                    "    if (light6Enabled) directionalLight(6, normal, ambient, diffuse, specular); \n"
                    "    if (light7Enabled) directionalLight(7, normal, ambient, diffuse, specular); \n"

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
                    "	 texCoord0 = gl_MultiTexCoord0.st;\n"
                    "    texCoord1 = gl_MultiTexCoord1.st;\n"
                    "    texCoord2 = gl_MultiTexCoord2.st;\n"
                    "    texCoord3 = gl_MultiTexCoord3.st;\n"
                    "}\n";


char frag_source[] = "uniform sampler2D osgEarth_Layer0_unit;\n"
                    "uniform float osgEarth_Layer0_opacity;\n"
                    "varying vec2 texCoord0;\n"
                    "uniform bool osgEarth_Layer0_enabled;\n"
                    "\n"
                    "uniform sampler2D osgEarth_Layer1_unit;\n"
                    "uniform float osgEarth_Layer1_opacity;\n"
                    "varying vec2 texCoord1;\n"
                    "uniform bool osgEarth_Layer1_enabled;\n"
                    "\n"
                    "uniform sampler2D osgEarth_Layer2_unit;\n"
                    "uniform float osgEarth_Layer2_opacity;\n"
                    "varying vec2 texCoord2;\n"
                    "uniform bool osgEarth_Layer2_enabled;\n"
                    "\n"
                    "uniform sampler2D osgEarth_Layer3_unit;\n"
                    "uniform float osgEarth_Layer3_opacity;\n"
                    "varying vec2 texCoord3;\n"
                    "uniform bool osgEarth_Layer3_enabled;\n"
                    "\n"
                    "void main (void )\n"
                    "{\n"
                    "  //Get the fragment for texture 0\n"
                    "  int numLayersOn = 0;\n"
                    "  vec4 tex0 = vec4(0.0,0.0,0.0,0.0);\n"
                    "  if (osgEarth_Layer0_enabled)\n"
                    "  {\n"
                    "    tex0 = texture2D(osgEarth_Layer0_unit, texCoord0);\n"
                    "    numLayersOn++;\n"
                    "  }\n"
                    "  tex0.a *= osgEarth_Layer0_opacity;\n"
                    "\n"
                    "  //Get the fragment for texture 1\n"
                    "  vec4 tex1 = vec4(0.0,0.0,0.0,0.0);\n"
                    "  if (osgEarth_Layer1_enabled)\n"
                    "  {\n"
                    "    tex1 = texture2D(osgEarth_Layer1_unit, texCoord1);\n"
                    "    numLayersOn++;\n"
                    "  }\n"
                    "  tex1.a *= osgEarth_Layer1_opacity;\n"
                    "\n"
					"  //Get the fragment for texture 2\n"
                    "  vec4 tex2 = vec4(0.0,0.0,0.0,0.0);\n"
                    "  if (osgEarth_Layer2_enabled)\n"
                    "  {\n"
                    "    tex2 = texture2D(osgEarth_Layer2_unit, texCoord2);\n"
                    "    numLayersOn++;\n"
                    "  }\n"
                    "  tex2.a *= osgEarth_Layer2_opacity;\n"
                    "\n"
                    "  //Get the fragment for texture 3\n"
                    "  vec4 tex3 = vec4(0.0,0.0,0.0,0.0);\n"
                    "  if (osgEarth_Layer3_enabled)\n"
                    "  {\n"
                    "    tex3 = texture2D(osgEarth_Layer3_unit, texCoord3);\n"
                    "    numLayersOn++;\n"
                    "  }\n"
                    "  tex3.a *= osgEarth_Layer3_opacity;\n"
                    "\n"

                    "  if (numLayersOn > 0)\n"
                    "  {\n"
                    "    //Interpolate the color between the first layer and second\n"
                    "    vec3 c = mix(tex0.rgb, tex1.rgb, tex1.a);\n"
                    "\n"
                    "    c = mix(c, tex2.rgb, tex2.a);\n"
                    " \n"
                    "    c = mix(c, tex3.rgb, tex3.a);\n"
                    " \n"
                    "    //Take the maximum alpha for the final destination alpha\n"
                    "    float a = tex0.a;\n"
                    "    if (a < tex1.a) a = tex1.a;\n"
                    "    if (a < tex2.a) a = tex2.a;\n"
                    "    if (a < tex3.a) a = tex3.a;\n"
                    "\n"
                    "    gl_FragColor = gl_Color * vec4(c, a);\n"
                    "  }\n"
                    "  else\n"
                    "  {\n"
                    "    //No layers are on, so just use the underlying color\n"
                    "    gl_FragColor = gl_Color;\n"
                    "  }\n"
                    "}\n"
                    "\n";


FadeLayerNode::FadeLayerNode( Map* map, const MapEngineProperties& mapEngineProperties) :
_map( map ),
_mapEngineProperties(mapEngineProperties)
{
	if (_mapEngineProperties.layeringTechnique() == MapEngineProperties::LAYERING_MULTITEXTURE)
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


void FadeLayerNode::updateStateSet()
{
	if (_mapEngineProperties.layeringTechnique() == MapEngineProperties::LAYERING_MULTITEXTURE)
	{
		osg::StateSet* stateset = getStateSet();
		unsigned int maxLayers = 4;
		for (unsigned int i = 0; i < maxLayers; ++i)
		{
			float opacity = 1.0f;
			bool enabled = false;

			if (i < _map->getImageMapLayers().size())
			{
			  opacity = _map->getImageMapLayers()[i]->opacity().value();
			  enabled = _map->getImageMapLayers()[i]->enabled().value();
			}

			std::stringstream ss;
			ss << "osgEarth_Layer" << i << "_unit";
			std::string ssStr;
			ssStr = ss.str();
			osg::Uniform* unitUniform = stateset->getOrCreateUniform(ssStr, osg::Uniform::INT);
			unitUniform->set( (int)i );

			ss.str("");
			ss << "osgEarth_Layer" << i << "_enabled";
			ssStr = ss.str();
			osg::Uniform* enabledUniform = stateset->getOrCreateUniform(ssStr, osg::Uniform::BOOL);
			enabledUniform->set(enabled);

			ss.str("");
			ss << "osgEarth_Layer" << i << "_opacity";
			ssStr = ss.str();
			osg::Uniform* opacityUniform = stateset->getOrCreateUniform(ssStr, osg::Uniform::FLOAT);
			opacityUniform->set(opacity);
		}
	}
}

typedef std::list<const osg::StateSet*> StateSetStack;
osg::StateAttribute::GLModeValue getModeValue(const StateSetStack& statesetStack, osg::StateAttribute::GLMode mode)
{
    osg::StateAttribute::GLModeValue base_val = osg::StateAttribute::ON;
    for(StateSetStack::const_iterator itr = statesetStack.begin();
        itr != statesetStack.end();
        ++itr)
    {
        osg::StateAttribute::GLModeValue val = (*itr)->getMode(mode);
        if ((val & ~osg::StateAttribute::INHERIT)!=0)
        {
            if ((val & osg::StateAttribute::PROTECTED)!=0 ||
                (base_val & osg::StateAttribute::OVERRIDE)==0)
            {
                base_val = val;
            }
        }
    }
    return base_val;
}

void FadeLayerNode::traverse(osg::NodeVisitor& nv)
{
	if (_mapEngineProperties.layeringTechnique() == MapEngineProperties::LAYERING_MULTITEXTURE)
	{
		if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
		{
			osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
			if (cv)
			{
				StateSetStack statesetStack;

				osgUtil::StateGraph* sg = cv->getCurrentStateGraph();
				while(sg)
				{
					const osg::StateSet* stateset = sg->getStateSet();
					if (stateset)
					{
						statesetStack.push_front(stateset);
					}                
					sg = sg->_parent;
				}

				osg::StateAttribute::GLModeValue lightingEnabled = getModeValue(statesetStack, GL_LIGHTING);     
				osg::Uniform* lightingEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("lightingEnabled", osg::Uniform::BOOL);
				lightingEnabledUniform->set((lightingEnabled & osg::StateAttribute::ON)!=0);

				//GL_LIGHT0
				{
					osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT0);     
					osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("light0Enabled", osg::Uniform::BOOL);
					lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
				}

				//GL_LIGHT1
				{
					osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT1);     
					osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("light1Enabled", osg::Uniform::BOOL);
					lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
				}

				//GL_LIGHT2
				{
					osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT2);     
					osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("light2Enabled", osg::Uniform::BOOL);
					lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
				}

				//GL_LIGHT3
				{
					osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT3);     
					osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("light3Enabled", osg::Uniform::BOOL);
					lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
				}

				//GL_LIGHT4
				{
					osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT4);     
					osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("light4Enabled", osg::Uniform::BOOL);
					lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
				}

				//GL_LIGHT5
				{
					osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT5);     
					osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("light5Enabled", osg::Uniform::BOOL);
					lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
				}

				//GL_LIGHT6
				{
					osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT6);     
					osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("light6Enabled", osg::Uniform::BOOL);
					lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
				}

				//GL_LIGHT7
				{
					osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT7);     
					osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("light7Enabled", osg::Uniform::BOOL);
					lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
				}
			}
		}
		updateStateSet();
	}
    osg::Group::traverse(nv);
}