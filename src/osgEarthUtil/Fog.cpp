/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthUtil/Fog>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

#define LC "[Fog] "

using namespace osgEarth;
using namespace osgEarth::Util;


namespace
{
    const char* vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "void oe_fog_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"        
        "} \n";

    const char* fs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "void oe_fog_frag(inout vec4 color) \n"
        "{ \n"
        "    const float LOG2 = 1.442695;\n"
        "    float z = gl_FragCoord.z / gl_FragCoord.w;\n"        
        "    float fogFactor = exp2( -gl_Fog.density * gl_Fog.density * z * z * LOG2 );\n"
        "    fogFactor = clamp(fogFactor, 0.0, 1.0);\n"        
        "    color.rgb = mix( gl_Fog.color.rgb, color.rgb, fogFactor);\n"
        "} \n";
}


Fog::Fog() :
TerrainEffect()
{
    init();
}

Fog::Fog(const Config& conf) :
TerrainEffect()
{
    mergeConfig(conf);
    init();
}


void
Fog::init()
{    
}

void
Fog::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();        
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_fog_vertex", vs, ShaderComp::LOCATION_VERTEX_VIEW );
        vp->setFunction( "oe_fog_frag", fs, ShaderComp::LOCATION_FRAGMENT_LIGHTING );
    }
}


void
Fog::onUninstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getStateSet();
        if ( stateset )
        {
            VirtualProgram* vp = VirtualProgram::get(stateset);
            if ( vp )
            {
                vp->removeShader( "oe_fog_vertex" );
                vp->removeShader( "oe_fog_frag" );
            }
        }
    }
}



//-------------------------------------------------------------


void
Fog::mergeConfig(const Config& conf)
{    
}

Config
Fog::getConfig() const
{
    Config conf("fog");    
    return conf;
}
