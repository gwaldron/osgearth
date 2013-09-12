/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
*
* Original author: Thomas Lerman
*/
#include <osgEarthUtil/GLSLColorFilter>
#include <osgEarth/VirtualProgram>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osg/Program>
#include <OpenThreads/Atomic>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    static OpenThreads::Atomic s_uniformNameGen;

    static const char* s_localShaderSource =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "void __ENTRY_POINT__(inout vec4 color)\n"
        "{\n"
        "__CODE__ \n"
        "} \n";
}

//---------------------------------------------------------------------------

#define FUNCTION_PREFIX "oe_glsl_color_filter"

//---------------------------------------------------------------------------

GLSLColorFilter::GLSLColorFilter()
{
    init();
}

void 
GLSLColorFilter::init()
{
    m_instanceId = (++s_uniformNameGen) - 1;
}

std::string
GLSLColorFilter::getEntryPointFunctionName(void) const
{
    return (osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId);
}

void 
GLSLColorFilter::install(osg::StateSet* stateSet) const
{
    osgEarth::VirtualProgram* vp = dynamic_cast<osgEarth::VirtualProgram*>(stateSet->getAttribute(VirtualProgram::SA_TYPE));
    if (vp)
    {
        // build the local shader (unique per instance). We will
        // use a template with search and replace for this one.
        std::string entryPoint = osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId;
        std::string code = s_localShaderSource;
        osgEarth::replaceIn(code, "__ENTRY_POINT__", entryPoint);
        osgEarth::replaceIn(code, "__CODE__", _code);

        osg::Shader* main = new osg::Shader(osg::Shader::FRAGMENT, code);
        vp->setShader(entryPoint, main);
    }
}


//---------------------------------------------------------------------------

OSGEARTH_REGISTER_COLORFILTER( glsl, osgEarth::Util::GLSLColorFilter );


GLSLColorFilter::GLSLColorFilter(const Config& conf)
{
    init();
    setCode( conf.value() );
}

Config
GLSLColorFilter::getConfig() const
{
    Config conf("glsl", getCode());
    return conf;
}
