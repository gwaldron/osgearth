/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthUtil/RGBColorFilter>
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
        "#version 110\n"
        "uniform vec3 __UNIFORM_NAME__;\n"

        "void __ENTRY_POINT__(inout vec4 color)\n"
        "{\n"
        "    color.rgb = clamp(color.rgb + __UNIFORM_NAME__.rgb, 0.0, 1.0); \n"
        "} \n";
}

//---------------------------------------------------------------------------

#define FUNCTION_PREFIX "osgearthutil_rgbColorFilter_"
#define UNIFORM_PREFIX  "osgearthutil_u_rgb_"

//---------------------------------------------------------------------------

RGBColorFilter::RGBColorFilter(void)
{
    init();
}

void RGBColorFilter::init()
{
    // Generate a unique name for this filter's uniform. This is necessary
    // so that each layer can have a unique uniform and entry point.
    m_instanceId = (++s_uniformNameGen) - 1;
    m_rgb = new osg::Uniform(osg::Uniform::FLOAT_VEC3, (osgEarth::Stringify() << UNIFORM_PREFIX << m_instanceId));
    m_rgb->set(osg::Vec3f(0.0f, 0.0f, 0.0f));
}

void RGBColorFilter::setRGBOffset(const osg::Vec3f& value)
{
    m_rgb->set(value);
}

osg::Vec3f RGBColorFilter::getRGBOffset(void) const
{
    osg::Vec3f value;
    m_rgb->get(value);
    return (value);
}

std::string RGBColorFilter::getEntryPointFunctionName(void) const
{
    return (osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId);
}

void RGBColorFilter::install(osg::StateSet* stateSet) const
{
    // safe: will not add twice.
    stateSet->addUniform(m_rgb.get());

    osgEarth::VirtualProgram* vp = dynamic_cast<osgEarth::VirtualProgram*>(stateSet->getAttribute(VirtualProgram::SA_TYPE));
    if (vp)
    {
        // build the local shader (unique per instance). We will
        // use a template with search and replace for this one.
        std::string entryPoint = osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId;
        std::string code = s_localShaderSource;
        osgEarth::replaceIn(code, "__UNIFORM_NAME__", m_rgb->getName());
        osgEarth::replaceIn(code, "__ENTRY_POINT__", entryPoint);

        osg::Shader* main = new osg::Shader(osg::Shader::FRAGMENT, code);
        //main->setName(entryPoint);
        vp->setShader(entryPoint, main);
    }
}


//---------------------------------------------------------------------------

OSGEARTH_REGISTER_COLORFILTER( rgb, osgEarth::Util::RGBColorFilter );


RGBColorFilter::RGBColorFilter(const Config& conf)
{
    init();

    osg::Vec3f val;
    val[0] = conf.value("r", 0.0);
    val[1] = conf.value("g", 0.0);
    val[2] = conf.value("b", 0.0);
    setRGBOffset( val );
}

Config
RGBColorFilter::getConfig() const
{
    osg::Vec3f val = getRGBOffset();
    Config conf("rgb");
    conf.add( "r", val[0] );
    conf.add( "g", val[1] );
    conf.add( "b", val[2] );
    return conf;
}
