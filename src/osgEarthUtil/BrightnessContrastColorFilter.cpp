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
#include <osgEarthUtil/BrightnessContrastColorFilter>
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
        "uniform vec2 __UNIFORM_NAME__;\n"

        "void __ENTRY_POINT__(inout vec4 color)\n"
        "{\n"
        "    color.rgb = ((color.rgb - 0.5) * __UNIFORM_NAME__.y + 0.5) * __UNIFORM_NAME__.x; \n"
        "    color.rgb = clamp(color.rgb, 0.0, 1.0); \n"
        "}\n";
}

//---------------------------------------------------------------------------

#define FUNCTION_PREFIX "osgearthutil_bcColorFilter_"
#define UNIFORM_PREFIX  "osgearthutil_u_bc_"

//---------------------------------------------------------------------------

BrightnessContrastColorFilter::BrightnessContrastColorFilter(void)
{
    init();
}

void BrightnessContrastColorFilter::init()
{
    // Generate a unique name for this filter's uniform. This is necessary
    // so that each layer can have a unique uniform and entry point.
    m_instanceId = (++s_uniformNameGen) - 1;
    m_bc = new osg::Uniform(osg::Uniform::FLOAT_VEC2, (osgEarth::Stringify() << UNIFORM_PREFIX << m_instanceId));
    m_bc->set(osg::Vec2f(1.0f, 1.0f));
}

void BrightnessContrastColorFilter::setBrightnessContrast(const osg::Vec2f& value)
{
    m_bc->set(value);
}

osg::Vec2f BrightnessContrastColorFilter::getBrightnessContrast(void) const
{
    osg::Vec2f value;
    m_bc->get(value);
    return (value);
}

std::string BrightnessContrastColorFilter::getEntryPointFunctionName(void) const
{
    return (osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId);
}

void BrightnessContrastColorFilter::install(osg::StateSet* stateSet) const
{
    // safe: will not add twice.
    stateSet->addUniform(m_bc.get());

    osgEarth::VirtualProgram* vp = dynamic_cast<osgEarth::VirtualProgram*>(stateSet->getAttribute(VirtualProgram::SA_TYPE));
    if (vp)
    {
        // build the local shader (unique per instance). We will
        // use a template with search and replace for this one.
        std::string entryPoint = osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId;
        std::string code = s_localShaderSource;
        osgEarth::replaceIn(code, "__UNIFORM_NAME__", m_bc->getName());
        osgEarth::replaceIn(code, "__ENTRY_POINT__", entryPoint);

        osg::Shader* main = new osg::Shader(osg::Shader::FRAGMENT, code);
        //main->setName(entryPoint);
        vp->setShader(entryPoint, main);
    }
}

//---------------------------------------------------------------------------

OSGEARTH_REGISTER_COLORFILTER( brightness_contrast, osgEarth::Util::BrightnessContrastColorFilter );


BrightnessContrastColorFilter::BrightnessContrastColorFilter(const Config& conf)
{
    init();

    osg::Vec2f val;
    val[0] = conf.value("b", 1.0);
    val[1] = conf.value("c", 1.0);
    setBrightnessContrast( val );
}

Config
BrightnessContrastColorFilter::getConfig() const
{
    osg::Vec2f val = getBrightnessContrast();
    Config conf("brightness_contrast");
    conf.add( "b", val[0] );
    conf.add( "c", val[1] );
    return conf;
}
