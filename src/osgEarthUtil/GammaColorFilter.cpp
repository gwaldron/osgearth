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
*
* Original author: Thomas Lerman (ThomasL@ImSAR.com)
*/
#include <osgEarthUtil/GammaColorFilter>
#include <osgEarth/ShaderComposition>
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

        "void __ENTRY_POINT__(in int slot, inout vec4 color)\n"
        "{\n"
        "    color.rgb = pow(color.rgb, __UNIFORM_NAME__.rgb); \n"
        "}\n";
}

//---------------------------------------------------------------------------

#define FUNCTION_PREFIX "osgearthutil_gammaColorFilter_"
#define UNIFORM_PREFIX  "osgearthutil_u_gamma_"

//---------------------------------------------------------------------------

GammaColorFilter::GammaColorFilter(void)
{
    // Generate a unique name for this filter's uniform. This is necessary
    // so that each layer can have a unique uniform and entry point.
    m_instanceId = (++s_uniformNameGen) - 1;
    m_gamma = new osg::Uniform(osg::Uniform::FLOAT_VEC3, (osgEarth::Stringify() << UNIFORM_PREFIX << m_instanceId));
    m_gamma->set( osg::Vec3(1.0f, 1.0f, 1.0f) );
}

void GammaColorFilter::setGamma(float value)
{
    setGamma(osg::Vec3f(value,value,value));
}

void GammaColorFilter::setGamma(const osg::Vec3f& value)
{
    m_gamma->set(value);
}

osg::Vec3f GammaColorFilter::getGamma(void) const
{
    osg::Vec3f value;
    m_gamma->get(value);
    return (value);
}

std::string GammaColorFilter::getEntryPointFunctionName(void) const
{
    return (osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId);
}

void GammaColorFilter::install(osg::StateSet* stateSet) const
{
    // safe: will not add twice.
    stateSet->addUniform(m_gamma.get());

    osgEarth::VirtualProgram* vp = dynamic_cast<osgEarth::VirtualProgram*>(stateSet->getAttribute(osg::StateAttribute::PROGRAM));
    if (vp)
    {
        // build the local shader (unique per instance). We will
        // use a template with search and replace for this one.
        std::string entryPoint = osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId;
        std::string code = s_localShaderSource;
        osgEarth::replaceIn(code, "__UNIFORM_NAME__", m_gamma->getName());
        osgEarth::replaceIn(code, "__ENTRY_POINT__", entryPoint);

        osg::Shader* main = new osg::Shader(osg::Shader::FRAGMENT, code);
        //main->setName(entryPoint);
        vp->setShader(entryPoint, main);
    }
}
