/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*
* Original author: Thomas Lerman
*/
#include <osgEarthUtil/CMYKColorFilter>
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
        "uniform vec4 __UNIFORM_NAME__;\n"

        "void __ENTRY_POINT__(inout vec4 color)\n"
        "{\n"
        // apply cmy (negative of rgb):
        "   color.rgb -= __UNIFORM_NAME__.xyz; \n"
        // apply black (applies to all colors):
        "   color.rgb -= __UNIFORM_NAME__.w; \n"
        "   color.rgb = clamp(color.rgb, 0.0, 1.0); \n"
        "}\n";
}

//---------------------------------------------------------------------------

#define FUNCTION_PREFIX "osgearthutil_cmykColorFilter_"
#define UNIFORM_PREFIX  "osgearthutil_u_cmyk_"

//---------------------------------------------------------------------------

CMYKColorFilter::CMYKColorFilter(void)
{
    init();
}

void
CMYKColorFilter::init()
{
    // Generate a unique name for this filter's uniform. This is necessary
    // so that each layer can have a unique uniform and entry point.
    m_instanceId = (++s_uniformNameGen) - 1;
    m_cmyk = new osg::Uniform(osg::Uniform::FLOAT_VEC4, (osgEarth::Stringify() << UNIFORM_PREFIX << m_instanceId));
    m_cmyk->set(osg::Vec4f(0.0f, 0.0f, 0.0f, 0.0f));
}


// CMY (without the K): http://forums.adobe.com/thread/428899
void CMYKColorFilter::setCMYOffset(const osg::Vec3f& value)
{
    osg::Vec4f cmyk;
    // find the minimum of all values
    cmyk[3] = 1.0;
    if (value[0] < cmyk[3])
    {
        cmyk[3] = value[0];
    }
    if (value[1] < cmyk[3])
    {
        cmyk[3] = value[1];
    }
    if (value[2] < cmyk[3])
    {
        cmyk[3] = value[2];
    }

    if (cmyk[3] == 1.0)
    {	// black
        cmyk[0] = cmyk[1] = cmyk[2] = 0.0;
    }
    else
    {
        cmyk[0] = (value[0] - cmyk[3]) / (1.0 - cmyk[3]);
        cmyk[1] = (value[1] - cmyk[3]) / (1.0 - cmyk[3]);
        cmyk[2] = (value[2] - cmyk[3]) / (1.0 - cmyk[3]);
    }

    setCMYKOffset(cmyk);
}

osg::Vec3f CMYKColorFilter::getCMYOffset(void) const
{
    osg::Vec4f cmyk = getCMYKOffset();
    osg::Vec3f cmy;

    if (cmyk[3] == 1.0)
    {
        cmy[0] = cmy[1] = cmy[2] = 1.0;
    }
    else
    {
        cmy[0] = (cmyk[0] * (1.0 - cmyk[3])) + cmyk[3];
        cmy[1] = (cmyk[1] * (1.0 - cmyk[3])) + cmyk[3];
        cmy[2] = (cmyk[2] * (1.0 - cmyk[3])) + cmyk[3];
    }
    return (cmy);
}

void CMYKColorFilter::setCMYKOffset(const osg::Vec4f& value)
{
    m_cmyk->set(value);
}

osg::Vec4f CMYKColorFilter::getCMYKOffset(void) const
{
    osg::Vec4f value;
    m_cmyk->get(value);
    return (value);
}

std::string CMYKColorFilter::getEntryPointFunctionName(void) const
{
    return (osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId);
}

void CMYKColorFilter::install(osg::StateSet* stateSet) const
{
    // safe: will not add twice.
    stateSet->addUniform(m_cmyk.get());

    osgEarth::VirtualProgram* vp = dynamic_cast<osgEarth::VirtualProgram*>(stateSet->getAttribute(VirtualProgram::SA_TYPE));
    if (vp)
    {
        // build the local shader (unique per instance). We will
        // use a template with search and replace for this one.
        std::string entryPoint = osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId;
        std::string code = s_localShaderSource;
        osgEarth::replaceIn(code, "__UNIFORM_NAME__", m_cmyk->getName());
        osgEarth::replaceIn(code, "__ENTRY_POINT__", entryPoint);

        osg::Shader* main = new osg::Shader(osg::Shader::FRAGMENT, code);
        //main->setName(entryPoint);
        vp->setShader(entryPoint, main);
    }
}


//---------------------------------------------------------------------------

OSGEARTH_REGISTER_COLORFILTER( cmyk, osgEarth::Util::CMYKColorFilter );


CMYKColorFilter::CMYKColorFilter(const Config& conf)
{
    init();

    osg::Vec4f val;
    val[0] = conf.value("c", 0.0);
    val[1] = conf.value("m", 0.0);
    val[2] = conf.value("y", 0.0);
    val[3] = conf.value("k", 0.0);
    setCMYKOffset( val );
}

Config
CMYKColorFilter::getConfig() const
{
    osg::Vec4f val = getCMYKOffset();
    Config conf("cmyk");
    conf.add( "c", val[0] );
    conf.add( "m", val[1] );
    conf.add( "y", val[2] );
    conf.add( "k", val[3] );
    return conf;
}
