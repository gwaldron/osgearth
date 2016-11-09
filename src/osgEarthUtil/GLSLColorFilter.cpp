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
    _instanceId = (++s_uniformNameGen) - 1;
    _type.init( osg::Shader::FRAGMENT );
    _functionName.init("");
}

std::string
GLSLColorFilter::getEntryPointFunctionName() const
{
    if ( _functionName.isSet() )
        return _functionName.get();
    else
        return osgEarth::Stringify() << FUNCTION_PREFIX << _instanceId; // default
}

void 
GLSLColorFilter::install(osg::StateSet* stateSet) const
{
    osgEarth::VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
    if (vp)
    {
        if (_functionName.isSet())
        {
            osg::Shader* shader = new osg::Shader(_type.value(), _code);
            vp->setShader( getEntryPointFunctionName(), shader );
        }
        else
        {
            // build the local shader (unique per instance). We will
            // use a template with search and replace for this one.
            std::string entryPoint = getEntryPointFunctionName();
            std::string code = s_localShaderSource;
            osgEarth::replaceIn(code, "__ENTRY_POINT__", entryPoint);
            osgEarth::replaceIn(code, "__CODE__", _code);

            osg::Shader* main = new osg::Shader(_type.value(), code);
            vp->setShader(entryPoint, main);
        }
    }
}


//---------------------------------------------------------------------------

OSGEARTH_REGISTER_COLORFILTER( glsl, osgEarth::Util::GLSLColorFilter );


GLSLColorFilter::GLSLColorFilter(const Config& conf)
{
    init();
    if ( conf.hasValue("function") )
        _functionName = conf.value("function");
    if ( conf.value("type").compare("vertex") == 0 )
        _type = osg::Shader::VERTEX;
    else if ( conf.value("type").compare("fragment") == 0 )
        _type = osg::Shader::FRAGMENT;
    setCode( conf.value() );
}

Config
GLSLColorFilter::getConfig() const
{
    Config conf("glsl", getCode());
    conf.addIfSet( "function", _functionName );
    conf.addIfSet( "type", "vertex",   _type, osg::Shader::VERTEX );
    conf.addIfSet( "type", "fragment", _type, osg::Shader::FRAGMENT );
    return conf;
}
