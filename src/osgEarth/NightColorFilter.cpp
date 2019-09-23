/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
*/
#include <osgEarthUtil/NightColorFilter>
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

        "#pragma import_defines(OE_NUM_LIGHTS)\n"

        "struct osg_LightSourceParameters\n"
        "{\n"
        "    vec4 ambient;\n"
        "    vec4 diffuse;\n"
        "    vec4 specular;\n"
        "    vec4 position;\n"
        "    vec3 spotDirection;\n"
        "    float spotExponent;\n"
        "    float spotCutoff;\n"
        "    float spotCosCutoff;\n"
        "    float constantAttenuation;\n"
        "    float linearAttenuation;\n"
        "    float quadraticAttenuation;\n"
        "    bool enabled;\n"
        "};\n"

        "#if OE_NUM_LIGHTS>0\n"
        "  uniform osg_LightSourceParameters osg_LightSource[OE_NUM_LIGHTS];\n"
        "#endif\n"

        // stage global - interpolated UP vector at fragment
        "vec3 vp_Normal;\n"

        "void __ENTRY_POINT__(inout vec4 color)\n"
        "{\n"
        "#if OE_NUM_LIGHTS>0\n"
        "    vec3 L = normalize(osg_LightSource[0].position.xyz);\n"
        "    vec3 N = normalize(vp_Normal);\n"
        "    float NdotL = dot(N,L);\n"
        "    float vmin = -0.25;\n"
        "    float vmax = 0.0;\n"
        //   Remap the -0.25 to 0 to 0 to 1.0
        "    float day = (clamp( NdotL, vmin, vmax) - vmin)/(vmax-vmin);\n"
        "    color.a *= (1.0 - day);\n"
        "#endif\n"
        "} \n";
}

//---------------------------------------------------------------------------

#define FUNCTION_PREFIX "osgearthutil_nightColorFilter_"
#define UNIFORM_PREFIX  "osgearthutil_u_night_"

//---------------------------------------------------------------------------

NightColorFilter::NightColorFilter(void)
{
    init();
}

void NightColorFilter::init()
{
    // Generate a unique name for this filter's uniform. This is necessary
    // so that each layer can have a unique uniform and entry point.
    m_instanceId = (++s_uniformNameGen) - 1;
}

std::string NightColorFilter::getEntryPointFunctionName(void) const
{
    return (osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId);
}

void NightColorFilter::install(osg::StateSet* stateSet) const
{
    // safe: will not add twice.
    osgEarth::VirtualProgram* vp = dynamic_cast<osgEarth::VirtualProgram*>(stateSet->getAttribute(VirtualProgram::SA_TYPE));
    if (vp)
    {
        // build the local shader (unique per instance). We will
        // use a template with search and replace for this one.
        std::string entryPoint = osgEarth::Stringify() << FUNCTION_PREFIX << m_instanceId;
        std::string code = s_localShaderSource;
        osgEarth::replaceIn(code, "__ENTRY_POINT__", entryPoint);

        osg::Shader* main = new osg::Shader(osg::Shader::FRAGMENT, code);
        //main->setName(entryPoint);
        vp->setShader(entryPoint, main);
    }
}


//---------------------------------------------------------------------------

OSGEARTH_REGISTER_COLORFILTER( night, osgEarth::Util::NightColorFilter );


NightColorFilter::NightColorFilter(const Config& conf)
{
    init();
}

Config
NightColorFilter::getConfig() const
{
    Config conf("night");   
    return conf;
}
