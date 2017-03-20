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
*/

#if 0
#include <osgEarthAnnotation/HighlightDecoration>
#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

#undef  LC
#define LC "[HighlightDecoration] "

using namespace osgEarth::Annotation;

#define FRAG_FUNCTION "oe_anno_highlight_frag"

namespace
{
    const char* fragSource =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform vec4 oe_anno_highlight_color; \n"
        "void " FRAG_FUNCTION "(inout vec4 color) {\n"
        "    color.rgb = mix(color.rgb, oe_anno_highlight_color.rgb, oe_anno_highlight_color.a); \n"
        "}\n";
}


HighlightDecoration::HighlightDecoration(const osg::Vec4f& color) :
Decoration(),
_color    (color)
{
    _supported = Registry::capabilities().supportsGLSL();
    if ( _supported )
    {
        _colorUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "oe_anno_highlight_color");
        _colorUniform->set(_color);
    }
}

void
HighlightDecoration::setColor(const osg::Vec4f& color)
{
    _color = color;
    if ( _colorUniform.valid() )
    {
        _colorUniform->set(_color);
    }
}

bool
HighlightDecoration::apply(AnnotationNode& node, bool enable)
{
    if ( _supported )
    {
        osg::StateSet* ss = node.getOrCreateStateSet();
        if ( enable )
        {
            VirtualProgram* vp = VirtualProgram::getOrCreate( ss );
            if ( vp->getPolyShader(FRAG_FUNCTION) == 0L )
            {
                vp->setFunction(FRAG_FUNCTION, fragSource, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.6f);
                ss->addUniform( _colorUniform.get() );
            }
            _colorUniform->set(_color);
        }
        else
        {
            // sets alpha=0 to disable highlighting
            _colorUniform->set(osg::Vec4f(1,1,1,0));
        }
    }
    return _supported;
}
#endif
