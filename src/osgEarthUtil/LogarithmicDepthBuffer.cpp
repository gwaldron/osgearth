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
*/
#include <osgEarthUtil/LogarithmicDepthBuffer>
#include <osgEarthUtil/Shaders>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

#define LC "[LogarithmicDepthBuffer] "

using namespace osgEarth;
using namespace osgEarth::Util;


LogarithmicDepthBuffer::LogarithmicDepthBuffer() :
_useFragDepth(false)
{
    _supported = Registry::capabilities().supportsGLSL();
    if ( !_supported )
    {
        OE_WARN << LC << "Not supported on this platform (no GLSL)" << std::endl;
    }
}

void
LogarithmicDepthBuffer::setUseFragDepth(bool value)
{
    _useFragDepth = value;
}

void
LogarithmicDepthBuffer::install(osg::Camera* camera)
{
    if ( camera && _supported )
    {
        // install the shader component:
        osg::StateSet* stateset = camera->getOrCreateStateSet();
        
        VirtualProgram* vp = VirtualProgram::getOrCreate( stateset );
        vp->setName("Log Depth Buffer");
        Shaders pkg;

        if ( _useFragDepth )
        {
            pkg.load( vp, pkg.LogDepthBuffer_VertFile );
            pkg.load( vp, pkg.LogDepthBuffer_FragFile );
        }
        else
        {
            pkg.load( vp, pkg.LogDepthBuffer_VertOnly_VertFile );
        }
    }
}

void
LogarithmicDepthBuffer::uninstall(osg::Camera* camera)
{
    if ( camera && _supported )
    {
        osg::StateSet* stateset = camera->getStateSet();
        if ( stateset )
        {
            VirtualProgram* vp = VirtualProgram::get( stateset );
            if ( vp )
            {
                Shaders pkg;
                pkg.unload( vp, pkg.LogDepthBuffer_FragFile );
                pkg.unload( vp, pkg.LogDepthBuffer_VertFile );
                pkg.unload( vp, pkg.LogDepthBuffer_VertOnly_VertFile );
            }
        }
    }
}
