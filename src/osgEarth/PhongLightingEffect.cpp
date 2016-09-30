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

#include <osgEarth/PhongLightingEffect>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderFactory>
#include <osgEarth/StringUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Shaders>

// GL_LIGHTING is not always defined on GLES so define it.
#ifndef GL_LIGHTING
    #define GL_LIGHTING 0x0B50
#endif

using namespace osgEarth;


PhongLightingEffect::PhongLightingEffect()
{
    init();
}

PhongLightingEffect::PhongLightingEffect(osg::StateSet* stateset)
{
    init();
    attach( stateset );
}

void
PhongLightingEffect::init()
{
    _supported = Registry::capabilities().supportsGLSL();
    if ( _supported )
    {
        _lightingUniform = Registry::shaderFactory()->createUniformForGLMode( GL_LIGHTING, 1 );
    }
}

void
PhongLightingEffect::setCreateLightingUniform(bool value)
{
    if ( !value )
    {        
        _lightingUniform = 0L;
    }
}

PhongLightingEffect::~PhongLightingEffect()
{
    detach();
}

void
PhongLightingEffect::attach(osg::StateSet* stateset)
{
    if ( stateset && _supported )
    {
        _statesets.push_back(stateset);
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setName( "osgEarth.PhongLightingEffect" );
        
        Shaders shaders;
        shaders.load(vp, shaders.PhongLightingVertex);
        shaders.load(vp, shaders.PhongLightingFragment);

        if ( _lightingUniform.valid() )
            stateset->addUniform( _lightingUniform.get() );
    }
}

void
PhongLightingEffect::detach()
{
    if ( _supported )
    {
        for (StateSetList::iterator it = _statesets.begin(); it != _statesets.end(); ++it)
        {
            osg::ref_ptr<osg::StateSet> stateset;
            if ( (*it).lock(stateset) )
            {
                detach( stateset );
                (*it) = 0L;
            }
        }

        _statesets.clear();
    }
}

void
PhongLightingEffect::detach(osg::StateSet* stateset)
{
    if ( stateset && _supported )
    {
        if ( _lightingUniform.valid() )
            stateset->removeUniform( _lightingUniform.get() );

        VirtualProgram* vp = VirtualProgram::get( stateset );
        if ( vp )
        {
            Shaders shaders;
            shaders.unload(vp, shaders.PhongLightingVertex);
            shaders.unload(vp, shaders.PhongLightingFragment);
        }
    }
}
