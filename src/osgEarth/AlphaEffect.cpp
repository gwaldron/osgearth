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

#include <osgEarth/AlphaEffect>
#include <osgEarth/StringUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Shaders>

using namespace osgEarth;

#define LC "[AlphaEffect] "

AlphaEffect::AlphaEffect()
{
    init();
}

AlphaEffect::AlphaEffect(osg::StateSet* stateset)
{
    init();
    attach( stateset );
}

void
AlphaEffect::init()
{
    _installed = false;
    _alphaUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_alphaEffect_alpha");
    _alphaUniform->set( 1.0f );
}

AlphaEffect::~AlphaEffect()
{
    detach();
}

void
AlphaEffect::setAlpha(float value)
{
    float oldValue;
    _alphaUniform->get(oldValue);

    if (value != oldValue)
    {        
        _alphaUniform->set( value );

        if (!_installed)
        {
            for (StateSetList::iterator it = _statesets.begin(); it != _statesets.end(); ++it)
            {
                osg::ref_ptr<osg::StateSet> stateset;
                if ((*it).lock(stateset))
                {
                    install(stateset.get());
                }
            }
            _installed = true;
        }
    }
}

float
AlphaEffect::getAlpha() const
{
    float value = 1.0f;
    _alphaUniform->get(value);
    return value;
}

void
AlphaEffect::attach(osg::StateSet* stateset)
{
    if ( stateset )
    {
        _statesets.push_back(stateset);
        if (_installed)
        {
            install(stateset);
        }
    }
}

void
AlphaEffect::detach()
{
    if (_installed)
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
    }
    _statesets.clear();
}

void
AlphaEffect::detach(osg::StateSet* stateset)
{
    if ( stateset && _installed )
    {
        stateset->removeUniform( _alphaUniform.get() );
        VirtualProgram* vp = VirtualProgram::get( stateset );
        if ( vp )
        {
            Shaders pkg;
            pkg.unload( vp, pkg.AlphaEffectFragment );
        }
    }
}

void
AlphaEffect::install(osg::StateSet* stateset)
{
    if (stateset)
    {
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        if (vp)
        {
            vp->setName( "osgEarth.AlphaEffect" );
            Shaders pkg;
            pkg.load(vp, pkg.AlphaEffectFragment);
            stateset->addUniform(_alphaUniform.get());
        }
    }
}