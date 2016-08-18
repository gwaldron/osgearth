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
#include <osgEarthUtil/Fog>
#include <osg/Fog>
#include <osgEarthUtil/Shaders>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

#define LC "[Fog] "

using namespace osgEarth;
using namespace osgEarth::Util;


void FogCallback::operator() (osg::StateAttribute* attr, osg::NodeVisitor* nv)
{
    // Update the oe_fog_algo uniform to reflect the current algorithm.
    osg::Fog* fog = static_cast<osg::Fog*>(attr);
    for (unsigned int i = 0; i < attr->getNumParents(); i++)
    {
        osg::StateSet* stateSet = attr->getParent(i);

        if (fog->getMode() == osg::Fog::LINEAR)
        {
            stateSet->getOrCreateUniform("oe_fog_algo", osg::Uniform::INT)->set(0);
        }
        else if (fog->getMode() == osg::Fog::EXP)
        {
            stateSet->getOrCreateUniform("oe_fog_algo", osg::Uniform::INT)->set(1);
        }
        else if (fog->getMode() == osg::Fog::EXP2)
        {
            stateSet->getOrCreateUniform("oe_fog_algo", osg::Uniform::INT)->set(2);
        }
    }
}


FogEffect::FogEffect()
{
}

FogEffect::~FogEffect()
{
    detach();
}

void FogEffect::attach( osg::StateSet* stateSet )
{
    VirtualProgram* vp = VirtualProgram::getOrCreate( stateSet );
    Shaders pkg;
    pkg.load( vp, pkg.Fog_Vertex );
    pkg.load( vp, pkg.Fog_Fragment );
    _statesets.push_back(stateSet);
}

void FogEffect::detach( osg::StateSet* stateSet )
{
    VirtualProgram* vp = VirtualProgram::get(stateSet);
    if ( vp )
    {
        Shaders pkg;
        pkg.unload( vp, pkg.Fog_Vertex );
        pkg.unload( vp, pkg.Fog_Fragment );
    }
}

void FogEffect::detach()
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