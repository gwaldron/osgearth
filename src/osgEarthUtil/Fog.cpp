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
*/
#include <osgEarthUtil/Fog>
#include <osgEarthUtil/Shaders>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

#define LC "[Fog] "

using namespace osgEarth;
using namespace osgEarth::Util;


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
    pkg.loadFunction( vp, pkg.Fog_Vertex );
    pkg.loadFunction( vp, pkg.Fog_Fragment );
    _statesets.push_back(stateSet);
}

void FogEffect::detach( osg::StateSet* stateSet )
{
    VirtualProgram* vp = VirtualProgram::get(stateSet);
    if ( vp )
    {
        Shaders pkg;
        pkg.unloadFunction( vp, pkg.Fog_Vertex );
        pkg.unloadFunction( vp, pkg.Fog_Fragment );
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