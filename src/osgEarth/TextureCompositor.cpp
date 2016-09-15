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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <osgEarth/TextureCompositor>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

using namespace osgEarth;

#define LC "[TextureCompositor] "


TextureCompositor::TextureCompositor()
{
    //nop
}

bool
TextureCompositor::reserveTextureImageUnit(int&        out_unit,
                                           const char* requestor)
{
    out_unit = -1;
    unsigned maxUnits = osgEarth::Registry::instance()->getCapabilities().getMaxGPUTextureUnits();
    
    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );
    for( unsigned i=0; i<maxUnits; ++i )
    {
        if (_reservedUnits.find(i) == _reservedUnits.end())
        {
            _reservedUnits.insert( i );
            out_unit = i;
            if ( requestor )
            {
                OE_INFO << LC << "Texture unit " << i << " reserved for " << requestor << "\n";
            }
            return true;
        }
    }
    return false;
}

void
TextureCompositor::releaseTextureImageUnit(int unit)
{
    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );
    _reservedUnits.erase( unit );
}

bool
TextureCompositor::setTextureImageUnitOffLimits(int unit)
{
    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );
    if (_reservedUnits.find(unit) != _reservedUnits.end())
    {
        // uh-on. Already in use!
        return false;
    }
    else
    {
        _reservedUnits.insert( unit );
        return true;
    }
}
