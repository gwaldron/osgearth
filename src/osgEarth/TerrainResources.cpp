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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <osgEarth/TerrainResources>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Layer>

using namespace osgEarth;

#define LC "[TerrainResources] "


TerrainResources::TerrainResources()
{
    //nop
}

bool
TerrainResources::reserveTextureImageUnit(int&        out_unit,
                                          const char* requestor)
{
    out_unit = -1;
    unsigned maxUnits = osgEarth::Registry::instance()->getCapabilities().getMaxGPUTextureUnits();
    
    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );
    
    // first collect a list of units that are already in use.
    std::set<int> taken;
    taken.insert(_globallyReservedUnits.begin(), _globallyReservedUnits.end());
    for (PerLayerReservedUnits::const_iterator i = _perLayerReservedUnits.begin();
        i != _perLayerReservedUnits.end();
        ++i)
    {
        taken.insert(i->second.begin(), i->second.end());
    }

    // now find the first unused one.
    for( unsigned i=0; i<maxUnits; ++i )
    {
        if (taken.find(i) == taken.end())
        {
            _globallyReservedUnits.insert( i );
            out_unit = i;
            if ( requestor )
            {
                OE_INFO << LC << "Texture unit " << i << " reserved for " << requestor << std::endl;
            }
            return true;
        }
    }
    return false;
}

bool
TerrainResources::reserveTextureImageUnit(TextureImageUnitReservation& reservation,
                                          const char* requestor)
{
    reservation._unit = -1;
    unsigned maxUnits = osgEarth::Registry::instance()->getCapabilities().getMaxGPUTextureUnits();
    
    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );
    
    // first collect a list of units that are already in use.
    std::set<int> taken;
    taken.insert(_globallyReservedUnits.begin(), _globallyReservedUnits.end());
    for (PerLayerReservedUnits::const_iterator i = _perLayerReservedUnits.begin();
        i != _perLayerReservedUnits.end();
        ++i)
    {
        taken.insert(i->second.begin(), i->second.end());
    }

    for( unsigned i=0; i<maxUnits; ++i )
    {
        if (taken.find(i) == taken.end())
        {
            _globallyReservedUnits.insert( i );
            reservation._unit = i;
            reservation._layer = 0L;
            reservation._res = this;
            if ( requestor )
            {
                OE_INFO << LC << "Texture unit " << i << " reserved for " << requestor << std::endl;
            }
            return true;
        }
    }
    return false;
}

bool
TerrainResources::reserveTextureImageUnitForLayer(TextureImageUnitReservation& reservation,
                                                  const Layer* layer,
                                                  const char* requestor)
{
    if (layer == 0L)
    {
        OE_WARN << LC << "ILLEGAL USAGE: layer must be non-null\n";
        return false;
    }

    reservation._unit = -1;
    unsigned maxUnits = osgEarth::Registry::instance()->getCapabilities().getMaxGPUTextureUnits();
    
    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );
    
    // first collect a list of units that are already in use.
    std::set<int> taken;
    taken.insert(_globallyReservedUnits.begin(), _globallyReservedUnits.end());
    ReservedUnits& layerReservedUnits = _perLayerReservedUnits[layer];
    taken.insert(layerReservedUnits.begin(), layerReservedUnits.end());

    for( unsigned i=0; i<maxUnits; ++i )
    {
        if (taken.find(i) == taken.end())
        {
            layerReservedUnits.insert( i );
            reservation._unit = i;
            reservation._layer = layer;
            reservation._res = this;
            if ( requestor )
            {
                OE_INFO << LC << "Texture unit " << i << " reserved (on layer "
                    << layer->getName() << ") for " << requestor << std::endl;
            }
            return true;
        }
    }
    return false;
}

void
TerrainResources::releaseTextureImageUnit(int unit)
{
    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );
    _globallyReservedUnits.erase( unit );
    OE_INFO << LC << "Texture unit " << unit << " released\n";
}

void
TerrainResources::releaseTextureImageUnit(int unit, const Layer* layer)
{
    if (layer == 0L)
        releaseTextureImageUnit(unit);

    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );
    PerLayerReservedUnits::iterator i = _perLayerReservedUnits.find(layer);
    if (i != _perLayerReservedUnits.end())
    {
        ReservedUnits& reservedUnits = i->second;
        reservedUnits.erase(unit);

        // if there are no more units reserved for this layer, remove the record entirely
        if (reservedUnits.empty())
        {
            _perLayerReservedUnits.erase(i);
        }

        OE_INFO << LC << "Texture unit " << unit << " released (by layer " << layer->getName() << ")" << std::endl;
    }
}

bool
TerrainResources::setTextureImageUnitOffLimits(int unit)
{
    Threading::ScopedMutexLock exclusiveLock( _reservedUnitsMutex );

    // Make sure it's not already reserved:
    if (_globallyReservedUnits.find(unit) != _globallyReservedUnits.end())
    {
        // no good! Already in use globally.
        return false;
    }

    for (PerLayerReservedUnits::const_iterator i = _perLayerReservedUnits.begin();
        i != _perLayerReservedUnits.end();
        ++i)
    {
        if (i->second.find(unit) != i->second.end())
        {
            // no good! Already in use by a layer.
            return false;
        }
    }
       
    _globallyReservedUnits.insert( unit );
    return true;
}

//........................................................................
TextureImageUnitReservation::TextureImageUnitReservation()
{
    _unit = -1;
    _layer = 0L;
}

TextureImageUnitReservation::~TextureImageUnitReservation()
{
    osg::ref_ptr<TerrainResources> res;
    if (_unit >= 0 && _res.lock(res))
    {
        res->releaseTextureImageUnit(_unit, _layer);
    }
}
