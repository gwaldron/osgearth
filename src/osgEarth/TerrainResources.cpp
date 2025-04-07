/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/TerrainResources>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Layer>

using namespace osgEarth;

#define LC "[TerrainResources] "


TerrainResources::TerrainResources()
{
    // Unit 0 cannot be reserved
    _globallyReservedUnits.insert(0);
}

bool
TerrainResources::reserveTextureImageUnit(int& out_unit, const char* requestor)
{
    out_unit = -1;
    unsigned maxUnits = osgEarth::Registry::instance()->getCapabilities().getMaxGPUTextureUnits();
    
    std::lock_guard<std::mutex> exclusiveLock( _reservedUnitsMutex );
    
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
                OE_DEBUG << LC << "Texture unit " << i << " reserved for " << requestor << std::endl;
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
    
    std::lock_guard<std::mutex> exclusiveLock( _reservedUnitsMutex );
    
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
                OE_DEBUG << LC << "Texture unit " << i << " reserved for " << requestor << std::endl;
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
    
    std::lock_guard<std::mutex> exclusiveLock( _reservedUnitsMutex );
    
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
                OE_DEBUG << LC << "Texture unit " << i << " reserved (on layer "
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
    std::lock_guard<std::mutex> exclusiveLock( _reservedUnitsMutex );
    _globallyReservedUnits.erase( unit );
    OE_DEBUG << LC << "Texture unit " << unit << " released" << std::endl;
}

void
TerrainResources::releaseTextureImageUnit(int unit, const Layer* layer)
{
    if (layer == 0L)
        releaseTextureImageUnit(unit);

    std::lock_guard<std::mutex> exclusiveLock( _reservedUnitsMutex );
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

        OE_DEBUG << LC << "Texture unit " << unit << " released (by layer " << layer->getName() << ")" << std::endl;
    }
}

bool
TerrainResources::setTextureImageUnitOffLimits(int unit)
{
    std::lock_guard<std::mutex> exclusiveLock( _reservedUnitsMutex );

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


float
TerrainResources::getVisibilityRangeHint(unsigned lod) const
{
    return lod < _visibilityRanges.size() ? _visibilityRanges[lod] : FLT_MAX;
}

void
TerrainResources::setVisibilityRangeHint(unsigned lod, float range)
{
    if (_visibilityRanges.size() <= lod)
        _visibilityRanges.resize(lod+1);

    _visibilityRanges[lod] = range;
}

//........................................................................
TextureImageUnitReservation::TextureImageUnitReservation()
{
    _unit = -1;
    _layer = 0L;
}

TextureImageUnitReservation::~TextureImageUnitReservation()
{
    release();
}

void
TextureImageUnitReservation::release()
{
    osg::ref_ptr<TerrainResources> res;
    if (_unit >= 0 && _res.lock(res))
    {
        res->releaseTextureImageUnit(_unit, _layer);
        _unit = -1;
        _layer = 0L;
    }
}
