/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
 * MIT License
 */
#include "MapFrame"
#include <osgEarth/Map>

using namespace osgEarth;
using namespace osgEarth::Drivers::MPTerrainEngine;

#define LC "[MapFrame] "

MapFrame::MapFrame() :
_initialized    ( false ),
_highestMinLevel( 0 ),
_mapInfo        ( 0L )
{
    //nop
}

MapFrame::MapFrame(const MapFrame& rhs) :
_initialized         ( rhs._initialized ),
_map                 ( rhs._map.get() ),
_mapInfo             ( rhs._mapInfo ),
_highestMinLevel     ( rhs._highestMinLevel ),
_mapDataModelRevision( rhs._mapDataModelRevision ),
_layers              ( rhs._layers )
{
    //no sync required here; we copied the arrays etc
}

MapFrame::MapFrame(const Map* map) :
_initialized    ( false ),
_map            ( map ),
_mapInfo        ( map ),
_highestMinLevel( 0 )
{
    sync();
}

bool
MapFrame::valid() const
{
    // only a momentary result since this is an observer_ptr!
    return _map.valid();
}

void
MapFrame::setMap(const Map* map)
{
    _layers.clear();

    _map = map;
    if ( map )
    {
        _mapInfo.setMap(map);
    }

    _initialized = false;
    _highestMinLevel = 0;

    if (map)
    {
        sync();
    }
}

osg::ref_ptr<ElevationPool>
MapFrame::getElevationPool() const
{
    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
    {
        return osg::ref_ptr<ElevationPool>(map->getElevationPool());
    }
    else return 0L;
}

bool
MapFrame::sync()
{
    bool changed = false;

    osg::ref_ptr<const Map> map;
    if ( _map.lock(map) )
    {
        if (map->getDataModelRevision() != _mapDataModelRevision)
        {
            _layers.clear();
            map->getLayers(_layers);
            refreshComputedValues();
            _mapDataModelRevision = map->getDataModelRevision();
        }
    }
    else
    {
        _layers.clear();
        _elevationLayers.clear();
        changed = true;
    }    

    return changed;
}


bool
MapFrame::needsSync() const
{
    osg::ref_ptr<const Map> map;
    return 
        _map.lock(map) &&
        (map->getDataModelRevision() != _mapDataModelRevision || !_initialized);
}

void
MapFrame::release()
{
    _layers.clear();
    _elevationLayers.clear();
    _initialized = false;
    _highestMinLevel = 0;
}

bool
MapFrame::containsEnabledLayer(UID uid) const
{
    for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        if (i->get()->getUID() == uid)
        {
            return i->get()->getEnabled();
        }
    }
    return false;
}

void
MapFrame::refreshComputedValues()
{
    _highestMinLevel = 0;

    _elevationLayers.clear();

    for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        TerrainLayer* terrainLayer = dynamic_cast<TerrainLayer*>(i->get());
        if (terrainLayer)
        {
            const optional<unsigned>& minLevel = terrainLayer->options().minLevel();
            if (minLevel.isSet() && minLevel.value() > _highestMinLevel)
            {
                _highestMinLevel = minLevel.value();
            }
            
            ElevationLayer* elevation = dynamic_cast<ElevationLayer*>(terrainLayer);
            if (elevation)
            {
                _elevationLayers.push_back(elevation);
            }
        }
    }
}

bool
MapFrame::populateHeightField(osg::ref_ptr<osg::HeightField>& hf,
                              const TileKey&                  key,
                              bool                            convertToHAE,
                              ProgressCallback*               progress) const
{
    osg::ref_ptr<const Map> map;
    if ( _map.lock(map) )
    {        
        RasterInterpolation interp = map->options().elevationInterpolation().get();

        return _elevationLayers.populateHeightFieldAndNormalMap(
            hf.get(),
            0L,         // no normal map to populate
            key,
            convertToHAE ? map->getProfileNoVDatum() : 0L,
            interp,
            progress );
    }
    else
    {
        return false;
    }
}

bool
MapFrame::populateHeightFieldAndNormalMap(osg::ref_ptr<osg::HeightField>& hf,
                                          osg::ref_ptr<NormalMap>&        normalMap,
                                          const TileKey&                  key,
                                          bool                            convertToHAE,
                                          ProgressCallback*               progress) const
{
    osg::ref_ptr<const Map> map;
    if ( _map.lock(map) )
    {        
        RasterInterpolation interp = map->options().elevationInterpolation().get();

        return _elevationLayers.populateHeightFieldAndNormalMap(
            hf.get(),
            normalMap.get(),
            key,
            convertToHAE ? map->getProfileNoVDatum() : 0L,
            interp,
            progress );
    }
    else
    {
        return false;
    }
}

bool
MapFrame::isCached( const TileKey& key ) const
{
    // is there a map cache at all?
    osg::ref_ptr<const Map> map;
    if (_map.lock(map) && map->getCache() == 0L)
        return false;

    for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        TerrainLayer* layer = dynamic_cast<TerrainLayer*>(i->get());
        if (layer)
        {
            if (!layer->getEnabled())
                continue;

            // If we're cache only we should be fast
            if (layer->getCacheSettings()->cachePolicy()->isCacheOnly())
                continue;

            // no-cache? always slow
            if (layer->getCacheSettings()->cachePolicy()->isCacheDisabled())
                return false;

            //If no data is available on this tile, we'll be fast
            if (!layer->mayHaveData(key))
                continue;

            // No tile source? skip it
            osg::ref_ptr< TileSource > source = layer->getTileSource();
            if (!source.valid())
                continue;

            //If the tile is blacklisted, it should also be fast.
            if (source->getBlacklist()->contains(key))
                continue;

            if (!layer->isCached(key))
                return false;
        }
    }
    return true;
}

osg::ref_ptr<const Map>
MapFrame::lockMap() const
{
    osg::ref_ptr<const Map> map;
    _map.lock(map);
    return map;
}
