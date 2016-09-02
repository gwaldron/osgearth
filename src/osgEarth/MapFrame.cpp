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
#include <osgEarth/MapFrame>
#include <osgEarth/Cache>

using namespace osgEarth;

#define LC "[MapFrame] "

MapFrame::MapFrame() :
_initialized    ( false ),
_highestMinLevel( 0 ),
_mapInfo       ( 0L ),
_parts(Map::ENTIRE_MODEL)
{
    //nop
}

MapFrame::MapFrame(const MapFrame& rhs) :
_initialized         ( rhs._initialized ),
_map                 ( rhs._map.get() ),
_mapInfo             ( rhs._mapInfo ),
_parts               ( rhs._parts ),
_highestMinLevel     ( rhs._highestMinLevel ),
_mapDataModelRevision( rhs._mapDataModelRevision ),
_imageLayers         ( rhs._imageLayers ),
_elevationLayers     ( rhs._elevationLayers ),
_modelLayers         ( rhs._modelLayers ),
_maskLayers          ( rhs._maskLayers )
{
    //no sync required here; we copied the arrays etc
}

MapFrame::MapFrame(const Map* map) :
_initialized    ( false ),
_map            ( map ),
_mapInfo        ( map ),
_parts          ( Map::ENTIRE_MODEL ),
_highestMinLevel( 0 )
{
    sync();
}

MapFrame::MapFrame(const Map* map, Map::ModelParts parts) :
_initialized    ( false ),
_map            ( map ),
_mapInfo        ( map ),
_parts          ( parts ),
_highestMinLevel( 0 )
{
    sync();
}

bool
MapFrame::isValid() const
{
    return _map.valid();
}

void
MapFrame::setMap(const Map* map)
{
    _imageLayers.clear();
    _elevationLayers.clear();
    _modelLayers.clear();
    _maskLayers.clear();

    _map = map;
    if ( map )
        _mapInfo = MapInfo(map);

    _initialized = false;
    _highestMinLevel = 0;

    sync();
}

bool
MapFrame::sync()
{
    bool changed = false;

    osg::ref_ptr<const Map> map;
    if ( _map.lock(map) )
    {
        changed = _map->sync( *this );
        if ( changed )
        {
            refreshComputedValues();
        }
    }
    else
    {
        _imageLayers.clear();
        _elevationLayers.clear();
        _modelLayers.clear();
        _maskLayers.clear();
    }

    return changed;
}


bool
MapFrame::needsSync() const
{
    if ( !isValid() )
        return false;

    osg::ref_ptr<const Map> map;
    return 
        _map.lock(map) &&
        (map->getDataModelRevision() != _mapDataModelRevision || !_initialized);
}

UID
MapFrame::getUID() const
{
    osg::ref_ptr<const Map> map;
    if ( _map.lock(map) )
        return map->getUID();
    else
        return (UID)0;
}

void
MapFrame::refreshComputedValues()
{
    // cache the min LOD based on all image/elev layers
    _highestMinLevel = 0;

    for(ImageLayerVector::const_iterator i = _imageLayers.begin(); 
        i != _imageLayers.end();
        ++i)
    {
        const optional<unsigned>& minLevel = i->get()->getTerrainLayerRuntimeOptions().minLevel();
        if ( minLevel.isSet() && minLevel.value() > _highestMinLevel )
            _highestMinLevel = minLevel.value();
    }

    for(ElevationLayerVector::const_iterator i = _elevationLayers.begin(); 
        i != _elevationLayers.end();
        ++i)
    {
        const optional<unsigned>& minLevel = i->get()->getTerrainLayerRuntimeOptions().minLevel();
        if ( minLevel.isSet() && minLevel.value() > _highestMinLevel )
            _highestMinLevel = minLevel.value();
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
        ElevationInterpolation interp = map->getMapOptions().elevationInterpolation().get();    
        return _elevationLayers.populateHeightField(
            hf.get(),
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


int
MapFrame::indexOf( ImageLayer* layer ) const
{
    ImageLayerVector::const_iterator i = std::find( _imageLayers.begin(), _imageLayers.end(), layer );
    return i != _imageLayers.end() ? i - _imageLayers.begin() : -1;
}


int
MapFrame::indexOf( ElevationLayer* layer ) const
{
    ElevationLayerVector::const_iterator i = std::find( _elevationLayers.begin(), _elevationLayers.end(), layer );
    return i != _elevationLayers.end() ? i - _elevationLayers.begin() : -1;
}


int
MapFrame::indexOf( ModelLayer* layer ) const
{
    ModelLayerVector::const_iterator i = std::find( _modelLayers.begin(), _modelLayers.end(), layer );
    return i != _modelLayers.end() ? i - _modelLayers.begin() : -1;
}


ImageLayer*
MapFrame::getImageLayerByUID( UID uid ) const
{
    for(ImageLayerVector::const_iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i )
        if ( i->get()->getUID() == uid )
            return i->get();
    return 0L;
}


ImageLayer*
MapFrame::getImageLayerByName( const std::string& name ) const
{
    for(ImageLayerVector::const_iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i )
        if ( i->get()->getName() == name )
            return i->get();
    return 0L;
}


bool
MapFrame::isCached( const TileKey& key ) const
{
    // is there a map cache at all?
    if ( _map.valid() && _map->getCache() == 0L )
        return false;

    //Check to see if the tile will load fast
    // Check the imagery layers
    for( ImageLayerVector::const_iterator i = imageLayers().begin(); i != imageLayers().end(); i++ )
    {   
        const ImageLayer* layer = i->get();

        if (!layer->getEnabled())
            continue;

        // If we're cache only we should be fast
        if (layer->getCacheSettings()->cachePolicy()->isCacheOnly())
            continue;

        // no-cache? always slow
        if (layer->getCacheSettings()->cachePolicy()->isCacheDisabled())
            return false;

        // No tile source? skip it
        osg::ref_ptr< TileSource > source = layer->getTileSource();
        if (!source.valid())
            continue;

        //If the tile is blacklisted, it should also be fast.
        if ( source->getBlacklist()->contains( key ) )
            continue;

        //If no data is available on this tile, we'll be fast
        if ( !source->hasData( key ) )
            continue;

        if ( !layer->isCached(key) )
            return false;
    }

    for( ElevationLayerVector::const_iterator i = elevationLayers().begin(); i != elevationLayers().end(); ++i )
    {
        const ElevationLayer* layer = i->get();

        if (!layer->getEnabled())
            continue;

        //If we're cache only we should be fast
        if (layer->getCacheSettings()->cachePolicy()->isCacheOnly())
            continue;

        // no-cache? always high-latency.
        if (layer->getCacheSettings()->cachePolicy()->isCacheDisabled())
            return false;

        osg::ref_ptr< TileSource > source = layer->getTileSource();
        if (!source.valid())
            continue;

        //If the tile is blacklisted, it should also be fast.
        if ( source->getBlacklist()->contains( key ) )
            continue;

        if ( !source->hasData( key ) )
            continue;

        if ( !i->get()->isCached( key ) )
            return false;
    }

    return true;
}

const MapOptions&
MapFrame::getMapOptions() const
{
    return _map->getMapOptions();
}
