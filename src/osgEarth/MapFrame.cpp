/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

using namespace osgEarth;

#define LC "[MapFrame] "


MapFrame::MapFrame( const Map* map, Map::ModelParts parts, const std::string& name ) :
_initialized( false ),
_map        ( map ),
_name       ( name ),
_mapInfo    ( map ),
_parts      ( parts )
{
    sync();
}


MapFrame::MapFrame( const MapFrame& src, const std::string& name ) :
_initialized         ( src._initialized ),
_map                 ( src._map.get() ),
_name                ( name ),
_mapInfo             ( src._mapInfo ),
_parts               ( src._parts ),
_mapDataModelRevision( src._mapDataModelRevision ),
_imageLayers         ( src._imageLayers ),
_elevationLayers     ( src._elevationLayers ),
_modelLayers         ( src._modelLayers ),
_maskLayers          ( src._maskLayers )
{
    //no sync required here; we copied the arrays etc
}


bool
MapFrame::sync()
{
    bool changed = false;

    if ( _map.valid() )
    {
        changed = _map->sync( *this );        
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
    return
        (_map.valid()) &&
        (_map->getDataModelRevision() != _mapDataModelRevision || !_initialized);
}


bool
MapFrame::getHeightField(const TileKey&                  key,
                         bool                            fallback,
                         osg::ref_ptr<osg::HeightField>& out_hf,
                         bool*                           out_isFallback,    
                         bool                            convertToHAE,
                         ElevationSamplePolicy           samplePolicy,
                         ProgressCallback*               progress) const
{
    if ( !_map.valid() ) 
        return false;
    


    return _elevationLayers.createHeightField(
        key,
        fallback, 
        convertToHAE ? _map->getProfileNoVDatum() : 0L,
        _mapInfo.getElevationInterpolation(), 
        samplePolicy, 
        out_hf, 
        out_isFallback,
        progress );    
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
    //Check to see if the tile will load fast
    // Check the imagery layers
    for( ImageLayerVector::const_iterator i = imageLayers().begin(); i != imageLayers().end(); i++ )
    {   
        //If we're cache only we should be fast
        if (i->get()->isCacheOnly()) continue;

        osg::ref_ptr< TileSource > source = i->get()->getTileSource();
        if (!source.valid()) continue;

        //If the tile is blacklisted, it should also be fast.
        if ( source->getBlacklist()->contains( key.getTileId() ) ) continue;
        //If no data is available on this tile, we'll be fast
        if ( !source->hasData( key ) ) continue;

        if ( !i->get()->isCached( key ) ) return false;
    }

    for( ElevationLayerVector::const_iterator i = elevationLayers().begin(); i != elevationLayers().end(); ++i )
    {
        //If we're cache only we should be fast
        if (i->get()->isCacheOnly()) continue;

        osg::ref_ptr< TileSource > source = i->get()->getTileSource();
        if (!source.valid()) continue;

        //If the tile is blacklisted, it should also be fast.
        if ( source->getBlacklist()->contains( key.getTileId() ) ) continue;
        if ( !source->hasData( key ) ) continue;
        if ( !i->get()->isCached( key ) )
        {
            return false;
        }
    }

    return true;
}
