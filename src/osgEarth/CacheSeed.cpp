/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

#include <osgEarth/CacheSeed>
#include <OpenThreads/ScopedLock>
#include <limits.h>

#define LC "[CacheSeed] "

using namespace osgEarth;
using namespace OpenThreads;

void CacheSeed::seed( Map* map )
{
    if ( !map->getCache() )
    {
        OE_WARN << LC << "Warning: No cache defined; aborting." << std::endl;
        return;
    }

    std::vector<TileKey> keys;
    map->getProfile()->getRootKeys(keys);

    //Set the default bounds to the entire profile if the user didn't override the bounds
    if (_bounds.xMin() == 0 && _bounds.yMin() == 0 &&
        _bounds.xMax() == 0 && _bounds.yMax() == 0)
    {
        const GeoExtent& mapEx =  map->getProfile()->getExtent();
        _bounds = Bounds( mapEx.xMin(), mapEx.yMin(), mapEx.xMax(), mapEx.yMax() );
    }


    bool hasCaches = false;
    int src_min_level = INT_MAX;
    unsigned int src_max_level = 0;

    MapFrame mapf( map, Map::TERRAIN_LAYERS, "CacheSeed::seed" );

    //Assumes the the TileSource will perform the caching for us when we call createImage
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); i++ )
    {
		ImageLayer* layer = i->get();
        TileSource* src   = layer->getTileSource();

        const ImageLayerOptions& opt = layer->getImageLayerOptions();

        if ( layer->isCacheOnly() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" is set to cache-only; skipping." << std::endl;
        }
        else if (!src)
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" could not create TileSource; skipping." << std::endl;
        }
        else if ( !src->supportsPersistentCaching() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" does not support seeding; skipping." << std::endl;
        }
        else if ( !layer->getCache() )
        {
            OE_WARN << LC << "Notice: Layer \"" << layer->getName() << "\" has no cache defined; skipping." << std::endl;
        }
        else
        {
            hasCaches = true;

			if (opt.minLevel().isSet() && opt.minLevel().get() < src_min_level)
                src_min_level = opt.minLevel().get();
			if (opt.maxLevel().isSet() && opt.maxLevel().get() > (int)src_max_level)
                src_max_level = opt.maxLevel().get();
        }
    }

    for( ElevationLayerVector::const_iterator i = mapf.elevationLayers().begin(); i != mapf.elevationLayers().end(); i++ )
    {
		ElevationLayer* layer = i->get();
        TileSource*     src   = layer->getTileSource();
        const ElevationLayerOptions& opt = layer->getElevationLayerOptions();

        if ( layer->isCacheOnly() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" is set to cache-only; skipping." << std::endl;
        }
        else if (!src)
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" could not create TileSource; skipping." << std::endl;
        }
        else if ( !src->supportsPersistentCaching() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" does not support seeding; skipping." << std::endl;
        }
        else if ( !layer->getCache() )
        {
            OE_WARN << LC << "Notice: Layer \"" << layer->getName() << "\" has no cache defined; skipping." << std::endl;
        }
        else
        {
            hasCaches = true;

			if (opt.minLevel().isSet() && opt.minLevel().get() < src_min_level)
                src_min_level = opt.minLevel().get();
			if (opt.maxLevel().isSet() && opt.maxLevel().get() > (int)src_max_level)
                src_max_level = opt.maxLevel().get();
		}
    }

    if ( !hasCaches )
    {
        OE_WARN << LC << "There are either no caches defined in the map, or no sources to cache; aborting." << std::endl;
        return;
    }

    if ( src_max_level > 0 && (unsigned)src_max_level < _maxLevel )
    {
        _maxLevel = src_max_level;
    }

    OE_NOTICE << LC << "Maximum cache level will be " << _maxLevel << std::endl;

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        processKey( mapf, keys[i] );
    }
}


void
CacheSeed::processKey(const MapFrame& mapf, const TileKey& key ) const
{
    unsigned int x, y, lod;
    key.getTileXY(x, y);
    lod = key.getLevelOfDetail();

    bool gotData = true;

    if ( _minLevel <= lod && _maxLevel >= lod )
    {
        gotData = cacheTile( mapf, key );

    	if ( _progress.valid() && _progress->isCanceled() )
	        return; // Task has been cancelled by user

        if ( _progress.valid() && gotData && _progress->reportProgress(0, 0, "Cached tile: " + key.str()) )
            return; // Canceled
    }

    if ( gotData && lod <= _maxLevel )
    {
        TileKey k0 = key.createChildKey(0);
        TileKey k1 = key.createChildKey(1);
        TileKey k2 = key.createChildKey(2);
        TileKey k3 = key.createChildKey(3);        

        //Check to see if the bounds intersects ANY of the tile's children.  If it does, then process all of the children
        //for this level
        if (_bounds.intersects( k0.getExtent().bounds() ) || _bounds.intersects(k1.getExtent().bounds()) ||
            _bounds.intersects( k2.getExtent().bounds() ) || _bounds.intersects(k3.getExtent().bounds()) )
        {
            processKey(mapf, k0);
            processKey(mapf, k1);
            processKey(mapf, k2);
            processKey(mapf, k3);
        }
    }
}

bool
CacheSeed::cacheTile(const MapFrame& mapf, const TileKey& key ) const
{
    bool gotData = false;

    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); i++ )
    {
        ImageLayer* layer = i->get();
        if ( layer->isKeyValid( key ) )
        {
            GeoImage image = layer->createImage( key );
            if ( image.valid() )
                gotData = true;
        }
    }

    if ( mapf.elevationLayers().size() > 0 )
    {
        osg::ref_ptr<osg::HeightField> hf;
        mapf.getHeightField( key, false, hf );
        if ( hf.valid() )
            gotData = true;
    }

    return gotData;
}
