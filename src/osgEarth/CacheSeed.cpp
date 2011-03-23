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
#include <osgEarth/Caching>
#include <OpenThreads/ScopedLock>
#include <limits.h>

using namespace osgEarth;
using namespace OpenThreads;

void CacheSeed::seed( Map* map )
{
    //Threading::ScopedReadLock lock( map->getMapDataMutex() );

    if ( !map->getMapOptions().cache().isSet() )
    //if (!map->getCache())
    {
        OE_WARN << "Warning:  Map does not have a cache defined, please define a cache." << std::endl;
        return;
    }

//    osg::ref_ptr<MapEngine> engine = new MapEngine(); //map->createMapEngine();

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
        TileSource* src = i->get()->getTileSource();
        const ImageLayerOptions& opt = layer->getImageLayerOptions();

        if ( opt.cacheOnly() == true )
        {
            OE_WARN << "Warning:  Cannot seed b/c Layer \"" << layer->getName() << "\" is cache only." << std::endl;
            return;
        }
        else if (!src)
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" could not create TileSource." << std::endl;
        }
        else if ( !src->supportsPersistentCaching() )
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" does not support seeding." << std::endl;
        }
        else if ( !layer->getCache() )
        {
            OE_NOTICE << "Notice: Layer \"" << layer->getName() << "\" has no persistent cache defined; skipping." << std::endl;
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
        TileSource* src = i->get()->getTileSource();
        const ElevationLayerOptions& opt = layer->getElevationLayerOptions();

        if ( opt.cacheOnly().get())
        {
            OE_WARN << "Warning:  Cannot seed b/c Layer \"" << layer->getName() << "\" is cache only." << std::endl;
            return;
        }
        else if (!src)
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" could not create TileSource." << std::endl;
        }
        else if ( !src->supportsPersistentCaching() )
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" does not support seeding." << std::endl;
        }
        else if ( !layer->getCache() )
        {
            OE_NOTICE << "Notice: Layer \"" << src->getName() << "\" has no persistent cache defined; skipping." << std::endl;
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

    if (!hasCaches)
    {
        OE_NOTICE << "There are either no caches defined in the map, or no sources to cache. Exiting." << std::endl;
        return;
    }

    if ( src_max_level > 0 && (unsigned)src_max_level < _maxLevel )
    {
        _maxLevel = src_max_level;
    }

    OE_NOTICE << "Maximum cache level will be " << _maxLevel << std::endl;

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

//	osg::ref_ptr<osgEarth::VersionedTerrain> terrain = new osgEarth::VersionedTerrain( map, engine );

    if ( _minLevel <= lod && _maxLevel >= lod )
    {
//        OE_NOTICE << "Caching tile = " << key.str() << std::endl; //<< lod << " (" << x << ", " << y << ") " << std::endl;
	if ( _progress.valid() && _progress->reportProgress(0, 0, "Caching tile: " + key.str()) )
	    return; // Task has been cancelled by user

        cacheTile( mapf, key );
  //      bool validData;
		//osg::ref_ptr<osg::Node> node = engine->createTile( map, terrain.get(), key, true, false, false, validData );        
    }

    if (lod <= _maxLevel)
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

void
CacheSeed::cacheTile(const MapFrame& mapf, const TileKey& key ) const
{
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); i++ )
    {
        ImageLayer* layer = i->get();
        if ( layer->isKeyValid( key ) )
        {
            GeoImage image = layer->createImage( key );
        }
    }

    if ( mapf.elevationLayers().size() > 0 )
    {
        osg::ref_ptr<osg::HeightField> hf;
        mapf.getHeightField( key, false, hf );
    }
}
