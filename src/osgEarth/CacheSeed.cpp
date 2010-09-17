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
    Threading::ScopedReadLock lock( map->getMapDataMutex() );

    if (!map->getCache())
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
    int src_max_level = 0;

    //Assumes the the TileSource will perform the caching for us when we call createImage
    for( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
		MapLayer* layer = i->get();
        TileSource* src = i->get()->getTileSource();

        if (layer->cacheOnly().get())
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

			if (layer->minLevel().isSet() && layer->minLevel().get() < src_min_level)
                src_min_level = layer->minLevel().get();
			if (layer->maxLevel().isSet() && layer->maxLevel().get() > src_max_level)
                src_max_level = layer->maxLevel().get();
        }
    }

    for( MapLayerList::const_iterator i = map->getHeightFieldMapLayers().begin(); i != map->getHeightFieldMapLayers().end(); i++ )
    {
		MapLayer* layer = i->get();
        TileSource* src = i->get()->getTileSource();

        if (layer->cacheOnly().get())
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

			if (layer->minLevel().isSet() && layer->minLevel().get() < src_min_level)
                src_min_level = layer->minLevel().get();
			if (layer->maxLevel().isSet() && layer->maxLevel().get() > src_max_level)
                src_max_level = layer->maxLevel().get();
		}
    }

    if (!hasCaches)
    {
        OE_NOTICE << "There are either no caches defined in the map, or no sources to cache. Exiting." << std::endl;
        return;
    }

    if ( src_max_level > 0 && src_max_level < _maxLevel )
    {
        _maxLevel = src_max_level;
    }

    OE_NOTICE << "Maximum cache level will be " << _maxLevel << std::endl;

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        processKey( map, keys[i] );
    }
}


void CacheSeed::processKey( Map* map, const TileKey& key ) const
{
    unsigned int x, y, lod;
    key.getTileXY(x, y);
    lod = key.getLevelOfDetail();

//	osg::ref_ptr<osgEarth::VersionedTerrain> terrain = new osgEarth::VersionedTerrain( map, engine );

    if ( _minLevel <= lod && _maxLevel >= lod )
    {
        OE_NOTICE << "Caching tile = " << key.str() << std::endl; //<< lod << " (" << x << ", " << y << ") " << std::endl;
        cacheTile( map, key );
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
        if (_bounds.intersects( k0.getGeoExtent().bounds() ) || _bounds.intersects(k1.getGeoExtent().bounds()) ||
            _bounds.intersects( k2.getGeoExtent().bounds() ) || _bounds.intersects(k3.getGeoExtent().bounds()) )
        {
            processKey(map, k0); 
            processKey(map, k1); 
            processKey(map, k2); 
            processKey(map, k3); 
        }
    }
}

void
CacheSeed::cacheTile( Map* map, const TileKey& key ) const
{
    for( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
        MapLayer* layer = i->get();
        if ( layer->isKeyValid( key ) )
        {
            GeoImage image = layer->createImage( key );
        }
    }

    if ( map->getHeightFieldMapLayers().size() > 0 )
    {
        osg::ref_ptr<osg::HeightField> hf = map->createHeightField( key, false );
    }
}

