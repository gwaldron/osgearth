/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
    ScopedReadLock lock( map->getMapDataMutex() );

    osg::ref_ptr<MapEngine> engine = new MapEngine(); //map->createMapEngine();

    std::vector< osg::ref_ptr<TileKey> > keys;
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

        if ( !src->supportsPersistentCaching() )
        {
            osg::notify(osg::WARN) << "Warning: Layer \"" << layer->getName() << "\" does not support seeding." << std::endl;
        }
        else if ( !layer->getCache() )
        {
            osg::notify(osg::NOTICE) << "Notice: Layer \"" << layer->getName() << "\" has no persistent cache defined; skipping." << std::endl;
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

        if ( !src->supportsPersistentCaching() )
        {
            osg::notify(osg::WARN) << "Warning: Layer \"" << layer->getName() << "\" does not support seeding." << std::endl;
        }
        else if ( !layer->getCache() )
        {
            osg::notify(osg::NOTICE) << "Notice: Layer \"" << src->getName() << "\" has no persistent cache defined; skipping." << std::endl;
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
        osg::notify(osg::NOTICE) << "There are either no caches defined in the map, or no sources to cache. Exiting." << std::endl;
        return;
    }

    if ( src_max_level > 0 && src_max_level < _maxLevel )
    {
        _maxLevel = src_max_level;
    }

    osg::notify(osg::NOTICE) << "Maximum cache level will be " << _maxLevel << std::endl;

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        processKey( map, engine.get(), keys[i].get() );
    }
}


void CacheSeed::processKey( Map* map, MapEngine* engine, TileKey* key )
{
    unsigned int x, y, lod;
    key->getTileXY(x, y);
    lod = key->getLevelOfDetail();

	osg::ref_ptr<osgEarth::VersionedTerrain> terrain = new osgEarth::VersionedTerrain( map, engine );

    if ( _minLevel <= lod && _maxLevel >= lod )
    {
        osg::notify(osg::NOTICE) << "Caching tile = " << key->str() << std::endl; //<< lod << " (" << x << ", " << y << ") " << std::endl;
        bool validData;
		osg::ref_ptr<osg::Node> node = engine->createTile( map, terrain.get(), key, true, false, validData );        
    }

    if (key->getLevelOfDetail() <= _maxLevel)
    {
        osg::ref_ptr<TileKey> k0 = key->createSubkey(0);
        osg::ref_ptr<TileKey> k1 = key->createSubkey(1);
        osg::ref_ptr<TileKey> k2 = key->createSubkey(2);
        osg::ref_ptr<TileKey> k3 = key->createSubkey(3);        

        //Check to see if the bounds intersects ANY of the tile's children.  If it does, then process all of the children
        //for this level
        if (_bounds.intersects( k0->getGeoExtent().bounds() ) || _bounds.intersects(k1->getGeoExtent().bounds()) ||
            _bounds.intersects( k2->getGeoExtent().bounds() ) || _bounds.intersects(k3->getGeoExtent().bounds()) )
        {
            processKey(map, engine, k0.get()); 
            processKey(map, engine, k1.get()); 
            processKey(map, engine, k2.get()); 
            processKey(map, engine, k3.get()); 
        }
    }
}
