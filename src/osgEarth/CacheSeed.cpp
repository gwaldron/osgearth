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
#include <osgEarth/Mercator>
#include <OpenThreads/ScopedLock>
#include <limits.h>

using namespace osgEarth;
using namespace OpenThreads;

void CacheSeed::seed( Map* map )
{
    ScopedReadLock lock( map->getMapDataMutex() );

    osg::ref_ptr<MapEngine> engine = map->createMapEngine();

    std::vector< osg::ref_ptr<TileKey> > keys;
    map->getProfile()->getRootKeys(keys);

    //Set the default bounds to the entire profile if the user didn't override the bounds
    if (_bounds._min.x() == 0 && _bounds._min.y() == 0 &&
        _bounds._max.x() == 0 && _bounds._max.y() == 0)
    {
        const GeoExtent& mapEx =  map->getProfile()->getExtent();

        _bounds._min.x() = mapEx.xMin();
        _bounds._min.y() = mapEx.yMin();
        _bounds._max.x() = mapEx.xMax();
        _bounds._max.y() = mapEx.yMax();    }


    bool hasCaches = false;
    int src_min_level = INT_MAX;
    int src_max_level = 0;

    //Assumes the the TileSource will perform the caching for us when we call createImage
    for( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
        TileSource* src = i->get()->getTileSource();

        if ( !src->supportsPersistentCaching() )
        {
            osg::notify(osg::WARN) << "Warning: source \"" << src->getName() << "\" does not support seeding." << std::endl;
        }
        else if ( !src->hasPersistentCache() )
        {
            osg::notify(osg::NOTICE) << "Notice: source \"" << src->getName() << "\" has no persistent cache defined; skipping." << std::endl;
        }
        else
        {
            hasCaches = true;

            if ( src->getMinLevel() < src_min_level )
                src_min_level = src->getMinLevel();
            if ( src->getMaxLevel() > src_max_level )
                src_max_level = src->getMaxLevel();
        }
    }

    for( MapLayerList::const_iterator i = map->getHeightFieldMapLayers().begin(); i != map->getHeightFieldMapLayers().end(); i++ )
    {
        TileSource* src = i->get()->getTileSource();

        if ( !src->supportsPersistentCaching() )
        {
            osg::notify(osg::WARN) << "Warning: source \"" << src->getName() << "\" does not support seeding." << std::endl;
        }
        else if ( !src->hasPersistentCache() )
        {
            osg::notify(osg::NOTICE) << "Notice: source \"" << src->getName() << "\" has no persistent cache defined; skipping." << std::endl;
        }
        else
        {
            hasCaches = true;

            if ( src->getMinLevel() < src_min_level )
                src_min_level = src->getMinLevel();
            if ( src->getMaxLevel() > src_max_level )
                src_max_level = src->getMaxLevel();
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

    if ( _minLevel <= lod && _maxLevel >= lod )
    {
        osg::notify(osg::NOTICE) << "Caching tile = " << key->str() << std::endl; //<< lod << " (" << x << ", " << y << ") " << std::endl;
        engine->createQuadrant( map, 0, key );        
    }

    if (key->getLevelOfDetail() <= _maxLevel)
    {
        osg::ref_ptr<TileKey> k0 = key->getSubkey(0);
        osg::ref_ptr<TileKey> k1 = key->getSubkey(1);
        osg::ref_ptr<TileKey> k2 = key->getSubkey(2);
        osg::ref_ptr<TileKey> k3 = key->getSubkey(3);        

        //Check to see if the bounds intersects ANY of the tile's children.  If it does, then process all of the children
        //for this level
        if ((k0.valid() && _bounds.intersects(k0.get())) ||
            (k1.valid() && _bounds.intersects(k1.get())) ||
            (k2.valid() && _bounds.intersects(k2.get())) ||
            (k3.valid() && _bounds.intersects(k3.get())))
        {
            if (k0.valid()) processKey(map, engine, k0.get()); 
            if (k1.valid()) processKey(map, engine, k1.get()); 
            if (k2.valid()) processKey(map, engine, k2.get()); 
            if (k3.valid()) processKey(map, engine, k3.get()); 
        }
    }
}
