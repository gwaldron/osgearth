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
#include <osgEarth/Layer>
#include <limits.h>

using namespace osgEarth;

struct TileLodSortOp
{
    inline bool operator() (const TileKey* lhs,const TileKey* rhs) const
    {
        return (lhs->getLevelOfDetail() < rhs->getLevelOfDetail());
    }
};

void CacheSeed::seed( const MapConfig& conf )
{
    //Create a Map for the map
    //TODO: really, we just need the MapEngine and not the Map..
    osg::ref_ptr<Map> _map = new Map( conf );

    std::vector< osg::ref_ptr<TileKey> > keys;
    _map->getEngine()->getProfile()->getRootKeys(keys);

    //Set the default bounds to the entire profile if the user didn't override the bounds
    if (_bounds._min.x() == 0 && _bounds._min.y() == 0 &&
        _bounds._max.x() == 0 && _bounds._max.y() == 0)
    {
        const GeoExtent& mapEx =  _map->getEngine()->getProfile()->getExtent();

        _bounds._min.x() = mapEx.xMin();
        _bounds._min.y() = mapEx.yMin();
        _bounds._max.x() = mapEx.xMax();
        _bounds._max.y() = mapEx.yMax();
    }


    bool hasCaches = false;
    int src_min_level = INT_MAX;
    int src_max_level = 0;

    ImageLayerList imageLayers;
    _map->getImageLayers( imageLayers );

    //Assumes the the TileSource will perform the caching for us when we call createImage
    for (ImageLayerList::iterator itr = imageLayers.begin(); itr != imageLayers.end(); ++itr)
    {
        TileSource* src = itr->get()->getTileSource();
        if (!dynamic_cast<CachedTileSource*>(src))
        {
            osg::notify(osg::NOTICE) << "Warning:  Image " << src->getName() << " has no cache." << std::endl;
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

    ElevationLayerList elevationLayers;
    _map->getElevationLayers( elevationLayers );

    for (ElevationLayerList::iterator itr = elevationLayers.begin(); itr != elevationLayers.end(); ++itr)
    {
        TileSource* src = itr->get()->getTileSource();
        if (!dynamic_cast<CachedTileSource*>(src))
        {
            osg::notify(osg::NOTICE) << "Warning:  Heightfield " << src->getName() << " has no cache." << std::endl;
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
        osg::notify(osg::NOTICE) << "There are no caches specified for the given map.  Please configure a cache in the mapconfig" << std::endl;
        return;
    }

    if ( src_max_level > 0 && src_max_level < _maxLevel )
    {
        _maxLevel = src_max_level;
    }

    osg::notify(osg::NOTICE) << "Maximum cache level will be " << _maxLevel << std::endl;

    TileKeyList tileKeys;
    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        collectKeys( _map->getEngine(), keys[i].get(), tileKeys);
    }

    //Sort the keys by lod so that they are processed in order
    std::sort(tileKeys.begin(), tileKeys.end(), TileLodSortOp());

    osg::notify(osg::NOTICE) << "Caching " << tileKeys.size() << " keys " << std::endl;

    for (unsigned int i = 0; i < tileKeys.size(); ++i)
    {
        float percentComplete = (float)i / (float)tileKeys.size() * 100.0f;
        osg::notify(osg::NOTICE) << "Caching tile " << i << " of " << tileKeys.size() <<".  " << percentComplete << "% complete" << std::endl;

        TileKey* key = tileKeys[i].get();

        unsigned int x, y, lod;
        key->getTileXY(x, y);
        lod = key->getLevelOfDetail();

        ImageLayerList imageLayers;
        _map->getImageLayers( imageLayers );

        //Assumes the the TileSource will perform the caching for us when we call createImage
        for (ImageLayerList::iterator itr = imageLayers.begin(); itr != imageLayers.end(); ++itr)
        {
            TileSource* source = itr->get()->getTileSource();
            if ( lod >= source->getMinLevel() && lod <= source->getMaxLevel() )
            {
                osg::notify(osg::NOTICE) << "Caching " << source->getName() << ", tile = " << lod << " (" << x << ", " << y << ") " << std::endl;
                osg::ref_ptr<GeoImage> image = _map->getEngine()->createGeoImage(key, source);
            }
        }

        ElevationLayerList elevationLayers;
        _map->getElevationLayers( elevationLayers );

        for (ElevationLayerList::iterator itr = elevationLayers.begin(); itr != elevationLayers.end(); ++itr)
        {
            //TODO:  Handle compatible but non exact heightfield keys (JB)
            TileSource* source = itr->get()->getTileSource();
            if ( lod >= source->getMinLevel() && lod <= source->getMaxLevel() )
            {
                osg::notify(osg::NOTICE) << "Caching " << source->getName() << ", tile = " << lod << " (" << x << ", " << y << ") " << std::endl;
                osg::ref_ptr<osg::HeightField> heightField = source->createHeightField(key);
            }
        }
    }
}


void CacheSeed::collectKeys( MapEngine* map, TileKey* key, TileKeyList& keys)
{
    unsigned int x, y, lod;
    key->getTileXY(x, y);
    lod = key->getLevelOfDetail();

    //    osg::notify(osg::NOTICE) << "Checking key = " << key->str() << std::endl;

    if ( _minLevel <= lod && _maxLevel >= lod )
    {
        keys.push_back(key);
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
            if (k0.valid()) collectKeys(map, k0.get(), keys); 
            if (k1.valid()) collectKeys(map, k1.get(), keys); 
            if (k2.valid()) collectKeys(map, k2.get(), keys); 
            if (k3.valid()) collectKeys(map, k3.get(), keys); 
        }
    }
}