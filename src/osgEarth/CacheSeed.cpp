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
#include <osgEarth/Mercator>
#include <osgEarth/PlateCarre>

using namespace osgEarth;

void CacheSeed::seed(MapConfig *map)
{
    //Create a TileBuilder for the map
    osg::ref_ptr<TileBuilder> tile_builder = TileBuilder::create( map, map->getFilename() );
    
    //Set the default bounds to the entire profile if the user didn't override the bounds
    if (_bounds._min.x() == 0 && _bounds._min.y() == 0 &&
        _bounds._max.x() == 0 && _bounds._max.y() == 0)
    {
        _bounds._min.x() = tile_builder->getDataProfile().xMin();
        _bounds._min.y() = tile_builder->getDataProfile().yMin();
        _bounds._max.x() = tile_builder->getDataProfile().xMax();
        _bounds._max.y() = tile_builder->getDataProfile().yMax();
    }

    osg::ref_ptr<TileKey> key = tile_builder->getDataProfile().getTileKey( "" );

    bool hasCaches = false;

    //Assumes the the TileSource will perform the caching for us when we call createImage
    for (TileSourceList::iterator itr = tile_builder->getImageSources().begin(); itr != tile_builder->getImageSources().end(); ++itr)
    {
        if (!itr->get()->getCache())
        {
            osg::notify(osg::NOTICE) << "Warning:  Image " << itr->get()->getName() << " has no cache." << std::endl;
        }
        else
        {
            hasCaches = true;
        }
    }

    for (TileSourceList::iterator itr = tile_builder->getHeightFieldSources().begin(); itr != tile_builder->getHeightFieldSources().end(); ++itr)
    {
        if (!itr->get()->getCache())
        {
            osg::notify(osg::NOTICE) << "Warning:  Heightfield " << itr->get()->getName() << " has no cache." << std::endl;
        }
        else
        {
            hasCaches = true;
        }
    }

    if (!hasCaches)
    {
        osg::notify(osg::NOTICE) << "There are no caches specified for the given map.  Please configure a cache in the mapconfig" << std::endl;
        return;
    }

    processKey( tile_builder.get(), key.get() );
}

void CacheSeed::processKey(TileBuilder* tile_builder, TileKey *key)
{
    if (_maxLevel >= key->getLevelOfDetail())
    {
        osg::notify(osg::NOTICE) << "Processing " << key->str() << std::endl;

        if (key->getLevelOfDetail() != 0)
        {
            //Assumes the the TileSource will perform the caching for us when we call createImage
            for (TileSourceList::iterator itr = tile_builder->getImageSources().begin(); itr != tile_builder->getImageSources().end(); ++itr)
            {
                osg::ref_ptr<osg::Image> image = itr->get()->readImage(key);
            }

            for (TileSourceList::iterator itr = tile_builder->getHeightFieldSources().begin(); itr != tile_builder->getHeightFieldSources().end(); ++itr)
            {
                osg::ref_ptr<osg::HeightField> heightField = itr->get()->readHeightField(key);
            }
        }

        osg::ref_ptr<TileKey> k0 = key->getSubkey(0);
        osg::ref_ptr<TileKey> k1 = key->getSubkey(1);
        osg::ref_ptr<TileKey> k2;
        osg::ref_ptr<TileKey> k3;

        if (key->getLevelOfDetail() > 0 || !dynamic_cast<PlateCarreTileKey*>(key))
        {
            k2 = key->getSubkey(2);
            k3 = key->getSubkey(3);
        }

        //Check to see if the bounds intersects ANY of the tile's children.  If it does, then process all of the children
        //for this level
        if ((k0.valid() && _bounds.intersects(k0.get())) ||
            (k1.valid() && _bounds.intersects(k1.get())) ||
            (k2.valid() && _bounds.intersects(k2.get())) ||
            (k3.valid() && _bounds.intersects(k3.get())))
        {
            if (k0.valid()) processKey(tile_builder, k0.get()); 
            if (k1.valid()) processKey(tile_builder, k1.get()); 
            if (k2.valid()) processKey(tile_builder, k2.get()); 
            if (k3.valid()) processKey(tile_builder, k3.get()); 
        }
    }
}