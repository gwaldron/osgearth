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
#include <osgEarth/CacheEstimator>
#include <osgEarth/Registry>
#include <osgEarth/TileKey>

using namespace osgEarth;

CacheEstimator::CacheEstimator():
_minLevel (0),
_maxLevel (12),
_profile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() )
{    
    // By default we can give them a somewhat worse case estimate since it's going to be next to impossible to know what the real size of the data is going to be due to the fact that it's 
    // dependant on the dataset itself as well as compression.  So lets just default to about 130 kb per tile to start with.
    // the goal of the size estimator is just to provide a reality check before you start to do a cache.  If you see that it's going to take 10TB of disk space to 
    // perform a cache you will think twice before starting it.
    _sizeInMBPerTile = 0.13;

    //Rough guess, assume it takes 1/10 of a second to process a tile.
    _timeInSecondsPerTile = 0.1;
}

void
CacheEstimator::addExtent( const GeoExtent& value)
{
    _extents.push_back( value );
}

unsigned int
CacheEstimator::getNumTiles() const
{
    unsigned int total = 0;

    for (unsigned int level = _minLevel; level <= _maxLevel; level++)
    {
        double coverageRatio = 0.0;

        if (_extents.empty())
        {
            unsigned int wide, high;
            _profile->getNumTiles( level, wide, high );
            total += (wide * high);
        }
        else
        {
            for (std::vector< GeoExtent >::const_iterator itr = _extents.begin(); itr != _extents.end(); itr++)
            {
                const GeoExtent& extent = *itr;
                double boundsArea = extent.area();

                TileKey ll = _profile->createTileKey(extent.xMin(), extent.yMin(), level);
                TileKey ur = _profile->createTileKey(extent.xMax(), extent.yMax(), level);

                if (!ll.valid() || !ur.valid()) continue;
                
                int tilesWide = ur.getTileX() - ll.getTileX() + 1;
                int tilesHigh = ll.getTileY() - ur.getTileY() + 1;
                int tilesAtLevel = tilesWide * tilesHigh;                
                total += tilesAtLevel;
            }
        }
    }
    return total;
}

double CacheEstimator::getSizeInMB() const
{
    return getNumTiles() * _sizeInMBPerTile;
}

double CacheEstimator::getTotalTimeInSeconds() const
{
    return getNumTiles() * _timeInSecondsPerTile;
}



