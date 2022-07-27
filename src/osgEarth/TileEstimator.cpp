/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/TileEstimator>
#include <osgEarth/Registry>
#include <osgEarth/TileKey>

using namespace osgEarth;

TileEstimator::TileEstimator():
_minLevel (0),
_maxLevel (12),
_profile( Profile::create(Profile::GLOBAL_GEODETIC))
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
TileEstimator::addExtent( const GeoExtent& value)
{
    _extents.push_back( value );
}

unsigned int
TileEstimator::getNumTiles() const
{
    unsigned int total = 0;

    GeoExtent extentsUnion;
    for (std::vector< GeoExtent >::const_iterator itr = _extents.begin(); itr != _extents.end(); ++itr)
        extentsUnion.expandToInclude(*itr);

    for (unsigned int level = _minLevel; level <= _maxLevel; level++)
    {
        if (_extents.empty())
        {
            unsigned int wide, high;
            _profile->getNumTiles( level, wide, high );
            total += (wide * high);
        }
        else
        {
            constexpr double e = 0.001;
            TileKey ll = _profile->createTileKey(extentsUnion.xMin()+e, extentsUnion.yMin()+e, level);
            TileKey ur = _profile->createTileKey(extentsUnion.xMax()-e, extentsUnion.yMax()-e, level);

            if (!ll.valid() || !ur.valid()) continue;
                
            int tilesWide = ur.getTileX() - ll.getTileX() + 1;
            int tilesHigh = ll.getTileY() - ur.getTileY() + 1;
            int tilesAtLevel = tilesWide * tilesHigh;                
            total += tilesAtLevel;
        }
    }
    return total;
}

double TileEstimator::getSizeInMB() const
{
    return getNumTiles() * _sizeInMBPerTile;
}

double TileEstimator::getTotalTimeInSeconds() const
{
    return getNumTiles() * _timeInSecondsPerTile;
}



