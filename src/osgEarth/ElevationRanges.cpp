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
#include <osgEarth/ElevationRanges>
#include <osgEarth/Registry>

using namespace osgEarth;

// Generated using code at https://gist.github.com/jasonbeverage/9fa0388163b44e47d67a97cc62b68bc8
#include "elevation_ranges.cpp"

unsigned int ElevationRanges::getMaxLevel()
{
    return s_maxLevel;
}

osg::ref_ptr<const Profile> ElevationRanges::getProfile()
{
    return Profile::create(Profile::GLOBAL_GEODETIC);
}

bool ElevationRanges::getDefaultElevationRange(short& min, short& max)
{
    min = -2000;
    max = 9000;
    return true;
}

bool ElevationRanges::getElevationRange(unsigned int level, unsigned int x, unsigned int y, short& min, short& max)
{
    osg::ref_ptr< const Profile > profile = getProfile();

    unsigned int width;
    unsigned int height;
    profile->getNumTiles(level, width, height);

    if (level < 0 || level > s_maxLevel ||
        x < 0 || x >= width ||
        y < 0 || y >= height)
    {
        OE_WARN << "Requested tile coordinate " << level << " (" << x << ", " << y << ") are outside of valid range" << std::endl;
        return false;
    }

    unsigned int index = y * width + x;    
    min = s_minElevationsLOD[level][index];
    max = s_maxElevationsLOD[level][index];
    return true;
}