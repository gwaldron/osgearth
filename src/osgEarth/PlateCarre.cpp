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

#include <osgEarth/PlateCarre>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <sstream>

using namespace osgEarth;

const std::string PlateCarreTileKey::TYPE_CODE = "P";

PlateCarreTileKey::PlateCarreTileKey( const PlateCarreTileKey& rhs )
: TileKey( rhs )
{
    //NOP
}

PlateCarreTileKey::PlateCarreTileKey( const std::string& input )
: TileKey( input, TileGridProfile(TileGridProfile::GLOBAL_GEODETIC ) )
{
    //NOP
}

PlateCarreTileKey::PlateCarreTileKey( const std::string& input, const TileGridProfile& profile )
: TileKey( input, profile )
{
    //NOP
}

TileKey*
PlateCarreTileKey::getSubkey( unsigned int quadrant ) const
{
    if ( !subkeys[quadrant].valid() )
        const_cast<PlateCarreTileKey*>(this)->subkeys[quadrant] = new PlateCarreTileKey( key + (char)('0' + quadrant), profile );
    return subkeys[quadrant].get();
}

TileKey*
PlateCarreTileKey::createParentKey() const
{
    if (getLevelOfDetail() == 1) return NULL;
    return new PlateCarreTileKey(key.substr(0, key.length()-1));
}

bool
PlateCarreTileKey::getGeoExtents(double& out_min_lon,
                                 double& out_min_lat,
                                 double& out_max_lon,
                                 double& out_max_lat ) const
{
    double width =  profile.xMax() - profile.xMin();
    double height = profile.yMax() - profile.yMin();

    out_max_lat = profile.yMax();
    out_min_lon = profile.xMin();

    for( unsigned int lod = 0; lod < getLevelOfDetail(); lod++ )
    {
        width /= 2.0;
        height /= 2.0;

        char digit = key[lod];
        switch( digit )
        {
            case '1': out_min_lon += width; break;
            case '2': out_max_lat -= height; break;
            case '3': out_min_lon += width; out_max_lat -= height; break;
        }
    }

    out_min_lat = out_max_lat - height;
    out_max_lon = out_min_lon + width;
    return true;
}