/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/MapInfo>
#include <osgEarth/Map>

using namespace osgEarth;

#define LC "[MapInfo] "


MapInfo::MapInfo( const Map* map ) :
_profile               ( 0L ),
_isGeocentric          ( true ),
_isCube                ( false ),
_elevationInterpolation( INTERP_BILINEAR )
{ 
    if ( map )
    {
        _profile = map->getProfile();
        _isGeocentric = map->isGeocentric();
        _isCube = map->getMapOptions().coordSysType() == MapOptions::CSTYPE_GEOCENTRIC_CUBE;
        _elevationInterpolation = *map->getMapOptions().elevationInterpolation();
    }
}

MapInfo::MapInfo( const MapInfo& rhs ) :
_profile               ( rhs._profile ),
_isGeocentric          ( rhs._isGeocentric ),
_isCube                ( rhs._isCube ),
_elevationInterpolation( rhs._elevationInterpolation )
{
    //nop
}
