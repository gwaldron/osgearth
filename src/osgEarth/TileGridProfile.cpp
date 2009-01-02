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

#include <osgEarth/TileGridProfile>
#include <osgEarth/Mercator>
#include <osgEarth/PlateCarre>
#include <osgEarth/Projected>
#include <algorithm>

using namespace osgEarth;

// these bounds form a square tile set; the bottom half of LOD 0 is not used.
#define GLOBAL_GEODETIC_MIN_LON -180
#define GLOBAL_GEODETIC_MAX_LON  180
#define GLOBAL_GEODETIC_MIN_LAT -270
#define GLOBAL_GEODETIC_MAX_LAT   90


#define GLOBAL_MERCATOR_MIN_LON -180.0
#define GLOBAL_MERCATOR_MAX_LON  180.0
#define GLOBAL_MERCATOR_MIN_LAT  -85.05112878  
#define GLOBAL_MERCATOR_MAX_LAT   85.05112878

TileGridProfile::TileGridProfile():
xmin(0),
xmax(0),
ymin(0),
ymax(0),
_profileType(UNKNOWN)
{
}

TileGridProfile::TileGridProfile(TileGridProfile::ProfileType profileType)
{
    _profileType = profileType;
    if (_profileType == GLOBAL_GEODETIC)
    {
        xmin = GLOBAL_GEODETIC_MIN_LON;
        xmax = GLOBAL_GEODETIC_MAX_LON;
        ymin = GLOBAL_GEODETIC_MIN_LAT;
        ymax = GLOBAL_GEODETIC_MAX_LAT;
        _srs = "EPSG:4326";
    }
    else if (_profileType == GLOBAL_MERCATOR)
    {
        xmin = GLOBAL_MERCATOR_MIN_LON;
        xmax = GLOBAL_MERCATOR_MAX_LON;
        ymin = GLOBAL_MERCATOR_MIN_LAT;
        ymax = GLOBAL_MERCATOR_MAX_LAT;
        _srs = "EPSG:900913";
    }
}

TileGridProfile::TileGridProfile( double _xmin, double _ymin, double _xmax, double _ymax, const std::string& srs )
{
    xmin = _xmin;
    ymin = _ymin;
    xmax = _xmax;
    ymax = _ymax;
    _profileType = PROJECTED;
    _srs = srs;
}

TileGridProfile::TileGridProfile( const TileGridProfile& rhs )
: xmin( rhs.xmin ),
  ymin( rhs.ymin ),
  xmax( rhs.xmax ),
  ymax( rhs.ymax ),
  _profileType(rhs._profileType),
  _srs(rhs._srs)
{
    //NOP
}

double
TileGridProfile::xMin() const {
    return xmin;
}

double
TileGridProfile::yMin() const {
    return ymin;
}

double
TileGridProfile::xMax() const {
    return xmax;
}

double
TileGridProfile::yMax() const {
    return ymax;
}

const std::string&
TileGridProfile::srs() const {
    return _srs;
}

TileGridProfile::ProfileType
TileGridProfile::profileType() const {
    return _profileType;
}

TileKey*
TileGridProfile::getTileKey( const std::string &key ) const
{
    if (_profileType == TileGridProfile::GLOBAL_GEODETIC)
    {
        return new PlateCarreTileKey( key, *this );
    }
    else if (_profileType == TileGridProfile::GLOBAL_MERCATOR)
    {
        return new MercatorTileKey( key, *this);
    }
    else if (_profileType == TileGridProfile::PROJECTED)
    {
        return new ProjectedTileKey( key, *this );
    }
    else
    {
        return 0;
    }
}

TileGridProfile::ProfileType TileGridProfile::getProfileTypeFromSRS(const std::string &srs)
{
    //We have no SRS at all
    if (srs.empty()) return TileGridProfile::UNKNOWN;

    std::string upperSRS = srs;

    //convert to upper case
    std::transform( upperSRS.begin(), upperSRS.end(), upperSRS.begin(), toupper );

    if (upperSRS == "EPSG:4326")
    {
        return TileGridProfile::GLOBAL_GEODETIC;
    }
    else if ((upperSRS == "EPSG:41001") ||
             (upperSRS == "OSGEO:41001") ||
             (upperSRS == "EPSG:3785") ||
             (upperSRS == "EPSG:900913"))
    {
        return TileGridProfile::GLOBAL_MERCATOR;
    }

    //Assume that if we have an SRS and its not GLOBAL_GEODETIC or GLOBAL_MERCATOR, it's PROJECTED
    return TileGridProfile::PROJECTED;
}