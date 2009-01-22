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
#include <algorithm>

using namespace osgEarth;

// these bounds form a square tile set; the bottom half of LOD 0 is not used.
#define GLOBAL_GEODETIC_MIN_LON -180
#define GLOBAL_GEODETIC_MAX_LON  180
#define GLOBAL_GEODETIC_MIN_LAT -270
#define GLOBAL_GEODETIC_MAX_LAT   90

#define GLOBAL_MERCATOR_MIN_X -20037508.342789244
#define GLOBAL_MERCATOR_MIN_Y -20037508.342789244
#define GLOBAL_MERCATOR_MAX_X 20037508.342789244
#define GLOBAL_MERCATOR_MAX_Y 20037508.342789244

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
    init( profileType );
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

TileGridProfile::TileGridProfile( const std::string& _string )
{
    if ( _string == "global-geodetic" )
        init( TileGridProfile::GLOBAL_GEODETIC );
    else if ( _string == "global-mercator" )
        init( TileGridProfile::GLOBAL_MERCATOR );
    else
        init( TileGridProfile::UNKNOWN );
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

bool
TileGridProfile::isValid() const
{
    return _profileType != TileGridProfile::UNKNOWN;
}

void
TileGridProfile::init( TileGridProfile::ProfileType profileType )
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
        xmin = GLOBAL_MERCATOR_MIN_X;
        xmax = GLOBAL_MERCATOR_MAX_X;
        ymin = GLOBAL_MERCATOR_MIN_Y;
        ymax = GLOBAL_MERCATOR_MAX_Y;
        _srs = "EPSG:900913";
    }
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
    return new TileKey(key, *this);
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

bool TileGridProfile::operator == (const TileGridProfile& rhs) const
{
    if (this == &rhs) return true;

    return (_profileType == rhs._profileType &&
            _srs == rhs._srs &&
            xmin == rhs.xmin &&
            ymin == rhs.ymin &&
            xmax == rhs.xmax &&
            ymax == rhs.ymax);
}

bool TileGridProfile::operator != (const TileGridProfile& rhs) const
{
    if (this == &rhs) return false;

        return (_profileType != rhs._profileType ||
            _srs != rhs._srs ||
            xmin != rhs.xmin ||
            ymin != rhs.ymin ||
            xmax != rhs.xmax ||
            ymax != rhs.ymax);
}