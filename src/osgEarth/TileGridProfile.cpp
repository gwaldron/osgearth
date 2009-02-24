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
#include <osgDB/FileNameUtils>
#include <algorithm>

using namespace osgEarth;

#define GLOBAL_GEODETIC_MIN_LON -180
#define GLOBAL_GEODETIC_MAX_LON 180
#define GLOBAL_GEODETIC_MIN_LAT -90
#define GLOBAL_GEODETIC_MAX_LAT 90

#define GLOBAL_MERCATOR_MIN_X -20037508.34
#define GLOBAL_MERCATOR_MIN_Y -20037508.34
#define GLOBAL_MERCATOR_MAX_X 20037508.43
#define GLOBAL_MERCATOR_MAX_Y 20037508.34

TileGridProfile::TileGridProfile():
_xmin(0),
_xmax(0),
_ymin(0),
_ymax(0),
_profileType(UNKNOWN),
_numTilesWideAtLod0(1),
_numTilesHighAtLod0(1)

{
}

TileGridProfile::TileGridProfile(TileGridProfile::ProfileType profileType)
{
    init( profileType );
}

TileGridProfile::TileGridProfile( ProfileType profileType, double xmin, double ymin, double xmax, double ymax, const std::string& srs )
{
    _numTilesHighAtLod0 = 1;
    _numTilesWideAtLod0 = 1;
    _xmin = xmin;
    _ymin = ymin;
    _xmax = xmax;
    _ymax = ymax;
    _profileType = profileType;
    _srs = srs;
    if (_profileType == GLOBAL_GEODETIC)
    {
        _numTilesWideAtLod0 = 2;
    }
}

TileGridProfile::TileGridProfile( double xmin, double ymin, double xmax, double ymax, const std::string& srs )
{
    _numTilesHighAtLod0 = 1;
    _numTilesWideAtLod0 = 1;
    _xmin = xmin;
    _ymin = ymin;
    _xmax = xmax;
    _ymax = ymax;
    _profileType = getProfileTypeFromSRS( srs );
    _srs = srs;    
    if (_profileType == GLOBAL_GEODETIC)
    {
        _numTilesWideAtLod0 = 2;
    }
}

TileGridProfile::TileGridProfile( const std::string& _string )
{
    if ( _string == STR_GLOBAL_GEODETIC )
        init( TileGridProfile::GLOBAL_GEODETIC );
    else if ( _string == STR_GLOBAL_MERCATOR )
        init( TileGridProfile::GLOBAL_MERCATOR );
    else
        init( TileGridProfile::UNKNOWN );
}

TileGridProfile::TileGridProfile( const TileGridProfile& rhs )
: _xmin( rhs._xmin ),
  _ymin( rhs._ymin ),
  _xmax( rhs._xmax ),
  _ymax( rhs._ymax ),
  _profileType(rhs._profileType),
  _srs(rhs._srs),
  _numTilesWideAtLod0(rhs._numTilesWideAtLod0),
  _numTilesHighAtLod0(rhs._numTilesHighAtLod0)
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
    _numTilesHighAtLod0 = 1;
    _numTilesWideAtLod0 = 1;

    if (_profileType == GLOBAL_GEODETIC)
    {
        _xmin = GLOBAL_GEODETIC_MIN_LON;
        _xmax = GLOBAL_GEODETIC_MAX_LON;
        _ymin = GLOBAL_GEODETIC_MIN_LAT;
        _ymax = GLOBAL_GEODETIC_MAX_LAT;
        _srs = "EPSG:4326";
        _numTilesWideAtLod0 = 2;
    }
    else if (_profileType == GLOBAL_MERCATOR)
    {
        _xmin = GLOBAL_MERCATOR_MIN_X;
        _xmax = GLOBAL_MERCATOR_MAX_X;
        _ymin = GLOBAL_MERCATOR_MIN_Y;
        _ymax = GLOBAL_MERCATOR_MAX_Y;
        _srs = "EPSG:900913";
    }
}

double
TileGridProfile::xMin() const {
    return _xmin;
}

double
TileGridProfile::yMin() const {
    return _ymin;
}

double
TileGridProfile::xMax() const {
    return _xmax;
}

double
TileGridProfile::yMax() const {
    return _ymax;
}

const std::string&
TileGridProfile::srs() const {
    return _srs;
}

TileGridProfile::ProfileType
TileGridProfile::getProfileType() const {
    return _profileType;
}

void TileGridProfile::getRootKeys(std::vector< osg::ref_ptr<osgEarth::TileKey> > &keys) const
{
    keys.clear();

    for (unsigned int c = 0; c < _numTilesWideAtLod0; ++c)
    {
        for (unsigned int r = 0; r < _numTilesHighAtLod0; ++r)
        {
            keys.push_back(new TileKey(c, r, 0, *this));
        }
    }
}

TileGridProfile::ProfileType TileGridProfile::getProfileTypeFromSRS(const std::string& srs)
{
    //We have no SRS at all
    if (srs.empty()) return TileGridProfile::UNKNOWN;

    std::string upperSRS = srs;

    //convert to upper case
    std::transform( upperSRS.begin(), upperSRS.end(), upperSRS.begin(), toupper );

    if (upperSRS == "EPSG:4326") //TODO: add other ellipsoids
    {
        return TileGridProfile::GLOBAL_GEODETIC;
    }
    else if ((upperSRS == "EPSG:41001")  ||
             (upperSRS == "OSGEO:41001") ||
             (upperSRS == "EPSG:3785")   ||
             (upperSRS == "EPSG:900913") ||
             (upperSRS == "EPSG:54004") )
    {
        return TileGridProfile::GLOBAL_MERCATOR;
    }

    //Assume that if we have an SRS and its not GLOBAL_GEODETIC or GLOBAL_MERCATOR, it's PROJECTED
    return TileGridProfile::PROJECTED;
}

void
TileGridProfile::applyTo( osg::CoordinateSystemNode* csn ) const
{
    if ( csn )
    {
        // first the format:
        std::string upperSRS = _srs;
        std::transform( upperSRS.begin(), upperSRS.end(), upperSRS.begin(), toupper );

        if ( upperSRS.length() >= 6 && ( upperSRS.substr( 0, 6 ) == "GEOGCS" || upperSRS.substr( 0, 6 ) == "PROJCS" ) )
        {
            csn->setFormat( "WKT" );
            csn->setCoordinateSystem( _srs );
        }
        else if ( upperSRS.length() >= 5 && ( upperSRS.substr( 0, 5 ) == "EPSG:" || upperSRS.substr( 0, 6 ) == "OSGEO:" ) )
        {
            csn->setFormat( "PROJ4" );
            std::string temp = _srs;
            std::transform( temp.begin(), temp.end(), temp.begin(), tolower );
            csn->setCoordinateSystem( "+init=" + temp );
        }
        else
        {
            // unknown/unsupported format
            csn->setFormat( "UNKNOWN" );
            csn->setCoordinateSystem( _srs );
        }
    }
}

bool TileGridProfile::operator == (const TileGridProfile& rhs) const
{
    if (this == &rhs) return true;

    return (_profileType == rhs._profileType &&
            osgDB::equalCaseInsensitive(_srs, rhs._srs) &&
            _numTilesWideAtLod0 == rhs._numTilesWideAtLod0 &&
            _numTilesHighAtLod0 == rhs._numTilesHighAtLod0 &&
            _xmin == rhs._xmin &&
            _ymin == rhs._ymin &&
            _xmax == rhs._xmax &&
            _ymax == rhs._ymax);
}

bool TileGridProfile::operator != (const TileGridProfile& rhs) const
{
    if (this == &rhs) return false;

        return (_profileType != rhs._profileType ||
            _numTilesWideAtLod0 != rhs._numTilesWideAtLod0 ||
            _numTilesHighAtLod0 != rhs._numTilesHighAtLod0 ||
            !osgDB::equalCaseInsensitive(_srs, rhs._srs) ||
            _xmin != rhs._xmin ||
            _ymin != rhs._ymin ||
            _xmax != rhs._xmax ||
            _ymax != rhs._ymax);
}

bool TileGridProfile::isCompatible(const TileGridProfile& rhs) const
{
    //For now, assume that if the profile types are the same, that the maps are compatible.
    if ((_profileType == rhs._profileType))
    {
        return true;
    }

    return false;
}

void TileGridProfile::getTileDimensions(unsigned int lod, double &width, double &height) const
{
    width  = (_xmax - _xmin) / (double)_numTilesWideAtLod0;
    height = (_ymax - _ymin) / (double)_numTilesHighAtLod0;

    for (unsigned int i = 0; i < lod; ++i)
    {
        width /= 2.0;
        height /= 2.0;
    }
}

void TileGridProfile::getNumTiles(unsigned int lod, unsigned int &tilesWide, unsigned &tilesHigh) const
{
    tilesWide = _numTilesWideAtLod0;
    tilesHigh = _numTilesHighAtLod0;

    for (unsigned int i = 0; i < lod; ++i)
    {
        tilesWide *= 2;
        tilesHigh *= 2;
    }
}

void TileGridProfile::getIntersectingTiles(const TileKey *key, std::vector<osg::ref_ptr<const TileKey> > &intersectingKeys) const
{
    //Clear the incoming list
    intersectingKeys.clear();

    //If the profiles are exactly equal, just add the given tile key.
    if (key->getProfile() == *this)
    {
        intersectingKeys.push_back(key);
        return;
    }

    if (!isCompatible(key->getProfile()))
    {
        osg::notify(osg::NOTICE) << "Cannot compute intersecting tiles, profiles are incompatible" << std::endl;
    }

    double keyMinX, keyMinY, keyMaxX, keyMaxY;
    key->getGeoExtents(keyMinX, keyMinY, keyMaxX, keyMaxY);

    double keyWidth = keyMaxX - keyMinX;
    double keyHeight = keyMaxY - keyMinY;

    double keyArea = keyWidth * keyHeight;

    int destLOD = 1;
    double destTileWidth, destTileHeight;

    int currLOD = 0;
    destLOD = currLOD;
    getTileDimensions(destLOD, destTileWidth, destTileHeight);

    //Find the LOD that most closely matches the area of the incoming key without going under.
    while (true)
    {
        currLOD++;
        double w, h;
        getTileDimensions(currLOD, w,h);
        //osg::notify(osg::NOTICE) << currLOD << "(" << destTileWidth << ", " << destTileHeight << ")" << std::endl;
        double a = w * h;
        if (a < keyArea) break;
        destLOD = currLOD;
        destTileWidth = w;
        destTileHeight = h;
    }


    osg::notify(osg::INFO) << "Source Tile: " << key->getLevelOfDetail() << " (" << keyWidth << ", " << keyHeight << ")" << std::endl;
    osg::notify(osg::INFO) << "Dest Tiles: " << destLOD << " (" << destTileWidth << ", " << destTileHeight << ")" << std::endl;

    int tileMinX = (int)((keyMinX - _xmin) / destTileWidth);
    int tileMaxX = (int)((keyMaxX - _xmin) / destTileWidth);

    int tileMinY = (int)((_ymax - keyMaxY) / destTileHeight); 
    int tileMaxY = (int)((_ymax - keyMinY) / destTileHeight); 

    unsigned int numWide, numHigh;
    getNumTiles(destLOD, numWide, numHigh);

    tileMinX = osg::clampBetween(tileMinX, 0, (int)numWide-1);
    tileMaxX = osg::clampBetween(tileMaxX, 0, (int)numWide-1);
    tileMinY = osg::clampBetween(tileMinY, 0, (int)numHigh-1);
    tileMaxY = osg::clampBetween(tileMaxY, 0, (int)numHigh-1);

    for (int i = tileMinX; i <= tileMaxX; ++i)
    {
        for (int j = tileMinY; j <= tileMaxY; ++j)
        {
            intersectingKeys.push_back(new TileKey(i, j, destLOD, *this));
        }
    }

    //osg::notify(osg::NOTICE) << "Found " << intersectingKeys.size() << " keys " << std::endl;
}
