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

#include <osgEarth/Profile>
#include <osgEarth/Mercator>
#include <osgDB/FileNameUtils>
#include <algorithm>

using namespace osgEarth;

//#define GEODETIC_MIN_LON -180
//#define GEODETIC_MAX_LON 180
//#define GEODETIC_MIN_LAT -90
//#define GEODETIC_MAX_LAT 90
//
//#define GLOBAL_MERCATOR_MIN_X -20037508.34
//#define GLOBAL_MERCATOR_MIN_Y -20037508.34
//#define GLOBAL_MERCATOR_MAX_X 20037508.34
//#define GLOBAL_MERCATOR_MAX_Y 20037508.34


/** Static profile definitions **/

Profile
Profile::GLOBAL_GEODETIC = Profile(Profile::TYPE_GEODETIC,
                                   -180.0, -90.0, 180.0, 90.0,
                                   "EPSG:4326",
                                   2, 1 );

Profile
Profile::GLOBAL_MERCATOR = Profile(Profile::TYPE_MERCATOR,
                                   -20037508.34, -20037508.34, 20037508.34, 20037508.34,
                                   "EPSG:900913",
                                   1, 1 );

Profile
Profile::INVALID = Profile();


Profile
Profile::createGeodetic(unsigned int numTilesWideAtLod0,
                        unsigned int numTilesHighAtLod0)
{
    Profile p = Profile::GLOBAL_GEODETIC;
    p._numTilesHighAtLod0 = numTilesHighAtLod0;
    p._numTilesWideAtLod0 = numTilesWideAtLod0;
    return p;
}

Profile
Profile::createGlobal(const ProfileType& type)
{
    return
        type == TYPE_GEODETIC? Profile::GLOBAL_GEODETIC :
        type == TYPE_MERCATOR? Profile::GLOBAL_MERCATOR :
        Profile::INVALID;
}


Profile
Profile::createLocal(double xmin, double ymin, double xmax, double ymax,
                     const std::string& srs,
                     unsigned int numTilesWideAtLod0,
                     unsigned int numTilesHighAtLod0)
{
    return Profile(
        Profile::TYPE_LOCAL,
        xmin, ymin, xmax, ymax, srs, numTilesWideAtLod0, numTilesHighAtLod0);
}

Profile
Profile::create(const ProfileType& type,
                double xmin, double ymin, double xmax, double ymax,
                const std::string& srs,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    return Profile(type, xmin, ymin, xmax, ymax, srs, numTilesWideAtLod0, numTilesHighAtLod0);
}


Profile
Profile::create( const std::string& wkt ) 
{
    if ( wkt == STR_GLOBAL_GEODETIC )
        return Profile::GLOBAL_GEODETIC;
    else if ( wkt == STR_GLOBAL_MERCATOR )
        return Profile::GLOBAL_MERCATOR;
    else
        return Profile();
}

/****************************************************************************/

Profile::Profile()
{
    // set to UNKNOWN to render it invalid
    _profileType = Profile::TYPE_UNKNOWN;
}


Profile::Profile(ProfileType profileType,
                 double xmin, double ymin, double xmax, double ymax,
                 const std::string& srs,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0)
{
    _profileType = profileType;
    _xmin = xmin;
    _ymin = ymin;
    _xmax = xmax;
    _ymax = ymax;
    _srs = srs;
    _numTilesWideAtLod0 = numTilesWideAtLod0;
    _numTilesHighAtLod0 = numTilesHighAtLod0;
}


// copier
Profile::Profile( const Profile& rhs )
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
Profile::isValid() const
{
    return _profileType != Profile::TYPE_UNKNOWN;
}

double
Profile::xMin() const {
    return _xmin;
}

double
Profile::yMin() const {
    return _ymin;
}

double
Profile::xMax() const {
    return _xmax;
}

double
Profile::yMax() const {
    return _ymax;
}

const std::string&
Profile::srs() const {
    return _srs;
}

const Profile::ProfileType&
Profile::getProfileType() const {
    return _profileType;
}

void
Profile::getRootKeys(std::vector< osg::ref_ptr<osgEarth::TileKey> >& out_keys) const
{
    out_keys.clear();

    for (unsigned int c = 0; c < _numTilesWideAtLod0; ++c)
    {
        for (unsigned int r = 0; r < _numTilesHighAtLod0; ++r)
        {
            out_keys.push_back(new TileKey(c, r, 0, *this));
        }
    }
}

Profile::ProfileType
Profile::getProfileTypeFromSRS(const std::string& srs)
{
    //We have no SRS at all
    if (srs.empty()) return Profile::TYPE_UNKNOWN;

    std::string upperSRS = srs;

    //convert to upper case
    std::transform( upperSRS.begin(), upperSRS.end(), upperSRS.begin(), toupper );

    if (upperSRS == "EPSG:4326") //TODO: add other ellipsoids
    {
        return Profile::TYPE_GEODETIC;
    }
    else if ((upperSRS == "EPSG:41001")  ||
             (upperSRS == "OSGEO:41001") ||
             (upperSRS == "EPSG:3785")   ||
             (upperSRS == "EPSG:900913") ||
             (upperSRS == "EPSG:54004") )
    {
        return Profile::TYPE_MERCATOR;
    }

    //Assume that if we have an SRS and its not GEODETIC or MERCATOR, it's PROJECTED/LOCAL
    return Profile::TYPE_LOCAL;
}

void
Profile::applyTo( osg::CoordinateSystemNode* csn ) const
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

bool
Profile::operator == (const Profile& rhs) const
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

bool
Profile::operator != (const Profile& rhs) const
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

bool
Profile::isCompatibleWith(const Profile& rhs) const
{
    //For now, assume that if the profile types are the same, that the maps are compatible.
    if ((_profileType == rhs._profileType))
    {
        return true;
    }

    return false;
}

void
Profile::getTileDimensions(unsigned int lod, double& out_width, double& out_height) const
{
    out_width  = (_xmax - _xmin) / (double)_numTilesWideAtLod0;
    out_height = (_ymax - _ymin) / (double)_numTilesHighAtLod0;

    for (unsigned int i = 0; i < lod; ++i)
    {
        out_width /= 2.0;
        out_height /= 2.0;
    }
}

void
Profile::getNumTiles(unsigned int lod, unsigned int& out_tiles_wide, unsigned int& out_tiles_high) const
{
    out_tiles_wide = _numTilesWideAtLod0;
    out_tiles_high = _numTilesHighAtLod0;

    for (unsigned int i = 0; i < lod; ++i)
    {
        out_tiles_wide *= 2;
        out_tiles_high *= 2;
    }
}

void
Profile::getIntersectingTiles(const TileKey *key, std::vector<osg::ref_ptr<const TileKey> >& out_intersectingKeys) const
{
    //Clear the incoming list
    out_intersectingKeys.clear();

    //If the profiles are exactly equal, just add the given tile key.
    if (key->getProfile() == *this)
    {
        out_intersectingKeys.push_back(key);
        return;
    }

    if (!isCompatibleWith(key->getProfile()))
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
            out_intersectingKeys.push_back(new TileKey(i, j, destLOD, *this));
        }
    }

    //osg::notify(osg::NOTICE) << "Found " << intersectingKeys.size() << " keys " << std::endl;
}
