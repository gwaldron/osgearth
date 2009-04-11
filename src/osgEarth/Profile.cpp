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
#include <osgEarth/Registry>
#include <osgEarth/TileKey>
#include <osgEarth/SpatialReference>
#include <osgDB/FileNameUtils>
#include <algorithm>

using namespace osgEarth;


// FACTORY METHODS:

Profile*
Profile::create(const std::string& init_string,
                double xmin, double ymin, double xmax, double ymax,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    return new Profile(
        //getProfileTypeFromSRS( init_string ),
        SpatialReference::create( init_string ),
        xmin, ymin, xmax, ymax,
        numTilesWideAtLod0,
        numTilesHighAtLod0 );
}

Profile*
Profile::create(const std::string& init_string,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    const Profile* named = osgEarth::Registry::instance()->getNamedProfile( init_string );
    if ( named )
        return const_cast<Profile*>( named );

    osg::ref_ptr<const SpatialReference> _srs = SpatialReference::create( init_string );
    if ( _srs.valid() && _srs->isGeographic() )
    {
        return new Profile(
            //getProfileTypeFromSRS( init_string ),
            _srs.get(),
            -180.0, -90.0, 180.0, 90.0 );
    }
    return 0;
}

/****************************************************************************/


Profile::Profile(//ProfileType profileType,
                 const SpatialReference* srs,
                 double xmin, double ymin, double xmax, double ymax,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0)
{
//    _profileType = profileType;
    _srs = srs;
    _xmin = xmin;
    _ymin = ymin;
    _xmax = xmax;
    _ymax = ymax;
    _numTilesWideAtLod0 = numTilesWideAtLod0;
    _numTilesHighAtLod0 = numTilesHighAtLod0;
}

Profile::ProfileType
Profile::getProfileType() const
{
    return
        _srs.valid() && _srs->isGeographic() ? TYPE_GEODETIC :
        _srs.valid() && _srs->isMercator() ? TYPE_MERCATOR :
        _srs.valid() && _srs->isProjected() ? TYPE_LOCAL :
        TYPE_UNKNOWN;
}

//Profile::Profile(const std::string& srs_string,
//                 double xmin, double ymin, double xmax, double ymax,
//                 unsigned int numTilesWideAtLod0,
//                 unsigned int numTilesHighAtLod0)
//{
//    _profileType = getProfileTypeFromSRS( srs_string );
//    _srs = SpatialReference::create( srs_string );
//    _xmin = xmin;
//    _ymin = ymin;
//    _xmax = xmax;
//    _ymax = ymax;
//    _numTilesWideAtLod0 = numTilesWideAtLod0;
//    _numTilesHighAtLod0 = numTilesHighAtLod0;
//}

bool
Profile::isOK() const
{
    return _srs.valid();
        //_profileType != Profile::TYPE_UNKNOWN &&
        //_srs.valid();
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

const SpatialReference*
Profile::getSRS() const {
    return _srs.get();
}

void
Profile::getRootKeys(std::vector< osg::ref_ptr<osgEarth::TileKey> >& out_keys) const
{
    out_keys.clear();

    for (unsigned int c = 0; c < _numTilesWideAtLod0; ++c)
    {
        for (unsigned int r = 0; r < _numTilesHighAtLod0; ++r)
        {
            out_keys.push_back(new TileKey(c, r, 0, this));
        }
    }
}

//TODO: DEPRECATE THIS and replace by examining the SRS itself.
Profile::ProfileType
Profile::getProfileTypeFromSRS(const std::string& srs_string)
{
    osg::ref_ptr<SpatialReference> srs = SpatialReference::create( srs_string );
    return 
        srs.valid() && srs->isGeographic()? Profile::TYPE_GEODETIC :
        srs.valid() && srs->isMercator()? Profile::TYPE_MERCATOR :
        srs.valid() && srs->isProjected()? Profile::TYPE_LOCAL :
        Profile::TYPE_UNKNOWN;
}

bool
Profile::isCompatibleWith( const Profile* rhs ) const
{
    return
        rhs &&
        isOK() && rhs->isOK() &&
        getProfileType() == rhs->getProfileType();
}

bool
Profile::isEquivalentTo( const Profile* rhs ) const
{
    return
        rhs &&
        _xmin == rhs->_xmin && 
        _ymin == rhs->_ymin &&
        _xmax == rhs->_xmax &&
        _ymax == rhs->_ymax &&
        _numTilesWideAtLod0 == rhs->_numTilesWideAtLod0 &&
        _numTilesHighAtLod0 == rhs->_numTilesHighAtLod0 &&
        _srs && rhs->_srs &&
        _srs->isEquivalentTo( rhs->_srs );
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
    if ( isEquivalentTo( key->getProfile() ) )
    {
        out_intersectingKeys.push_back(key);
        return;
    }

    if ( !isCompatibleWith( key->getProfile() ) )
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
            out_intersectingKeys.push_back(new TileKey(i, j, destLOD, this));
        }
    }

    //osg::notify(osg::NOTICE) << "Found " << intersectingKeys.size() << " keys " << std::endl;
}
