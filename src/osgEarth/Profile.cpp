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
#include <sstream>

using namespace osgEarth;


// FACTORY METHODS:

Profile*
Profile::create(const std::string& init_string,
                double xmin, double ymin, double xmax, double ymax,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    return new Profile(
        SpatialReference::create( init_string ),
        xmin, ymin, xmax, ymax,
        numTilesWideAtLod0,
        numTilesHighAtLod0 );
}

Profile*
Profile::create(const SpatialReference* srs,
                double xmin, double ymin, double xmax, double ymax,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    return new Profile(
        srs,
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
            _srs.get(),
            -180.0, -90.0, 180.0, 90.0 );
    }
    return 0;
}

Profile*
Profile::createCube(const SpatialReference* geog_srs)
{
    Profile* result = new Profile( geog_srs, -180.0, -90.0, 180.0, 90.0 ); //-180.0, -90.0, 180.0, 90.0 );
    //result->_face_profiles.push_back( new Profile( geog_srs, -180.0, -45.0, -90.0, 45.0 ) );
    //result->_face_profiles.push_back( new Profile( geog_srs,  -90.0, -45.0,   0.0, 45.0 ) );
    //result->_face_profiles.push_back( new Profile( geog_srs,    0.0, -45.0,  90.0, 45.0 ) );
    //result->_face_profiles.push_back( new Profile( geog_srs,   90.0, -45.0, 180.0, 45.0 ) );
    //result->_face_profiles.push_back( new Profile( SpatialReference::create( "square-polar" ), -180.0, 45.0, 180.0, 90.0, 4, 1 ) );
    //result->_face_profiles.push_back( new Profile( SpatialReference::create( "square-polar" ), -180.0, 45.0, -90, 90, 1, 1 ) );
    //result->_face_profiles.push_back( new Profile( SpatialReference::create( "square-polar" ), 0.0, 0.0, 1.0, 1.0, 1, 1 ) );
    //result->_face_profiles.push_back( new Profile( SpatialReference::create( "square-polar" ), -180.0, -90.0, 180.0, -45.0 ) );

    result->_face_profiles.push_back( new Profile( new FaceSpatialReference(0), 0.0, 0.0, 1.0, 1.0, 1, 1 ) );
    result->_face_profiles.push_back( new Profile( new FaceSpatialReference(1), 0.0, 0.0, 1.0, 1.0, 1, 1 ) );
    result->_face_profiles.push_back( new Profile( new FaceSpatialReference(2), 0.0, 0.0, 1.0, 1.0, 1, 1 ) );
    result->_face_profiles.push_back( new Profile( new FaceSpatialReference(3), 0.0, 0.0, 1.0, 1.0, 1, 1 ) );
    result->_face_profiles.push_back( new Profile( new FaceSpatialReference(4), 0.0, 0.0, 1.0, 1.0, 1, 1 ) );
    result->_face_profiles.push_back( new Profile( new FaceSpatialReference(5), 0.0, 0.0, 1.0, 1.0, 1, 1 ) );
    return result;
}

/****************************************************************************/


Profile::Profile(const SpatialReference* srs,
                 double xmin, double ymin, double xmax, double ymax,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0)
{
    _extent = GeoExtent( srs, xmin, ymin, xmax, ymax );
    _numTilesWideAtLod0 = numTilesWideAtLod0;
    _numTilesHighAtLod0 = numTilesHighAtLod0;

    // automatically calculate the lat/long extents:
    _latlong_extent = srs->isGeographic()?
        _extent :
        _extent.transform( _extent.getSRS()->getGeographicSRS() );
}

Profile::ProfileType
Profile::getProfileType() const
{
    return
        _extent.isValid() && _extent.getSRS()->isGeographic() ? TYPE_GEODETIC :
        _extent.isValid() && _extent.getSRS()->isMercator() ? TYPE_MERCATOR :
        _extent.isValid() && _extent.getSRS()->isProjected() ? TYPE_LOCAL :
        TYPE_UNKNOWN;
}

bool
Profile::isOK() const {
    return _extent.isValid();
}

const SpatialReference*
Profile::getSRS() const {
    return _extent.getSRS();
}

const GeoExtent&
Profile::getExtent() const {
    return _extent;
}

const GeoExtent&
Profile::getLatLongExtent() const {
    return _latlong_extent;
}

std::string
Profile::toString() const
{
    std::stringstream buf;
    buf << "[srs=" << _extent.getSRS()->getName() << ", min=" << _extent.xMin() << "," << _extent.yMin()
        << " max=" << _extent.xMax() << "," << _extent.yMax()
        << " lod0=" << _numTilesWideAtLod0 << "," << _numTilesHighAtLod0
        << "]";
    return buf.str();
}

int
Profile::getNumFaces() const
{
    return _face_profiles.size() > 0? _face_profiles.size() : 1;
}

const Profile*
Profile::getFaceProfile( int face ) const
{
    return _face_profiles.size() > 0 && face < _face_profiles.size() ?
        _face_profiles[face] :
        this;
}

void
Profile::getRootKeys(std::vector< osg::ref_ptr<osgEarth::TileKey> >& out_keys, int face) const
{
    out_keys.clear();

    for (unsigned int c = 0; c < _numTilesWideAtLod0; ++c)
    {
        for (unsigned int r = 0; r < _numTilesHighAtLod0; ++r)
        {
            //TODO: upgrade to support multi-face profile:
            out_keys.push_back( new TileKey(face, 0, c, r, this) ); // face, lod, x, y, profile
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
Profile::isEquivalentTo( const Profile* rhs ) const
{
    return
        rhs &&
        _extent.isValid() && rhs->getExtent().isValid() &&
        _extent == rhs->getExtent() &&
        _numTilesWideAtLod0 == rhs->_numTilesWideAtLod0 &&
        _numTilesHighAtLod0 == rhs->_numTilesHighAtLod0;
}

void
Profile::getTileDimensions(unsigned int lod, double& out_width, double& out_height) const
{
    out_width  = (_extent.xMax() - _extent.xMin()) / (double)_numTilesWideAtLod0;
    out_height = (_extent.yMax() - _extent.yMin()) / (double)_numTilesHighAtLod0;

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

GeoExtent
Profile::clampAndTransformExtent( const GeoExtent& input ) const
{
    // do the clamping in LAT/LONG.
    const SpatialReference* geo_srs = getSRS()->getGeographicSRS();

    // get the input in lat/long:
    GeoExtent gcs_input =
        input.getSRS()->isGeographic()?
        input :
        input.transform( geo_srs );

    if ( !gcs_input.isValid() )
        return GeoExtent::INVALID;

    // clamp it to the profile's extents:
    GeoExtent clamped_gcs_input = GeoExtent(
        gcs_input.getSRS(),
        osg::clampBetween( gcs_input.xMin(), _latlong_extent.xMin(), _latlong_extent.xMax() ),
        osg::clampBetween( gcs_input.yMin(), _latlong_extent.yMin(), _latlong_extent.yMax() ),
        osg::clampBetween( gcs_input.xMax(), _latlong_extent.xMin(), _latlong_extent.xMax() ),
        osg::clampBetween( gcs_input.yMax(), _latlong_extent.yMin(), _latlong_extent.yMax() ) );

    // finally, transform the clamped extent into this profile's SRS and return it.
    return
        clamped_gcs_input.getSRS()->isEquivalentTo( this->getSRS() )?
        clamped_gcs_input :
        clamped_gcs_input.transform( this->getSRS() );
}


void
Profile::getIntersectingTiles(const TileKey* key, std::vector<osg::ref_ptr<const TileKey> >& out_intersectingKeys) const
{
    osg::notify(osg::INFO) << "GET ISECTING TILES for key " << key->str() << std::endl;

    //Clear the incoming list
    out_intersectingKeys.clear();

    //If the profiles are exactly equal, just add the given tile key.
    if ( isEquivalentTo( key->getProfile() ) )
    {
        out_intersectingKeys.push_back(key);
        return;
    }

    //TODO: put this back in???
    //if ( !isCompatibleWith( key->getProfile() ) )
    //{
    //    osg::notify(osg::NOTICE) << "Cannot compute intersecting tiles, profiles are incompatible" << std::endl;
    //}

    GeoExtent key_ext = key->getGeoExtent();

    //double keyMinX, keyMinY, keyMaxX, keyMaxY;
    //key->getGeoExtents( keyMinX, keyMinY, keyMaxX, keyMaxY );
    //const SpatialReference* key_srs = key->getProfile()->getSRS();

    // reproject into the profile's SRS if necessary:
    if ( ! getSRS()->isEquivalentTo( key_ext.getSRS() ) )
    {
        // localize the extents and clamp them to legal values
        key_ext = clampAndTransformExtent( key_ext );
        if ( !key_ext.isValid() )
            return;
    }

    double keyWidth = key_ext.xMax() - key_ext.xMin();
    double keyHeight = key_ext.yMax() - key_ext.yMin();

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
        osg::notify(osg::INFO) << std::fixed << "  " << currLOD << "(" << destTileWidth << ", " << destTileHeight << ")" << std::endl;
        double a = w * h;
        if (a < keyArea) break;
        destLOD = currLOD;
        destTileWidth = w;
        destTileHeight = h;
    }


    osg::notify(osg::INFO) << std::fixed << "  Source Tile: " << key->getLevelOfDetail() << " (" << keyWidth << ", " << keyHeight << ")" << std::endl;
    osg::notify(osg::INFO) << std::fixed << "  Dest Size: " << destLOD << " (" << destTileWidth << ", " << destTileHeight << ")" << std::endl;

    int tileMinX = (int)((key_ext.xMin() - _extent.xMin()) / destTileWidth);
    int tileMaxX = (int)((key_ext.xMax() - _extent.xMin()) / destTileWidth);

    int tileMinY = (int)((_extent.yMax() - key_ext.yMax()) / destTileHeight); 
    int tileMaxY = (int)((_extent.yMax() - key_ext.yMin()) / destTileHeight); 

    unsigned int numWide, numHigh;
    getNumTiles(destLOD, numWide, numHigh);

    tileMinX = osg::clampBetween(tileMinX, 0, (int)numWide-1);
    tileMaxX = osg::clampBetween(tileMaxX, 0, (int)numWide-1);
    tileMinY = osg::clampBetween(tileMinY, 0, (int)numHigh-1);
    tileMaxY = osg::clampBetween(tileMaxY, 0, (int)numHigh-1);

    osg::notify(osg::INFO) << std::fixed << "  Dest Tiles: " << tileMinX << "," << tileMinY << " => " << tileMaxX << "," << tileMaxY << std::endl;

    for (int i = tileMinX; i <= tileMaxX; ++i)
    {
        for (int j = tileMinY; j <= tileMaxY; ++j)
        {
            //TODO: does not support multi-face destination keys.
            out_intersectingKeys.push_back( new TileKey(0, destLOD, i, j, this) );
        }
    }

    //osg::notify(osg::NOTICE) << "Found " << intersectingKeys.size() << " keys " << std::endl;
}
