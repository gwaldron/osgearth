/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/Cube>
#include <osgEarth/SpatialReference>
#include <osgEarth/StringUtils>
#include <osgDB/FileNameUtils>
#include <algorithm>
#include <sstream>

using namespace osgEarth;

#define LC "[Profile] "

//------------------------------------------------------------------------

ProfileOptions::ProfileOptions( const ConfigOptions& options ) :
ConfigOptions      ( options ),
_namedProfile      ( "" ),
_srsInitString     ( "" ),
_vsrsInitString    ( "" ),
_bounds            ( Bounds() ),
_numTilesWideAtLod0( 1 ),
_numTilesHighAtLod0( 1 )
{
    fromConfig( _conf );
}

ProfileOptions::ProfileOptions( const std::string& namedProfile ) :
_srsInitString     ( "" ),
_vsrsInitString    ( "" ),
_bounds            ( Bounds() ),
_numTilesWideAtLod0( 1 ),
_numTilesHighAtLod0( 1 )
{
    _namedProfile = namedProfile; // don't set above
}

void
ProfileOptions::fromConfig( const Config& conf )
{
    if ( !conf.value().empty() )
        _namedProfile = conf.value();

    conf.getIfSet( "srs", _srsInitString );
    conf.getIfSet( "vsrs", _vsrsInitString );

    if ( conf.hasValue( "xmin" ) && conf.hasValue( "ymin" ) && conf.hasValue( "xmax" ) && conf.hasValue( "ymax" ) )
    {
        _bounds = Bounds(
            conf.value<double>( "xmin", 0 ),
            conf.value<double>( "ymin", 0 ),
            conf.value<double>( "xmax", 0 ),
            conf.value<double>( "ymax", 0 ) );
    }

    conf.getIfSet( "num_tiles_wide_at_lod_0", _numTilesWideAtLod0 );
    conf.getIfSet( "num_tiles_high_at_lod_0", _numTilesHighAtLod0 );
}

Config
ProfileOptions::getConfig() const
{
    Config conf( "profile" );
    if ( _namedProfile.isSet() )
    {
        conf.value() = _namedProfile.value();
    }
    else
    {
        conf.updateIfSet( "srs", _srsInitString );
        conf.updateIfSet( "vsrs", _vsrsInitString );

        if ( _bounds.isSet() )
        {
            conf.update( "xmin", toString(_bounds->xMin()) );
            conf.update( "ymin", toString(_bounds->yMin()) );
            conf.update( "xmax", toString(_bounds->xMax()) );
            conf.update( "ymax", toString(_bounds->yMax()) );
        }

        conf.updateIfSet( "num_tiles_wide_at_lod_0", _numTilesWideAtLod0 );
        conf.updateIfSet( "num_tiles_high_at_lod_0", _numTilesHighAtLod0 );
    }
    return conf;
}

bool
ProfileOptions::defined() const
{
    return _namedProfile.isSet() || _srsInitString.isSet();
}

/***********************************************************************/


// FACTORY METHODS:

const Profile*
Profile::create(const std::string& srsInitString,
                double xmin, double ymin, double xmax, double ymax,
                const std::string& vsrsInitString,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    return new Profile(
        SpatialReference::create( srsInitString ),
        xmin, ymin, xmax, ymax,
        VerticalSpatialReference::create( vsrsInitString ),
        numTilesWideAtLod0,
        numTilesHighAtLod0 );
}

const Profile*
Profile::create(const SpatialReference* srs,
                double xmin, double ymin, double xmax, double ymax,
                const VerticalSpatialReference* vsrs,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    return new Profile(
        srs,
        xmin, ymin, xmax, ymax,
        vsrs,
        numTilesWideAtLod0,
        numTilesHighAtLod0 );
}

const Profile*
Profile::create(const SpatialReference* srs,
                double xmin, double ymin, double xmax, double ymax,
                double geoxmin, double geoymin, double geoxmax, double geoymax,
                const VerticalSpatialReference* vsrs,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    return new Profile(
        srs,
        xmin, ymin, xmax, ymax,
        geoxmin, geoymin, geoxmax, geoymax,
        vsrs,
        numTilesWideAtLod0,
        numTilesHighAtLod0 );
}

const Profile*
Profile::create(const std::string& srsInitString,
                const std::string& vsrsInitString,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    const Profile* named = osgEarth::Registry::instance()->getNamedProfile( srsInitString );
    if ( named )
        return const_cast<Profile*>( named );

    osg::ref_ptr<const SpatialReference> srs = SpatialReference::create( srsInitString );

    osg::ref_ptr<const VerticalSpatialReference> vsrs = VerticalSpatialReference::create( vsrsInitString );

    if ( srs.valid() && srs->isGeographic() )
    {
        return new Profile(
            srs.get(),
            -180.0, -90.0, 180.0, 90.0,
            vsrs.get(),
            numTilesWideAtLod0, numTilesHighAtLod0 );
    }
    else if ( srs.valid() && srs->isMercator() )
    {
        // automatically figure out proper mercator extents:
        GDAL_SCOPED_LOCK;
        double e, dummy;
        srs->getGeographicSRS()->transform2D( 180.0, 0.0, srs.get(), e, dummy );
        return Profile::create( srs.get(), -e, -e, e, e, vsrs.get(), numTilesWideAtLod0, numTilesHighAtLod0 );
    }
    else
    {
        OE_WARN << LC << "Failed to create profile; SRS spec requires addition information: \"" << srsInitString << 
            std::endl;
    }

    return NULL;
}

const Profile*
Profile::create( const ProfileOptions& options )
{
    const Profile* result = 0L;

    // Check for a "well known named" profile:
    if ( options.namedProfile().isSet() )
    {
        result = osgEarth::Registry::instance()->getNamedProfile( options.namedProfile().value() );
    }

    // Next check for a user-defined extents:
    else if ( options.srsString().isSet() && options.bounds().isSet() )
    {
        if ( options.numTilesWideAtLod0().isSet() && options.numTilesHighAtLod0().isSet() )
        {
            result = Profile::create(
                options.srsString().value(),
                options.bounds()->xMin(),
                options.bounds()->yMin(),
                options.bounds()->xMax(),
                options.bounds()->yMax(),
                options.vsrsString().value(),
                options.numTilesWideAtLod0().value(),
                options.numTilesHighAtLod0().value() );
        }
        else
        {
            result = Profile::create(
                options.srsString().value(),
                options.bounds()->xMin(),
                options.bounds()->yMin(),
                options.bounds()->xMax(),
                options.bounds()->yMax(),
                options.vsrsString().value() );
        }
    }

    // Next try SRS with default extents
    else if ( options.srsString().isSet() )
    {
        result = Profile::create(
            options.srsString().value(),
            options.vsrsString().value() );
    }

    return result;
}

/****************************************************************************/


Profile::Profile(const SpatialReference* srs,
                 double xmin, double ymin, double xmax, double ymax,
                 const VerticalSpatialReference* vsrs,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0) :
osg::Referenced( true ),
_vsrs          ( vsrs )
{
    _extent = GeoExtent( srs, xmin, ymin, xmax, ymax );

    _numTilesWideAtLod0 = numTilesWideAtLod0 != 0? numTilesWideAtLod0 : srs->isGeographic()? 2 : 1;
    _numTilesHighAtLod0 = numTilesHighAtLod0 != 0? numTilesHighAtLod0 : 1;

    // automatically calculate the lat/long extents:
    _latlong_extent = srs->isGeographic()?
        _extent :
        _extent.transform( _extent.getSRS()->getGeographicSRS() );

    if ( !_vsrs.valid() )
        _vsrs = Registry::instance()->getDefaultVSRS();

    // gen the signature
    _signature = Stringify() << std::hex << hashString( toProfileOptions().getConfig().toJSON() );
}

Profile::Profile(const SpatialReference* srs,
                 double xmin, double ymin, double xmax, double ymax,
                 double geo_xmin, double geo_ymin, double geo_xmax, double geo_ymax,
                 const VerticalSpatialReference* vsrs,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0 ) :
osg::Referenced( true ),
_vsrs          ( vsrs )
{
    _extent = GeoExtent( srs, xmin, ymin, xmax, ymax );

    _numTilesWideAtLod0 = numTilesWideAtLod0 != 0? numTilesWideAtLod0 : srs->isGeographic()? 2 : 1;
    _numTilesHighAtLod0 = numTilesHighAtLod0 != 0? numTilesHighAtLod0 : 1;

    _latlong_extent = GeoExtent( 
        srs->getGeographicSRS(),
        geo_xmin, geo_ymin, geo_xmax, geo_ymax );

    if ( !_vsrs.valid() )
        _vsrs = Registry::instance()->getDefaultVSRS();

    // gen the signature
    _signature = Stringify() << std::hex << hashString( toProfileOptions().getConfig().toJSON() );
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
        << " vsrs=" << ( _vsrs.valid() ? _vsrs->getName() : "default" )
        << "]";
    std::string bufStr;
	bufStr = buf.str();
	return bufStr;
}

ProfileOptions
Profile::toProfileOptions() const
{
    ProfileOptions op;
    op.srsString() = getSRS()->getInitString();
    op.vsrsString() = getVerticalSRS()->getInitString();
    op.bounds()->xMin() = _extent.xMin();
    op.bounds()->yMin() = _extent.yMin();
    op.bounds()->xMax() = _extent.xMax();
    op.bounds()->yMax() = _extent.yMax();
    op.numTilesWideAtLod0() = _numTilesWideAtLod0;
    op.numTilesHighAtLod0() = _numTilesHighAtLod0;
    return op;
}

void
Profile::getRootKeys( std::vector<TileKey>& out_keys ) const
{
    out_keys.clear();

    for (unsigned int c = 0; c < _numTilesWideAtLod0; ++c)
    {
        for (unsigned int r = 0; r < _numTilesHighAtLod0; ++r)
        {
            //TODO: upgrade to support multi-face profile:
            out_keys.push_back( TileKey(0, c, r, this) ); // lod, x, y, profile
        }
    }
}

GeoExtent
Profile::calculateExtent( unsigned int lod, unsigned int tileX, unsigned int tileY )
{
    double width, height;
    getTileDimensions(lod, width, height);

    double xmin = getExtent().xMin() + (width * (double)tileX);
    double ymax = getExtent().yMax() - (height * (double)tileY);
    double xmax = xmin + width;
    double ymin = ymax - height;

    return GeoExtent( getSRS(), xmin, ymin, xmax, ymax );
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
    return rhs && getSignature() == rhs->getSignature();
    //return
    //    rhs &&
    //    _extent.isValid() && rhs->getExtent().isValid() &&
    //    _extent == rhs->getExtent() &&
    //    _numTilesWideAtLod0 == rhs->_numTilesWideAtLod0 &&
    //    _numTilesHighAtLod0 == rhs->_numTilesHighAtLod0;
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

unsigned int
Profile::getLevelOfDetailForHorizResolution( double resolution, int tileSize ) const
{
    if ( tileSize <= 0 || resolution <= 0.0 ) return 0;

    double tileRes = (_extent.width() / (double)_numTilesWideAtLod0) / (double)tileSize;
    unsigned int level = 0;
    while( tileRes > resolution ) 
    {
        level++;
        tileRes *= 0.5;
    }
    return level;
}

TileKey
Profile::createTileKey( double x, double y, unsigned int level ) const
{
    if ( _extent.contains( x, y ) )
    {
        int tilesX = (int)_numTilesWideAtLod0 * (1 << (int)level);
        int tilesY = (int)_numTilesHighAtLod0 * (1 << (int)level);

        double rx = (x - _extent.xMin()) / _extent.width();
        int tileX = osg::clampBelow( (int)(rx * (double)tilesX), tilesX-1 );
        double ry = (y - _extent.yMin()) / _extent.height();
        int tileY = osg::clampBelow( (int)((1.0-ry) * (double)tilesY), tilesY-1 );

        return TileKey( level, tileX, tileY, this );
    }
    else
    {
        return TileKey::INVALID;
    }
}

GeoExtent
Profile::clampAndTransformExtent( const GeoExtent& input, bool* out_clamped ) const
{
    if ( out_clamped )
        *out_clamped = false;

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

    if ( out_clamped )
        *out_clamped = (clamped_gcs_input != gcs_input);

    // finally, transform the clamped extent into this profile's SRS and return it.
    GeoExtent result =
        clamped_gcs_input.getSRS()->isEquivalentTo( this->getSRS() )?
        clamped_gcs_input :
        clamped_gcs_input.transform( this->getSRS() );

    if (result.isValid())
    {
        OE_DEBUG << LC << "clamp&xform: input=" << input.toString() << ", output=" << result.toString() << std::endl;
    }

    return result;
}


void
Profile::addIntersectingTiles(const GeoExtent& key_ext, std::vector<TileKey>& out_intersectingKeys) const
{
    // assume a non-crossing extent here.
    if ( key_ext.crossesDateLine() )
    {
        OE_WARN << "Profile::addIntersectingTiles cannot process date-line cross" << std::endl;
        return;
    }

    double keyWidth = key_ext.width();
    double keyHeight = key_ext.height();
    double keyArea = keyWidth * keyHeight;

    // bail out if the key has a null extent. This might happen is the original key represents an
    // area in one profile that is out of bounds in this profile.
    if ( keyArea <= 0.0 )
        return;

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
        //OE_INFO << std::fixed << "  " << currLOD << "(" << destTileWidth << ", " << destTileHeight << ")" << std::endl;
        double a = w * h;
        if (a < keyArea) break;
        destLOD = currLOD;
        destTileWidth = w;
        destTileHeight = h;
    }

    //OE_DEBUG << std::fixed << "  Source Tile: " << key.getLevelOfDetail() << " (" << keyWidth << ", " << keyHeight << ")" << std::endl;
    OE_DEBUG << std::fixed << "  Dest Size: " << destLOD << " (" << destTileWidth << ", " << destTileHeight << ")" << std::endl;

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

    OE_DEBUG << std::fixed << "  Dest Tiles: " << tileMinX << "," << tileMinY << " => " << tileMaxX << "," << tileMaxY << std::endl;

    for (int i = tileMinX; i <= tileMaxX; ++i)
    {
        for (int j = tileMinY; j <= tileMaxY; ++j)
        {
            //TODO: does not support multi-face destination keys.
            out_intersectingKeys.push_back( TileKey(destLOD, i, j, this) );
        }
    }

    OE_DEBUG << "    Found " << out_intersectingKeys.size() << " keys " << std::endl;
}


void
Profile::getIntersectingTiles(const TileKey& key, std::vector<TileKey>& out_intersectingKeys) const
{
    OE_DEBUG << "GET ISECTING TILES for key " << key.str() << " -----------------" << std::endl;

    //If the profiles are exactly equal, just add the given tile key.
    if ( isEquivalentTo( key.getProfile() ) )
    {
        //Clear the incoming list
        out_intersectingKeys.clear();

        out_intersectingKeys.push_back(key);
        return;
    }
    return getIntersectingTiles(key.getExtent(), out_intersectingKeys);
}

void
Profile::getIntersectingTiles(const GeoExtent& extent, std::vector<TileKey>& out_intersectingKeys) const
{
    GeoExtent ext = extent;

    // reproject into the profile's SRS if necessary:
    if ( ! getSRS()->isEquivalentTo( extent.getSRS() ) )
    {
        // localize the extents and clamp them to legal values
        ext = clampAndTransformExtent( extent );
        if ( !ext.isValid() )
            return;
    }

    if ( ext.crossesDateLine() )
    {
        GeoExtent first, second;
        if (ext.splitAcrossDateLine( first, second ))
        {
            addIntersectingTiles( first, out_intersectingKeys );
            addIntersectingTiles( second, out_intersectingKeys );
        }
    }
    else
    {
        addIntersectingTiles( ext, out_intersectingKeys );
    }
}
