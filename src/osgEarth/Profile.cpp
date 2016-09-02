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

    conf.getIfSet( "vdatum", _vsrsInitString );
    conf.getIfSet( "vsrs", _vsrsInitString ); // back compat

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
        conf.updateIfSet( "vdatum", _vsrsInitString );

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
    osg::ref_ptr<osgEarth::SpatialReference> srs = SpatialReference::create( srsInitString, vsrsInitString );
    if (srs.valid() == true)
    {
        return new Profile(
            srs.get(),
            xmin, ymin, xmax, ymax,
            numTilesWideAtLod0,
            numTilesHighAtLod0 );
    }

    OE_WARN << LC << "Failed to create profile; unrecognized SRS: \"" << srsInitString << "\"" << std::endl;
    return NULL;
}

const Profile*
Profile::create(const SpatialReference* srs,
                double xmin, double ymin, double xmax, double ymax,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    if (srs != NULL)
    {
        return new Profile(
            srs,
            xmin, ymin, xmax, ymax,
            numTilesWideAtLod0,
            numTilesHighAtLod0 );
    }

    OE_WARN << LC << "Failed to create profile; null SRS" << std::endl;
    return 0L;
}

const Profile*
Profile::create(const SpatialReference* srs,
                double xmin, double ymin, double xmax, double ymax,
                double geoxmin, double geoymin, double geoxmax, double geoymax,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    if ( srs )
    {
        return new Profile(
            srs,
            xmin, ymin, xmax, ymax,
            geoxmin, geoymin, geoxmax, geoymax,
            numTilesWideAtLod0,
            numTilesHighAtLod0 );
    }
    
    OE_WARN << LC << "Failed to create profile; null SRS" << std::endl;
    return 0L;
}

const Profile*
Profile::create(const std::string& srsInitString,
                const std::string& vsrsInitString,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    if ( vsrsInitString.empty() && numTilesWideAtLod0 == 0 && numTilesHighAtLod0 == 0 )
    {
        const Profile* named = osgEarth::Registry::instance()->getNamedProfile( srsInitString );
        if ( named )
            return const_cast<Profile*>( named );
    }

    osg::ref_ptr<const SpatialReference> srs = SpatialReference::create( srsInitString, vsrsInitString );

    if ( srs.valid() && srs->isGeographic() )
    {
        return new Profile(
            srs.get(),
            -180.0, -90.0, 180.0, 90.0,
            numTilesWideAtLod0, numTilesHighAtLod0 );
    }
    else if ( srs.valid() && srs->isMercator() )
    {
        // automatically figure out proper mercator extents:
        osg::Vec3d point(180.0, 0.0, 0.0);
        srs->getGeographicSRS()->transform(point, srs.get(), point);
        double e = point.x();
        return Profile::create( srs.get(), -e, -e, e, e, numTilesWideAtLod0, numTilesHighAtLod0 );
    }
    else if ( srs.valid() )
    {
        OE_WARN << LC << "Failed to create profile; you must provide extents with a projected SRS." << std::endl;
    }
    else
    {
        OE_WARN << LC << "Failed to create profile; unrecognized SRS: \"" << srsInitString << "\"" << std::endl;
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
        result = Profile::createNamed(options.namedProfile().get());
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
        if ( options.numTilesWideAtLod0().isSet() && options.numTilesHighAtLod0().isSet() )
        {
            result = Profile::create(
                options.srsString().value(),
                options.vsrsString().value(),
                options.numTilesWideAtLod0().value(),
                options.numTilesHighAtLod0().value() );
        }
        else
        {
            result = Profile::create(
                options.srsString().value(),
                options.vsrsString().value() );
        }
    }

    return result;
}

const Profile*
Profile::createNamed(const std::string& name)
{
    // TODO: move the named profiles from Registry into here.
    if ( ciEquals(name, "plate-carre") || ciEquals(name, "eqc-wgs84") )
    {
        // Yes I know this is not really Plate Carre but it will stand in for now.
        return Profile::create(
            "+proj=eqc +units=m +no_defs",
            -20037508, -10001966,
             20037508,  10001966,
            "", // vdatum
            2, 1 );

    }

    else
    {
        return osgEarth::Registry::instance()->getNamedProfile( name );
    }
}

/****************************************************************************/


Profile::Profile(const SpatialReference* srs,
                 double xmin, double ymin, double xmax, double ymax,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0) :

osg::Referenced( true )
{
    _extent = GeoExtent( srs, xmin, ymin, xmax, ymax );

    _numTilesWideAtLod0 = numTilesWideAtLod0 != 0? numTilesWideAtLod0 : srs->isGeographic()? 2 : 1;
    _numTilesHighAtLod0 = numTilesHighAtLod0 != 0? numTilesHighAtLod0 : 1;

    // automatically calculate the lat/long extents:
    _latlong_extent = srs->isGeographic()?
        _extent :
        _extent.transform( _extent.getSRS()->getGeographicSRS() );

    // make a profile sig (sans srs) and an srs sig for quick comparisons.
    ProfileOptions temp = toProfileOptions();
    _fullSignature = Stringify() << std::hex << hashString( temp.getConfig().toJSON() );
    temp.vsrsString() = "";
    _horizSignature = Stringify() << std::hex << hashString( temp.getConfig().toJSON() );
}

Profile::Profile(const SpatialReference* srs,
                 double xmin, double ymin, double xmax, double ymax,
                 double geo_xmin, double geo_ymin, double geo_xmax, double geo_ymax,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0 ) :

osg::Referenced( true )
{
    _extent = GeoExtent( srs, xmin, ymin, xmax, ymax );

    _numTilesWideAtLod0 = numTilesWideAtLod0 != 0? numTilesWideAtLod0 : srs->isGeographic()? 2 : 1;
    _numTilesHighAtLod0 = numTilesHighAtLod0 != 0? numTilesHighAtLod0 : 1;

    _latlong_extent = GeoExtent( 
        srs->getGeographicSRS(),
        geo_xmin, geo_ymin, geo_xmax, geo_ymax );

    //if ( !_vsrs.valid() )
    //    _vsrs = Registry::instance()->getDefaultVSRS();

    // make a profile sig (sans srs) and an srs sig for quick comparisons.
    ProfileOptions temp = toProfileOptions();
    _fullSignature = Stringify() << std::hex << hashString( temp.getConfig().toJSON() );
    temp.vsrsString() = "";
    _horizSignature = Stringify() << std::hex << hashString( temp.getConfig().toJSON() );
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
    const SpatialReference* srs = _extent.getSRS();
    return Stringify()
        << std::setprecision(16)
        << "[srs=" << srs->getName() << ", min=" << _extent.xMin() << "," << _extent.yMin()
        << " max=" << _extent.xMax() << "," << _extent.yMax()
        << " lod0=" << _numTilesWideAtLod0 << "," << _numTilesHighAtLod0
        << " vdatum=" << (srs->getVerticalDatum() ? srs->getVerticalDatum()->getName() : "geodetic")
        << "]";
}

ProfileOptions
Profile::toProfileOptions() const
{
    ProfileOptions op;
    op.srsString() = getSRS()->getHorizInitString();
    op.vsrsString() = getSRS()->getVertInitString();
    op.bounds()->xMin() = _extent.xMin();
    op.bounds()->yMin() = _extent.yMin();
    op.bounds()->xMax() = _extent.xMax();
    op.bounds()->yMax() = _extent.yMax();
    op.numTilesWideAtLod0() = _numTilesWideAtLod0;
    op.numTilesHighAtLod0() = _numTilesHighAtLod0;
    return op;
}


Profile*
Profile::overrideSRS( const SpatialReference* srs ) const
{
    return new Profile(
        srs,
        _extent.xMin(), _extent.yMin(), _extent.xMax(), _extent.yMax(),
        _numTilesWideAtLod0, _numTilesHighAtLod0 );
}

void
Profile::getRootKeys( std::vector<TileKey>& out_keys ) const
{
    getAllKeysAtLOD(0, out_keys);
}

void
Profile::getAllKeysAtLOD( unsigned lod, std::vector<TileKey>& out_keys ) const
{
    out_keys.clear();

    unsigned tx, ty;
    getNumTiles( lod, tx, ty );

    for(unsigned c=0; c<tx; ++c)
    {
        for(unsigned r=0; r<ty; ++r)
        {
            out_keys.push_back( TileKey(lod, c, r, this) );
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
    return rhs && getFullSignature() == rhs->getFullSignature();
}

bool
Profile::isHorizEquivalentTo( const Profile* rhs ) const
{
    return rhs && getHorizSignature() == rhs->getHorizSignature();
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
        unsigned tilesX = _numTilesWideAtLod0 * (1 << (unsigned)level);
        unsigned tilesY = _numTilesHighAtLod0 * (1 << (unsigned)level);

        // overflow checks:

        if (_numTilesWideAtLod0 == 0u || ((tilesX / _numTilesWideAtLod0) != (1 << (unsigned)level)))
            return TileKey::INVALID;

        if (_numTilesHighAtLod0 == 0u || ((tilesY / _numTilesHighAtLod0) != (1 << (unsigned)level)))
            return TileKey::INVALID;

        //if (((_numTilesWideAtLod0 != 0) && ((tilesX / _numTilesWideAtLod0) != (1 << (unsigned int) level))) ||
        //    ((_numTilesHighAtLod0 != 0) && ((tilesY / _numTilesHighAtLod0) != (1 << (unsigned int) level))))
        //{	// check for overflow condition
        //    return (TileKey::INVALID);
        //}

        double rx = (x - _extent.xMin()) / _extent.width();
        int tileX = osg::clampBelow( (unsigned int)(rx * (double)tilesX), tilesX-1 );
        double ry = (y - _extent.yMin()) / _extent.height();
        int tileY = osg::clampBelow( (unsigned int)((1.0-ry) * (double)tilesY), tilesY-1 );

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

    // bail out on a bad transform:
    if ( !gcs_input.isValid() )
        return GeoExtent::INVALID;

    // bail out if the extent's do not intersect at all:
    if ( !gcs_input.intersects(_latlong_extent, false) )
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

namespace
{
    double round( double in, int places )
    {
        for(int i=0; i<places; ++i)
            in *= 10.0;
        in = ceil(in);
        for(int i=0; i<places; ++i)
            in *= 0.1;
        return in;
    }

    int quantize( double in, double epsilon )
    {
        int floored = (int)in;
        int floored2 = (int)(in + epsilon);
        return floored == floored2 ? floored : floored2;
    }
}

void
Profile::addIntersectingTiles(const GeoExtent& key_ext, unsigned localLOD, std::vector<TileKey>& out_intersectingKeys) const
{
    // assume a non-crossing extent here.
    if ( key_ext.crossesAntimeridian() )
    {
        OE_WARN << "Profile::addIntersectingTiles cannot process date-line cross" << std::endl;
        return;
    }

    int tileMinX, tileMaxX;
    int tileMinY, tileMaxY;

    double destTileWidth, destTileHeight;
    getTileDimensions(localLOD, destTileWidth, destTileHeight);

    //OE_DEBUG << std::fixed << "  Source Tile: " << key.getLevelOfDetail() << " (" << keyWidth << ", " << keyHeight << ")" << std::endl;
    //OE_DEBUG << std::fixed << "  Dest Size: " << destLOD << " (" << destTileWidth << ", " << destTileHeight << ")" << std::endl;

    double east = key_ext.xMax() - _extent.xMin();
    bool xMaxOnTileBoundary = fmod(east, destTileWidth) == 0.0;

    double south = _extent.yMax() - key_ext.yMin();
    bool yMaxOnTileBoundary = fmod(south, destTileHeight) == 0.0;

    tileMinX = (int)((key_ext.xMin() - _extent.xMin()) / destTileWidth);
    tileMaxX = (int)(east / destTileWidth) - (xMaxOnTileBoundary ? 1 : 0);

    tileMinY = (int)((_extent.yMax() - key_ext.yMax()) / destTileHeight); 
    tileMaxY = (int)(south / destTileHeight) - (yMaxOnTileBoundary ? 1 : 0);

    unsigned int numWide, numHigh;
    getNumTiles(localLOD, numWide, numHigh);

    // bail out if the tiles are out of bounds.
    if ( tileMinX >= (int)numWide || tileMinY >= (int)numHigh ||
         tileMaxX < 0 || tileMaxY < 0 )
    {
        return;
    }

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
            out_intersectingKeys.push_back( TileKey(localLOD, i, j, this) );
        }
    }
}


void
Profile::getIntersectingTiles(const TileKey& key, std::vector<TileKey>& out_intersectingKeys) const
{
    OE_DEBUG << "GET ISECTING TILES for key " << key.str() << " -----------------" << std::endl;

    //If the profiles are exactly equal, just add the given tile key.
    if ( isHorizEquivalentTo( key.getProfile() ) )
    {
        //Clear the incoming list
        out_intersectingKeys.clear();
        out_intersectingKeys.push_back(key);
    }
    else
    {
        // figure out which LOD in the local profile is a best match for the LOD
        // in the source LOD in terms of resolution.
        unsigned localLOD = getEquivalentLOD(key.getProfile(), key.getLOD());
        getIntersectingTiles(key.getExtent(), localLOD, out_intersectingKeys);

        OE_DEBUG << LC << "GIT, key="<< key.str() << ", localLOD=" << localLOD
            << ", resulted in " << out_intersectingKeys.size() << " tiles" << std::endl;
    }
}

void
Profile::getIntersectingTiles(const GeoExtent& extent, unsigned localLOD, std::vector<TileKey>& out_intersectingKeys) const
{
    GeoExtent ext = extent;

    // reproject into the profile's SRS if necessary:
    if ( !getSRS()->isHorizEquivalentTo( extent.getSRS() ) )
    {
        // localize the extents and clamp them to legal values
        ext = clampAndTransformExtent( extent );
        if ( !ext.isValid() )
            return;
    }

    if ( ext.crossesAntimeridian() )
    {
        GeoExtent first, second;
        if (ext.splitAcrossAntimeridian( first, second ))
        {
            addIntersectingTiles( first, localLOD, out_intersectingKeys );
            addIntersectingTiles( second, localLOD, out_intersectingKeys );
        }
    }
    else
    {
        addIntersectingTiles( ext, localLOD, out_intersectingKeys );
    }
}

unsigned
Profile::getEquivalentLOD( const Profile* rhsProfile, unsigned rhsLOD ) const
{    
    //If the profiles are equivalent, just use the incoming lod
    if (rhsProfile->isHorizEquivalentTo( this ) ) 
        return rhsLOD;

    // Special check for geodetic to mercator or vise versa, they should match up in LOD.
    if ((rhsProfile->isEquivalentTo(Registry::instance()->getSphericalMercatorProfile()) && isEquivalentTo(Registry::instance()->getGlobalGeodeticProfile())) ||
        (rhsProfile->isEquivalentTo(Registry::instance()->getGlobalGeodeticProfile()) && isEquivalentTo(Registry::instance()->getSphericalMercatorProfile())))
    {
        return rhsLOD;
    }

    double rhsWidth, rhsHeight;
    rhsProfile->getTileDimensions( rhsLOD, rhsWidth, rhsHeight );    

    // safety catch
    if ( osg::equivalent(rhsWidth, 0.0) || osg::equivalent(rhsHeight, 0.0) )
    {
        OE_WARN << LC << "getEquivalentLOD: zero dimension" << std::endl;
        return rhsLOD;
    }

    const SpatialReference* rhsSRS = rhsProfile->getSRS();
    double rhsTargetHeight = rhsSRS->transformUnits( rhsHeight, getSRS() );    
    
    int currLOD = 0;
    int destLOD = currLOD;

    double delta = DBL_MAX;

    // Find the LOD that most closely matches the resolution of the incoming key.
    // We use the closest (under or over) so that you can match back and forth between profiles and be sure to get the same results each time.
    while( true )
    {
        double prevDelta = delta;
        
        double w, h;
        getTileDimensions(currLOD, w, h);

        delta = osg::absolute( h - rhsTargetHeight );
        if (delta < prevDelta)
        {
            // We're getting closer so keep going
            destLOD = currLOD;
        }
        else
        {
            // We are further away from the previous lod so stop.
            break;
        }        
        currLOD++;
    }
    return destLOD;
}
