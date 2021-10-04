/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/Math>

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
 ProfileOptions::mergeConfig( const Config& conf ) {
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

void
ProfileOptions::fromConfig( const Config& conf )
{
    if ( !conf.value().empty() )
        _namedProfile = conf.value();

    conf.get( "srs", _srsInitString );

    conf.get( "vdatum", _vsrsInitString );
    conf.get( "vsrs", _vsrsInitString ); // back compat

    if ( conf.hasValue( "xmin" ) && conf.hasValue( "ymin" ) && conf.hasValue( "xmax" ) && conf.hasValue( "ymax" ) )
    {
        _bounds = Bounds(
            conf.value<double>( "xmin", 0 ),
            conf.value<double>( "ymin", 0 ),
            conf.value<double>( "xmax", 0 ),
            conf.value<double>( "ymax", 0 ) );
    }

    conf.get( "num_tiles_wide_at_lod_0", _numTilesWideAtLod0 );
    conf.get( "tx", _numTilesWideAtLod0 );
    conf.get( "num_tiles_high_at_lod_0", _numTilesHighAtLod0 );
    conf.get( "ty", _numTilesHighAtLod0 );
}

Config
ProfileOptions::getConfig() const
{
    Config conf( "profile" );
    if ( _namedProfile.isSet() )
    {
        conf.setValue(_namedProfile.value());
    }
    else
    {
        conf.set( "srs", _srsInitString );
        conf.set( "vdatum", _vsrsInitString );

        if ( _bounds.isSet() )
        {
            conf.set( "xmin", toString(_bounds->xMin()) );
            conf.set( "ymin", toString(_bounds->yMin()) );
            conf.set( "xmax", toString(_bounds->xMax()) );
            conf.set( "ymax", toString(_bounds->yMax()) );
        }

        conf.set( "num_tiles_wide_at_lod_0", _numTilesWideAtLod0 );
        conf.set( "num_tiles_high_at_lod_0", _numTilesHighAtLod0 );
    }
    return conf;
}

bool
ProfileOptions::defined() const
{
    return _namedProfile.isSet() || _srsInitString.isSet();
}

/***********************************************************************/

std::string Profile::GLOBAL_GEODETIC("global-geodetic");
std::string Profile::GLOBAL_MERCATOR("global-mercator");
std::string Profile::SPHERICAL_MERCATOR("spherical-mercator");
std::string Profile::PLATE_CARREE("plate-carree");

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
    OE_SOFT_ASSERT_AND_RETURN(srs!=nullptr, nullptr);

    return new Profile(
        srs,
        xmin, ymin, xmax, ymax,
        numTilesWideAtLod0,
        numTilesHighAtLod0 );
}

const Profile*
Profile::create(const SpatialReference* srs)
{
    OE_SOFT_ASSERT_AND_RETURN(srs != nullptr, nullptr);

    Bounds bounds;
    if (srs->getBounds(bounds))
    {
        unsigned x = 1, y = 1;
        float ar = (float)bounds.width() / (float)bounds.height();
        if (ar > 1.5f)
        {
            x = (unsigned)::ceil(ar);
        }
        else if (ar < 0.5f)
        {
            y = (unsigned)::ceil(1.0f / ar);
        }

        return new Profile(
            srs,
            bounds.xMin(), bounds.yMin(), bounds.xMax(), bounds.yMax(),
            x, y);
    }
    else
    {
        return create(
            srs->getHorizInitString(),
            srs->getVertInitString(),
            0, 0);
    }
}

const Profile*
Profile::create(const SpatialReference* srs,
                double xmin, double ymin, double xmax, double ymax,
                double geoxmin, double geoymin, double geoxmax, double geoymax,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    OE_SOFT_ASSERT_AND_RETURN(srs!=nullptr, nullptr);

    return new Profile(
        srs,
        xmin, ymin, xmax, ymax,
        geoxmin, geoymin, geoxmax, geoymax,
        numTilesWideAtLod0,
        numTilesHighAtLod0);
}

const Profile*
Profile::create(const std::string& srsInitString,
                const std::string& vsrsInitString,
                unsigned int numTilesWideAtLod0,
                unsigned int numTilesHighAtLod0)
{
    osg::ref_ptr<const SpatialReference> srs = SpatialReference::create( 
        srsInitString, 
        vsrsInitString );

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
        OE_INFO << LC << "No extents given, making some up.\n";
        Bounds bounds;
        if (srs->getBounds(bounds))
        {
            if (numTilesWideAtLod0 == 0 || numTilesHighAtLod0 == 0)
            {
                double ar = (bounds.width() / bounds.height());
                if (ar >= 1.0) {
                    int ari = (int)ar;
                    numTilesHighAtLod0 = 1;
                    numTilesWideAtLod0 = ari;
                }
                else {
                    int ari = (int)(1.0/ar);
                    numTilesWideAtLod0 = 1;
                    numTilesHighAtLod0 = ari;
                }
            }            
            return Profile::create(srs.get(), bounds.xMin(), bounds.yMin(), bounds.xMax(), bounds.yMax(), numTilesWideAtLod0, numTilesHighAtLod0);
        }
        else
        {
            OE_WARN << LC << "Failed to create profile; you must provide extents with a projected SRS." << std::endl;
        }
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
        result = Profile::create(options.namedProfile().get());
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
Profile::create(const std::string& name)
{
    // TODO: move the named profiles from Registry into here.
    if ( ciEquals(name, PLATE_CARREE) || ciEquals(name, "plate-carre") || ciEquals(name, "eqc-wgs84") )
    {
        // Yes I know this is not really Plate Carre but it will stand in for now.
        osg::Vec3d ex;
        const SpatialReference* plateCarre = SpatialReference::get("plate-carre");
        const SpatialReference* wgs84 = SpatialReference::get("wgs84");
        wgs84->transform(osg::Vec3d(180,90,0), plateCarre, ex);

        return Profile::create(plateCarre, -ex.x(), -ex.y(), ex.x(), ex.y(), 2u, 1u);
    }
    else if (ciEquals(name, GLOBAL_GEODETIC))
    {
        return create(
            SpatialReference::create("wgs84"),
            -180.0, -90.0, 180.0, 90.0,
            2, 1);
    }
    else if (ciEquals(name, GLOBAL_MERCATOR))
    {
        return create(
            SpatialReference::create("global-mercator"),
            MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY,
            1, 1);
    }
    else if (ciEquals(name, SPHERICAL_MERCATOR))
    {
        return create(
            SpatialReference::create("spherical-mercator"),
            MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY,
            1, 1);
    }
    else
    {
        return create(name, "");
    }
}

/****************************************************************************/


Profile::Profile(const SpatialReference* srs,
                 double xmin, double ymin, double xmax, double ymax,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0) :

    _extent(srs, xmin, ymin, xmax, ymax)
{
    OE_SOFT_ASSERT(srs!=nullptr);

    _numTilesWideAtLod0 = numTilesWideAtLod0 != 0? numTilesWideAtLod0 : srs->isGeographic()? 2 : 1;
    _numTilesHighAtLod0 = numTilesHighAtLod0 != 0? numTilesHighAtLod0 : 1;

    // automatically calculate the lat/long extents:
    _latlong_extent = srs->isGeographic()?
        _extent :
        _extent.transform( _extent.getSRS()->getGeographicSRS() );

    // make a profile sig (sans srs) and an srs sig for quick comparisons.
    ProfileOptions temp = toProfileOptions();
    std::string fullJSON = temp.getConfig().toJSON();
    _fullSignature =  Stringify() << std::hex << hashString(fullJSON);
    temp.vsrsString() = "";
    _horizSignature = Stringify() << std::hex << hashString( temp.getConfig().toJSON() );

    _hash = std::hash<std::string>()(fullJSON);
}

Profile::Profile(const SpatialReference* srs,
                 double xmin, double ymin, double xmax, double ymax,
                 double geo_xmin, double geo_ymin, double geo_xmax, double geo_ymax,
                 unsigned int numTilesWideAtLod0,
                 unsigned int numTilesHighAtLod0 ) :

    _extent(srs, xmin, ymin, xmax, ymax)
{
    OE_SOFT_ASSERT(srs!=nullptr);

    _numTilesWideAtLod0 = numTilesWideAtLod0 != 0? numTilesWideAtLod0 : srs->isGeographic()? 2 : 1;
    _numTilesHighAtLod0 = numTilesHighAtLod0 != 0? numTilesHighAtLod0 : 1;

    _latlong_extent = GeoExtent( 
        srs->getGeographicSRS(),
        geo_xmin, geo_ymin, geo_xmax, geo_ymax );

    // make a profile sig (sans srs) and an srs sig for quick comparisons.
    ProfileOptions temp = toProfileOptions();
    std::string fullJSON = temp.getConfig().toJSON();
    _fullSignature =  Stringify() << std::hex << hashString(fullJSON);
    temp.vsrsString() = "";
    _horizSignature = Stringify() << std::hex << hashString( temp.getConfig().toJSON() );

    _hash = std::hash<std::string>()(fullJSON);
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
        << " ar=" << _numTilesWideAtLod0 << ":" << _numTilesHighAtLod0
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
Profile::calculateExtent( unsigned int lod, unsigned int tileX, unsigned int tileY ) const
{
    double width, height;
    getTileDimensions(lod, width, height);

    double xmin = getExtent().xMin() + (width * (double)tileX);
    double ymax = getExtent().yMax() - (height * (double)tileY);
    double xmax = xmin + width;
    double ymin = ymax - height;

    return GeoExtent( getSRS(), xmin, ymin, xmax, ymax );
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
    if ( tileSize <= 0 || resolution <= 0.0 ) return 23;

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
Profile::clampAndTransformExtent(const GeoExtent& input, bool* out_clamped) const
{
    // initialize the output flag
    if ( out_clamped )
        *out_clamped = false;

    // null checks
    if (input.isInvalid())
        return GeoExtent::INVALID;

    if (input.isWholeEarth())
    {
        if (out_clamped && !getExtent().isWholeEarth())
            *out_clamped = true;
        return getExtent();
    }

    // begin by transforming the input extent to this profile's SRS.
    GeoExtent inputInMySRS = input.transform(getSRS());

    if (inputInMySRS.isValid())
    {
        // Compute the intersection of the two:
        GeoExtent intersection = inputInMySRS.intersectionSameSRS(getExtent());

        // expose whether clamping took place:
        if (out_clamped != 0L)
        {
            *out_clamped = intersection != getExtent();
        }

        return intersection;
    }

    else
    {
        // The extent transformation failed, probably due to an out-of-bounds condition.
        // Go to Plan B: attempt the operation in lat/long
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

    double west = key_ext.xMin() - _extent.xMin();
    double east = key_ext.xMax() - _extent.xMin();
    double south = _extent.yMax() - key_ext.yMin();
    double north = _extent.yMax() - key_ext.yMax();

    tileMinX = (int)(west / destTileWidth);
    tileMaxX = (int)(east / destTileWidth);

    tileMinY = (int)(north / destTileHeight);
    tileMaxY = (int)(south / destTileHeight);

    // If the east or west border fell right on a tile boundary
    // but doesn't actually use that tile, detect that and eliminate
    // the extranous tiles. (This happens commonly when mapping
    // geodetic to mercator for example)

    double quantized_west = destTileWidth * (double)tileMinX;
    double quantized_east = destTileWidth * (double)(tileMaxX + 1);

    if (osg::equivalent(west - quantized_west, destTileWidth))
        ++tileMinX;
    if (osg::equivalent(quantized_east - east, destTileWidth))
        --tileMaxX;

    if (tileMaxX < tileMinX)
        tileMaxX = tileMinX;

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
    OE_SOFT_ASSERT_AND_RETURN(rhsProfile!=nullptr, rhsLOD);

    //If the profiles are equivalent, just use the incoming lod
    if (rhsProfile->isHorizEquivalentTo( this ) ) 
        return rhsLOD;

    static osg::ref_ptr<const Profile> ggProfile = Profile::create(Profile::GLOBAL_GEODETIC);
    static osg::ref_ptr<const Profile> smProfile = Profile::create(Profile::SPHERICAL_MERCATOR);

    // Special check for geodetic to mercator or vise versa, they should match up in LOD.
    // TODO not sure about this.. -gw
    if ((rhsProfile->isEquivalentTo(smProfile.get()) && isEquivalentTo(ggProfile.get())) ||
        (rhsProfile->isEquivalentTo(ggProfile.get()) && isEquivalentTo(smProfile.get())))
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

unsigned
Profile::getLOD(double height) const
{
    int currLOD = 0;
    int destLOD = currLOD;

    double delta = DBL_MAX;

    // Find the LOD that most closely matches the target height in this profile.
    while (true)
    {
        double prevDelta = delta;

        double w, h;
        getTileDimensions(currLOD, w, h);

        delta = osg::absolute(h - height);
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
