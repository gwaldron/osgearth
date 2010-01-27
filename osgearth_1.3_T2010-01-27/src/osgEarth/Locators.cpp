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
#include <osgEarth/Locators>
#include <osgEarth/TileKey>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osg/Notify>

using namespace osgEarth;

GeoLocator::GeoLocator() :
_x0(0.0), _x1(1.0),
_y0(0.0), _y1(1.0),
_inverseCalculated(false)
{
    this->setThreadSafeRefUnref(true);
}

GeoLocator::GeoLocator( const GeoExtent& dataExtent ) :
_dataExtent( dataExtent ),
_x0(0.0), _x1(1.0),
_y0(0.0), _y1(1.0),
_inverseCalculated(false)
{
    this->setThreadSafeRefUnref(true);
}

GeoLocator::GeoLocator( const osgTerrain::Locator& prototype, const GeoExtent& dataExtent ) :
osgTerrain::Locator( prototype ),
_dataExtent( dataExtent ),
_x0(0.0), _x1(1.0),
_y0(0.0), _y1(1.0),
_inverseCalculated(false)
{
    //nop
}

GeoLocator::GeoLocator( const osgTerrain::Locator& prototype, const GeoExtent& dataExtent, const GeoExtent& displayExtent ) :
osgTerrain::Locator( prototype ),
_dataExtent( dataExtent ),
_inverseCalculated(false)
{
    // assume they are the same SRS
    _x0 = osg::clampBetween( (displayExtent.xMin()-dataExtent.xMin())/dataExtent.width(), 0.0, 1.0 );
    _x1 = osg::clampBetween( (displayExtent.xMax()-dataExtent.xMin())/dataExtent.width(), 0.0, 1.0 );
    _y0 = osg::clampBetween( (displayExtent.yMin()-dataExtent.yMin())/dataExtent.height(), 0.0, 1.0 );
    _y1 = osg::clampBetween( (displayExtent.yMax()-dataExtent.yMin())/dataExtent.height(), 0.0, 1.0 );
}

GeoLocator*
GeoLocator::createForKey( const TileKey* key, Map* map )
{
    bool isProjected = map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED;
    bool isPlateCarre = isProjected && map->getProfile()->getSRS()->isGeographic();
    bool isGeocentric = !isProjected;

    const GeoExtent& ex = key->getGeoExtent();
    double xmin, ymin, xmax, ymax;
    key->getGeoExtent().getBounds( xmin, ymin, xmax, ymax );

    // A locator will place the tile on the globe:
    GeoLocator* locator = key->getProfile()->getSRS()->createLocator(
        ex.xMin(), ex.yMin(), ex.xMax(), ex.yMax(),
        isPlateCarre );

    if ( isGeocentric )
        locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

    return locator;
}

void
GeoLocator::setDataExtent( const GeoExtent& value ) {
    _dataExtent = value;
}

const GeoExtent&
GeoLocator::getDataExtent() const {
    return _dataExtent;
}

GeoLocator*
GeoLocator::cloneAndCrop( const osgTerrain::Locator& prototype, const GeoExtent& displayExtent )
{
    return new GeoLocator( prototype, _dataExtent, displayExtent );
}


bool
GeoLocator::convertModelToLocal(const osg::Vec3d& world, osg::Vec3d& local) const
{
    // required becasue of an OSG bug
    if ( !_inverseCalculated )
    {
        const_cast<GeoLocator*>(this)->_inverse.invert( _transform );
        const_cast<GeoLocator*>(this)->_inverseCalculated = true;
    }

    bool ok = Locator::convertModelToLocal( world, local );

    cropLocal( local );

    return ok;
}

void
GeoLocator::cropLocal( osg::Vec3d& local ) const
{
    // crop if necessary:
    local.x() = osg::clampBetween( _x0 + local.x()*(_x1 - _x0), 0.0, 1.0 );
    local.y() = osg::clampBetween( _y0 + local.y()*(_y1 - _y0), 0.0, 1.0 );
}

GeoLocator* 
GeoLocator::getGeographicFromGeocentric( )
{
    if (getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC)
    {
        double xmin, ymin, xmax, ymax;
        getDataExtent().getBounds( xmin, ymin,  xmax, ymax );
        GeoLocator* geographic = new GeoLocator( getDataExtent() );
        geographic->setCoordinateSystemType( Locator::GEOGRAPHIC );
        geographic->setTransformAsExtents( xmin, ymin, xmax, ymax);
        return geographic;
    }
    return NULL;
}


/****************************************************************************/


#define MERC_MAX_LAT  85.084059050110383
#define MERC_MIN_LAT -85.084059050110383

static double
lonToU(double lon) {
    return (lon + 180.0) / 360.0;
}

static double
latToV(double lat) {
    double sin_lat = sin( osg::DegreesToRadians( lat ) );
    return 0.5 - log( (1+sin_lat) / (1-sin_lat) ) / (4*osg::PI);
}

static void
getUV(const GeoExtent& ext,
      double lon, double lat,
      double& out_u, double& out_v)
{
    out_u = (lon-ext.xMin())/ext.width();

    double vmin = latToV( osg::clampBetween( ext.yMax(), MERC_MIN_LAT, MERC_MAX_LAT ) );
    double vmax = latToV( osg::clampBetween( ext.yMin(), MERC_MIN_LAT, MERC_MAX_LAT ) );
    double vlat = latToV( osg::clampBetween( lat, MERC_MIN_LAT, MERC_MAX_LAT ) );

    out_v = (vlat-vmin)/(vmax-vmin);
}

//static void
//mercatorToLatLon( double x, double y, double& out_lat, double& out_lon )
//{
//    const GeoExtent& m = osgEarth::Registry::instance()->getGlobalMercatorProfile()->getExtent();
//    double xr = -osg::PI + ((x-m.xMin())/m.width())*2.0*osg::PI;
//    double yr = -osg::PI + ((y-m.yMin())/m.height())*2.0*osg::PI;
//    out_lat = osg::RadiansToDegrees( 2.0 * atan( exp(yr) ) - osg::PI_2 );
//    out_lon = osg::RadiansToDegrees( xr );
//}

MercatorLocator::MercatorLocator( const osgTerrain::Locator& prototype, const GeoExtent& dataExtent ) :
GeoLocator( prototype, dataExtent )
{
    // assumption: incoming extent is Mercator SRS; transform it to LAT/LONG

    _geoDataExtent = dataExtent.transform( dataExtent.getSRS()->getGeographicSRS() );

    //// manually reproject for speed:
    //double latmin, lonmin, latmax, lonmax;
    //mercatorToLatLon( dataExtent.xMin(), dataExtent.yMin(), latmin, lonmin );
    //mercatorToLatLon( dataExtent.xMax(), dataExtent.yMax(), latmax, lonmax );

    //_geoDataExtent = GeoExtent(
    //    dataExtent.getSRS()->getGeographicSRS(),
    //    lonmin, latmin, lonmax, latmax );
}

GeoLocator*
MercatorLocator::cloneAndCrop( const osgTerrain::Locator& prototype, const GeoExtent& displayExtent )
{
    return new MercatorLocator( prototype, getDataExtent() );
}

bool
MercatorLocator::convertModelToLocal(const osg::Vec3d& world, osg::Vec3d& local) const
{
    bool result = false;

    // required becasue of an OSG bug
    if ( !_inverseCalculated )
    {
        const_cast<MercatorLocator*>(this)->_inverse.invert( _transform );
        const_cast<MercatorLocator*>(this)->_inverseCalculated = true;
    }

    switch(_coordinateSystemType)
    {
    case(GEOCENTRIC):
        {
            double longitude, latitude, height;

            _ellipsoidModel->convertXYZToLatLongHeight(world.x(), world.y(), world.z(),
                latitude, longitude, height );

            local = osg::Vec3d(longitude, latitude, height) * _inverse;

            double lon_deg = osg::RadiansToDegrees(longitude);
            double lat_deg = osg::RadiansToDegrees(latitude);
            double xr, yr;

            getUV( _geoDataExtent, lon_deg, lat_deg, xr, yr );

            local.x() = xr;
            local.y() = 1.0-yr;
            result = true;
        }
        break;


    case(GEOGRAPHIC):
        {        
            local = world * _inverse;

            osg::Vec3d w = world;
            double lon_deg = w.x();
            double lat_deg = w.y();

            double xr, yr;
            getUV( _geoDataExtent, lon_deg, lat_deg, xr, yr );

            local.x() = xr;
            local.y() = 1.0-yr;
            result = true;
        }
        break;

    case(PROJECTED):
        {        
            local = world * _inverse;
            result = true;      
        }
        break;
    }

    return result;
}

GeoLocator* 
MercatorLocator::getGeographicFromGeocentric( )
{
    if (getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC)
    {
        double xmin, ymin, xmax, ymax;
        getDataExtent().getBounds( xmin, ymin, xmax, ymax );
        MercatorLocator* geographic = new MercatorLocator( *this, getDataExtent() );
        geographic->setCoordinateSystemType( Locator::GEOGRAPHIC );
        geographic->setTransformAsExtents( xmin, ymin, xmax, ymax);
        return geographic;
    }
    return NULL;
}

