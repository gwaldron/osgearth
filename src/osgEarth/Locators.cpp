/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/MapInfo>
#include <osg/Notify>

using namespace osgEarth;

GeoLocator::GeoLocator() :
_inverseCalculated(false),
_x0(0.0), _x1(1.0),
_y0(0.0), _y1(1.0)
{
    this->setThreadSafeRefUnref(true);
}

GeoLocator::GeoLocator( const GeoExtent& dataExtent ) :
_inverseCalculated(false),
_dataExtent( dataExtent ),
_x0(0.0), _x1(1.0),
_y0(0.0), _y1(1.0)
{
    this->setThreadSafeRefUnref(true);
}

GeoLocator::GeoLocator( const osgTerrain::Locator& prototype, const GeoExtent& dataExtent ) :
osgTerrain::Locator( prototype ),
_inverseCalculated(false),
_dataExtent( dataExtent ),
_x0(0.0), _x1(1.0),
_y0(0.0), _y1(1.0)
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

bool
GeoLocator::isEquivalentTo( const GeoLocator& rhs ) const
{
    return
        _transform == rhs._transform &&
        _coordinateSystemType == rhs._coordinateSystemType &&
        _cs == rhs._cs;
}

GeoLocator*
GeoLocator::createForKey( const TileKey& key, const MapInfo& map )
{    
    const GeoExtent& ex = key.getExtent();
    return createForExtent( ex, map );    
}

GeoLocator* 
GeoLocator::createForExtent( const GeoExtent& extent, const class MapInfo& map)
{
    double xmin, ymin, xmax, ymax;
    extent.getBounds( xmin, ymin, xmax, ymax );

    // A locator will place the tile on the globe:
    GeoLocator* locator = extent.getSRS()->createLocator(
        extent.xMin(), extent.yMin(), extent.xMax(), extent.yMax(),
        map.isPlateCarre() );

    if ( map.isGeocentric() )
        locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

    return locator;
}

GeoLocator*
GeoLocator::createSameTypeForKey(const TileKey& key, const MapInfo& map)
{
    return createSameTypeForExtent( key.getExtent(), map );
}

GeoLocator*
GeoLocator::createSameTypeForExtent(const GeoExtent& extent, const MapInfo& map)
{
    return createForExtent( extent, map );
}

void
GeoLocator::setDataExtent( const GeoExtent& value ) {
    _dataExtent = value;
}

const GeoExtent&
GeoLocator::getDataExtent() const {
    return _dataExtent;
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

    //cropLocal( local );

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
GeoLocator::getGeographicFromGeocentric( ) const
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

bool
GeoLocator::createScaleBiasMatrix(const GeoExtent& window, osg::Matrixd& out) const
{
    double scalex = window.width() / _dataExtent.width();
    double scaley = window.height() / _dataExtent.height();
    double biasx  = (window.xMin()-_dataExtent.xMin()) / _dataExtent.width();
    double biasy  = (window.yMin()-_dataExtent.yMin()) / _dataExtent.height();

    out(0,0) = scalex;
    out(1,1) = scaley;
    out(3,0) = biasx;
    out(3,1) = biasy;

    return true;
}

/****************************************************************************/


#define MERC_MAX_LAT  85.084059050110383
#define MERC_MIN_LAT -85.084059050110383

namespace
{
    double lonToU(double lon)
    {
        return (lon + 180.0) / 360.0;
    }

    double latToV(double lat)
    {
        double sin_lat = sin( osg::DegreesToRadians( lat ) );
        return 0.5 - log( (1+sin_lat) / (1-sin_lat) ) / (4*osg::PI);
    }

    void getUV(const GeoExtent& ext,
               double lon, double lat,
               double& out_u, double& out_v)
    {
        out_u = osg::clampBetween( (lon-ext.xMin())/ext.width(), MERC_MINX, MERC_MAXX );

        double vmin = latToV( osg::clampBetween( ext.yMax(), MERC_MIN_LAT, MERC_MAX_LAT ) );
        double vmax = latToV( osg::clampBetween( ext.yMin(), MERC_MIN_LAT, MERC_MAX_LAT ) );
        double vlat = latToV( osg::clampBetween( lat, MERC_MIN_LAT, MERC_MAX_LAT ) );

        out_v = osg::clampBetween( (vlat-vmin)/(vmax-vmin), MERC_MINY, MERC_MAXY );
    }
}

MercatorLocator::MercatorLocator( const GeoExtent& dataExtent ) :
GeoLocator( dataExtent )
{
    postInit();
}


MercatorLocator::MercatorLocator(const osgTerrain::Locator& prototype,
                                 const GeoExtent& dataExtent ) :
GeoLocator( prototype, dataExtent )
{
    postInit();
}


void
MercatorLocator::postInit()
{
    // assumption: incoming extent is Mercator SRS; transform it to LAT/LONG
    _geoDataExtent = getDataExtent().transform( getDataExtent().getSRS()->getGeographicSRS() );

    setEllipsoidModel( const_cast<osg::EllipsoidModel*>(_geoDataExtent.getSRS()->getEllipsoid()) );
    setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

    double minX = _geoDataExtent.xMin(), maxX = _geoDataExtent.xMax();
    double minY = _geoDataExtent.yMin(), maxY = _geoDataExtent.yMax();

    osg::Matrixd transform;
    transform.set(
        maxX-minX, 0.0,       0.0, 0.0,
        0.0,       maxY-minY, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        minX,      minY,      0.0, 1.0); 

    setTransform(transform);
}


GeoLocator*
MercatorLocator::createSameTypeForExtent(const GeoExtent& extent, const MapInfo& map)
{
    return new MercatorLocator(extent);
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
MercatorLocator::getGeographicFromGeocentric() const
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

