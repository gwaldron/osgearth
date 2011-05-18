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

#include <osgEarth/SpatialReference>
#include <osgEarth/Registry>
#include <osgEarth/Cube>
#include <OpenThreads/ScopedLock>
#include <osg/Notify>
#include <ogr_api.h>
#include <ogr_spatialref.h>
#include <algorithm>

#define LC "[SpatialReference] "

using namespace osgEarth;

#define USE_CUSTOM_MERCATOR_TRANSFORM 1
//#undef USE_CUSTOM_MERCATOR_TRANSFORM

//------------------------------------------------------------------------

namespace
{
    std::string
    getOGRAttrValue( void* _handle, const std::string& name, int child_num, bool lowercase =false)
    {
        GDAL_SCOPED_LOCK;
	    const char* val = OSRGetAttrValue( _handle, name.c_str(), child_num );
        if ( val )
        {
            std::string t = val;
            if ( lowercase )
            {
                std::transform( t.begin(), t.end(), t.begin(), ::tolower );
            }
            return t;
        }
        return "";
    }

    std::string&
    replaceIn( std::string& s, const std::string& sub, const std::string& other)
    {
        if ( sub.empty() ) return s;
        size_t b=0;
        for( ; ; )
        {
            b = s.find( sub, b );
            if ( b == s.npos ) break;
            s.replace( b, sub.size(), other );
            b += other.size();
        }
        return s;
    }
}

//------------------------------------------------------------------------

SpatialReference::SpatialReferenceCache& SpatialReference::getSpatialReferenceCache()
{
    //Make sure the registry is created before the cache
    osgEarth::Registry::instance();
    static SpatialReferenceCache s_cache;
    return s_cache;
}


SpatialReference*
SpatialReference::createFromPROJ4( const std::string& init, const std::string& init_alias, const std::string& name )
{
    SpatialReference* result = NULL;
    GDAL_SCOPED_LOCK;
	void* handle = OSRNewSpatialReference( NULL );
    if ( OSRImportFromProj4( handle, init.c_str() ) == OGRERR_NONE )
	{
        result = new SpatialReference( handle, "PROJ4", init_alias, name );
	}
	else 
	{
        OE_WARN << LC << "Unable to create spatial reference from PROJ4: " << init << std::endl;
		OSRDestroySpatialReference( handle );
	}
    return result;
}

SpatialReference*
SpatialReference::createCube()
{
    // root the cube srs with a WGS84 intermediate ellipsoid.
    std::string init = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";

    SpatialReference* result = NULL;
    GDAL_SCOPED_LOCK;
	void* handle = OSRNewSpatialReference( NULL );
    if ( OSRImportFromProj4( handle, init.c_str() ) == OGRERR_NONE )
	{
        result = new CubeSpatialReference( handle );
	}
	else 
	{
        OE_WARN << LC << "Unable to create SRS: " << init << std::endl;
		OSRDestroySpatialReference( handle );
	}
    return result;
}


SpatialReference*
SpatialReference::createFromWKT( const std::string& init, const std::string& init_alias, const std::string& name )
{
    osg::ref_ptr<SpatialReference> result;
    GDAL_SCOPED_LOCK;
	void* handle = OSRNewSpatialReference( NULL );
    char buf[4096];
    char* buf_ptr = &buf[0];
	strcpy( buf, init.c_str() );
	if ( OSRImportFromWkt( handle, &buf_ptr ) == OGRERR_NONE )
	{
        result = new SpatialReference( handle, "WKT", init_alias, name );
        result = result->validate();
	}
	else 
	{
		OE_WARN << LC << "Unable to create spatial reference from WKT: " << init << std::endl;
		OSRDestroySpatialReference( handle );
	}
    return result.release();
}

SpatialReference*
SpatialReference::create( const std::string& init )
{
    static OpenThreads::Mutex s_mutex;
    OpenThreads::ScopedLock<OpenThreads::Mutex> exclusiveLock(s_mutex);

    std::string low = init;
    std::transform( low.begin(), low.end(), low.begin(), ::tolower );

    SpatialReferenceCache::iterator itr = getSpatialReferenceCache().find(init);
    if (itr != getSpatialReferenceCache().end())
    {
        //OE_NOTICE << "Returning cached SRS" << std::endl;
        return itr->second.get();
    }

    osg::ref_ptr<SpatialReference> srs;

    // shortcut for spherical-mercator:
    if (low == "spherical-mercator" || low == "epsg:900913" || low == "epsg:3785" ||
        low == "epsg:41001" || low == "epsg:102113" || low == "epsg:102100")
    {
        // note the use of nadgrids=@null (see http://proj.maptools.org/faq.html)
        // adjusted +a by ONE to work around osg manipulator error until we can figure out why.. GW
        srs = createFromPROJ4(
            "+proj=merc +a=6378137 +b=6378137 +lon_0=0 +k=1 +x_0=0 +y_0=0 +nadgrids=@null +units=m +no_defs",
            init,
            "Spherical Mercator" );
    }

    // ellipsoidal ("world") mercator:
    else if (low == "epsg:54004" || low == "epsg:9804" || low == "epsg:3832")
    {
        srs = createFromPROJ4(
            "+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs",
            init,
            "World Mercator" );
    }

    // common WGS84:
    else if (low == "epsg:4326" || low == "wgs84")
    {
        srs = createFromPROJ4(
            "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs",
            init,
            "WGS84" );
    }

    // custom srs for the unified cube
    else if ( low == "unified-cube" )
    {
        srs = createCube();
    }

    else if ( low.find( "+" ) == 0 )
    {
        srs = createFromPROJ4( low, init );
    }
    else if ( low.find( "epsg:" ) == 0 || low.find( "osgeo:" ) == 0 )
    {
        srs = createFromPROJ4( std::string("+init=") + low, init );
    }
    else if ( low.find( "projcs" ) == 0 || low.find( "geogcs" ) == 0 )
    {
        srs = createFromWKT( init, init );
    }
    else
    {
        return NULL;
    }

    getSpatialReferenceCache()[init] = srs;
    return srs.get();
}


SpatialReference*
SpatialReference::create( osg::CoordinateSystemNode* csn )
{
    SpatialReference* result = NULL;
    if ( csn && !csn->getCoordinateSystem().empty() )
    {
        result = create( csn->getCoordinateSystem() );
    }
    return result;
}


SpatialReference*
SpatialReference::createFromHandle( void* ogrHandle, bool xferOwnership )
{
    SpatialReference* srs = new SpatialReference( ogrHandle, xferOwnership );
    return srs;
}

SpatialReference*
SpatialReference::validate()
{
    std::string proj = getOGRAttrValue( _handle, "PROJECTION", 0 );

    // fix invalid ESRI LCC projections:
    if ( proj == "Lambert_Conformal_Conic" )
    {
        bool has_2_sps =
            !getOGRAttrValue( _handle, "Standard_Parallel_2", 0 ).empty() ||
            !getOGRAttrValue( _handle, "standard_parallel_2", 0 ).empty();

        std::string new_wkt = getWKT();
        if ( has_2_sps )
            replaceIn( new_wkt, "Lambert_Conformal_Conic", "Lambert_Conformal_Conic_2SP" );
        else
            replaceIn( new_wkt, "Lambert_Conformal_Conic", "Lambert_Conformal_Conic_1SP" );

        OE_INFO << LC << "Morphing Lambert_Conformal_Conic to 1SP/2SP" << std::endl;
        
        return createFromWKT( new_wkt, _init_str, _name );
    }

    // fixes for ESRI Plate_Carree and Equidistant_Cylindrical projections:
    else if ( proj == "Plate_Carree" )
    {
        std::string new_wkt = getWKT();
        replaceIn( new_wkt, "Plate_Carree", "Equirectangular" );
        OE_INFO << LC << "Morphing Plate_Carree to Equirectangular" << std::endl;
        return createFromWKT( new_wkt, _init_str, _name ); //, input->getReferenceFrame() );
    }
    else if ( proj == "Equidistant_Cylindrical" )
    {
        std::string new_wkt = getWKT();
        OE_INFO << LC << "Morphing Equidistant_Cylindrical to Equirectangular" << std::endl;
        replaceIn( new_wkt, "Equidistant_Cylindrical", "Equirectangular" );
        return createFromWKT( new_wkt, _init_str, _name );
    }

    // no changes.
    return this;
}


/****************************************************************************/


SpatialReference::SpatialReference(void* handle, 
                                   const std::string& init_type,
                                   const std::string& init_str,
                                   const std::string& name ) :
osg::Referenced( true ),
_initialized( false ),
_handle( handle ),
_owns_handle( true ),
_name( name ),
_init_type( init_type ),
_init_str( init_str )
{
    _init_str_lc = init_str;
    std::transform( _init_str_lc.begin(), _init_str_lc.end(), _init_str_lc.begin(), ::tolower );
}

SpatialReference::SpatialReference(void* handle, bool ownsHandle) :
osg::Referenced( true ),
_initialized( false ),
_handle( handle ),
_owns_handle( ownsHandle )
{
    //nop
}

SpatialReference::~SpatialReference()
{
    if ( _handle )
    {
        GDAL_SCOPED_LOCK;

        for (TransformHandleCache::iterator itr = _transformHandleCache.begin(); itr != _transformHandleCache.end(); ++itr)
        {
            OCTDestroyCoordinateTransformation(itr->second);
        }

        if ( _owns_handle )
        {
            OSRDestroySpatialReference( _handle );
        }

        _handle = NULL;
    }
}

bool
SpatialReference::isGeographic() const 
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _is_geographic;
}

bool
SpatialReference::isProjected() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return !_is_geographic;
}

const std::string&
SpatialReference::getName() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _name;
}

const osg::EllipsoidModel*
SpatialReference::getEllipsoid() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _ellipsoid.get();
}

const std::string&
SpatialReference::getWKT() const 
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _wkt;
}

const std::string&
SpatialReference::getInitString() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _init_str;
}

const std::string&
SpatialReference::getInitType() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _init_type;
}

bool
SpatialReference::isEquivalentTo( const SpatialReference* rhs ) const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    return _isEquivalentTo( rhs );
}

bool
SpatialReference::_isEquivalentTo( const SpatialReference* rhs ) const
{
    if ( !rhs )
        return false;

    if ( this == rhs )
        return true;

    if (isGeographic()  != rhs->isGeographic()  ||
        isMercator()    != rhs->isMercator()    ||
        isNorthPolar()  != rhs->isNorthPolar()  ||
        isSouthPolar()  != rhs->isSouthPolar()  ||
        isContiguous()  != rhs->isContiguous()  ||
        isUserDefined() != rhs->isUserDefined() ||
        isCube()        != rhs->isCube() )
    {
        return false;
    }

    if ( _init_str_lc == rhs->_init_str_lc )
        return true;

    if ( this->getWKT() == rhs->getWKT() )
        return true;

    if (this->isGeographic() && rhs->isGeographic() &&
        this->getEllipsoid()->getRadiusEquator() == rhs->getEllipsoid()->getRadiusEquator() &&
        this->getEllipsoid()->getRadiusPolar() == rhs->getEllipsoid()->getRadiusPolar())
    {
        return true;
    }

    // last resort, since it requires the lock
    GDAL_SCOPED_LOCK;
    return TRUE == ::OSRIsSame( _handle, rhs->_handle );
}

const SpatialReference*
SpatialReference::getGeographicSRS() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    if ( _is_geographic )
        return this;

    if ( !_geo_srs.valid() )
    {
        GDAL_SCOPED_LOCK;

        if ( !_geo_srs.valid() ) // double-check pattern
        {
            void* new_handle = OSRNewSpatialReference( NULL );
            int err = OSRCopyGeogCSFrom( new_handle, _handle );
            if ( err == OGRERR_NONE )
            {
                const_cast<SpatialReference*>(this)->_geo_srs = new SpatialReference( new_handle );
            }
            else
            {
                OSRDestroySpatialReference( new_handle );
            }
        }
    }

    return _geo_srs.get();
}

bool
SpatialReference::isMercator() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _is_mercator;
}

bool 
SpatialReference::isNorthPolar() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _is_north_polar;
}

bool 
SpatialReference::isSouthPolar() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _is_south_polar;
}

bool
SpatialReference::isContiguous() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _is_contiguous;
}

bool
SpatialReference::isUserDefined() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _is_user_defined;
}

bool
SpatialReference::isCube() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _is_cube;
}

osg::CoordinateSystemNode*
SpatialReference::createCoordinateSystemNode() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();
    populateCoordinateSystemNode( csn );
    return csn;
}

bool
SpatialReference::populateCoordinateSystemNode( osg::CoordinateSystemNode* csn ) const
{
    if ( !csn )
        return false;

    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    if ( !_wkt.empty() )
    {
        csn->setFormat( "WKT" );
        csn->setCoordinateSystem( _wkt );
    }
    else if ( !_proj4.empty() )
    {
        csn->setFormat( "PROJ4" );
        csn->setCoordinateSystem( _proj4 );
    }
    else
    {
        csn->setFormat( _init_type );
        csn->setCoordinateSystem( _init_str );
    }
    
    csn->setEllipsoidModel( _ellipsoid.get() );
    
    return true;
}

// Make a MatrixTransform suitable for use with a Locator object based on the given extents.
// Calling Locator::setTransformAsExtents doesn't work with OSG 2.6 due to the fact that the
// _inverse member isn't updated properly.  Calling Locator::setTransform works correctly.
static osg::Matrixd
getTransformFromExtents(double minX, double minY, double maxX, double maxY)
{
    osg::Matrixd transform;
    transform.set(
        maxX-minX, 0.0,       0.0, 0.0,
        0.0,       maxY-minY, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        minX,      minY,      0.0, 1.0); 
    return transform;
}

GeoLocator*
SpatialReference::createLocator(double xmin, double ymin, double xmax, double ymax,
                                bool plate_carre ) const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    GeoLocator* locator = new GeoLocator( GeoExtent(this, xmin, ymin, xmax, ymax) );
    locator->setEllipsoidModel( (osg::EllipsoidModel*)getEllipsoid() );
    locator->setCoordinateSystemType( isGeographic()? osgTerrain::Locator::GEOGRAPHIC : osgTerrain::Locator::PROJECTED );
    // note: not setting the format/cs on purpose.

    if ( isGeographic() && !plate_carre )
    {
        locator->setTransform( getTransformFromExtents(
            osg::DegreesToRadians( xmin ),
            osg::DegreesToRadians( ymin ),
            osg::DegreesToRadians( xmax ),
            osg::DegreesToRadians( ymax ) ) );
    }
    else
    {
        locator->setTransform( getTransformFromExtents( xmin, ymin, xmax, ymax ) );
    }
    return locator;
}

bool
SpatialReference::transform(double x, double y, 
                            const SpatialReference* out_srs, 
                            double& out_x, double& out_y,
                            void* context ) const
{        
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    //Check for equivalence and return if the coordinate systems are the same.
    if (isEquivalentTo(out_srs))
    {
        out_x = x;
        out_y = y;
        return true;
    }

    GDAL_SCOPED_LOCK;

    preTransform(x, y, context);

    void* xform_handle = NULL;
    TransformHandleCache::const_iterator itr = _transformHandleCache.find(out_srs->getWKT());
    if (itr != _transformHandleCache.end())
    {
        //OE_DEBUG << "SpatialReference: using cached transform handle" << std::endl;
        xform_handle = itr->second;
    }
    else
    {
        xform_handle = OCTNewCoordinateTransformation( _handle, out_srs->_handle);
        const_cast<SpatialReference*>(this)->_transformHandleCache[out_srs->getWKT()] = xform_handle;
    }

    if ( !xform_handle )
    {
        OE_WARN << LC
            << "SRS xform not possible" << std::endl
            << "    From => " << getName() << std::endl
            << "    To   => " << out_srs->getName() << std::endl;
        return false;
    }

    double temp_x = x;
    double temp_y = y;
    double temp_z = 0.0;
    bool result;

    if ( OCTTransform( xform_handle, 1, &temp_x, &temp_y, &temp_z ) )
    {
        result = true;
        out_x = temp_x;
        out_y = temp_y;

        out_srs->postTransform(out_x, out_y, context);
    }
    else
    {
        OE_WARN << LC << "Failed to xform a point from "
            << getName() << " to " << out_srs->getName()
            << std::endl;
        result = false;
    }
    return result;
}

// http://en.wikipedia.org/wiki/Mercator_projection#Mathematics_of_the_projection
static bool
mercatorToGeographic( double* x, double* y, int numPoints )
{
    const GeoExtent& merc = osgEarth::Registry::instance()->getGlobalMercatorProfile()->getExtent();

    for( int i=0; i<numPoints; i++ )
    {
        double xr = -osg::PI + ((x[i]-merc.xMin())/merc.width())*2.0*osg::PI;
        double yr = -osg::PI + ((y[i]-merc.yMin())/merc.height())*2.0*osg::PI;
        x[i] = osg::RadiansToDegrees( xr );
        y[i] = osg::RadiansToDegrees( 2.0 * atan( exp(yr) ) - osg::PI_2 );
    }
    return true;
}

// http://en.wikipedia.org/wiki/Mercator_projection#Mathematics_of_the_projection
static bool
geographicToMercator( double* x, double* y, int numPoints )
{
    const GeoExtent& merc = osgEarth::Registry::instance()->getGlobalMercatorProfile()->getExtent();

    for( int i=0; i<numPoints; i++ )
    {
        double xr = (osg::DegreesToRadians(x[i]) - (-osg::PI)) / (2.0*osg::PI);
        double sinLat = sin(osg::DegreesToRadians(y[i]));
        double oneMinusSinLat = 1-sinLat;
        if ( oneMinusSinLat != 0.0 )
        {
            double yr = ((0.5 * log( (1+sinLat)/oneMinusSinLat )) - (-osg::PI)) / (2.0*osg::PI);
            x[i] = merc.xMin() + (xr * merc.width());
            y[i] = merc.yMin() + (yr * merc.height());
        }
    }
    return true;
}

bool
SpatialReference::transformPoints(const SpatialReference* out_srs,
                                  double* x, double* y,
                                  unsigned int numPoints,
                                  void* context,
                                  bool ignore_errors ) const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    //Check for equivalence and return if the coordinate systems are the same.
    if (isEquivalentTo(out_srs)) return true;

    for (unsigned int i = 0; i < numPoints; ++i)
    {
        preTransform(x[i], y[i], context);
    }
    
    bool success = false;

#ifdef USE_CUSTOM_MERCATOR_TRANSFORM

    if ( isGeographic() && out_srs->isMercator() )
    {
        success = geographicToMercator( x, y, numPoints );
    }

    else if ( isMercator() && out_srs->isGeographic() )
    {
        success = mercatorToGeographic( x, y, numPoints );
    }

    else
#endif

    {    
        GDAL_SCOPED_LOCK;

        void* xform_handle = NULL;
        TransformHandleCache::const_iterator itr = _transformHandleCache.find(out_srs->getWKT());
        if (itr != _transformHandleCache.end())
        {
            //OE_DEBUG << "SpatialRefernece: using cached transform handle" << std::endl;
            xform_handle = itr->second;
        }
        else
        {
            xform_handle = OCTNewCoordinateTransformation( _handle, out_srs->_handle);
            const_cast<SpatialReference*>(this)->_transformHandleCache[out_srs->getWKT()] = xform_handle;
        }

        if ( !xform_handle )
        {
            OE_WARN << LC
                << "SRS xform not possible" << std::endl
                << "    From => " << getName() << std::endl
                << "    To   => " << out_srs->getName() << std::endl;
            return false;
        }

        double* temp_z = new double[numPoints];
        success = OCTTransform( xform_handle, numPoints, x, y, temp_z ) > 0;
        delete[] temp_z;

        // END GDAL_SCOPE_LOCK
    }

    if ( success || ignore_errors )
    {
        for (unsigned int i = 0; i < numPoints; ++i)
        {
            out_srs->postTransform(x[i], y[i], context);
        }
    }
    else
    {
        OE_WARN << LC << "Failed to xform a point from "
            << getName() << " to " << out_srs->getName()
            << std::endl;
    }
    return success;
}

bool
SpatialReference::transformPoints(const SpatialReference* out_srs,
                                  osg::Vec3dArray* points,
                                  void* context,
                                  bool ignore_errors ) const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    //Check for equivalence and return if the coordinate systems are the same.
    if (isEquivalentTo(out_srs)) return true;

    int numPoints = points->size();
    double* x = new double[numPoints];
    double* y = new double[numPoints];

    for( int i=0; i<numPoints; i++ )
    {
        x[i] = (*points)[i].x();
        y[i] = (*points)[i].y();
    }

    bool success = transformPoints( out_srs, x, y, numPoints, context, ignore_errors );

    if ( success )
    {
        for( int i=0; i<numPoints; i++ )
        {
            (*points)[i].x() = x[i];
            (*points)[i].y() = y[i];
        }
    }

    delete[] x;
    delete[] y;

    return success;
}

bool 
SpatialReference::transformToECEF(const osg::Vec3d& input,
                                  osg::Vec3d&       output ) const
{
    double lat = input.y(), lon = input.x();
    //osg::Vec3d geo( x, y, z );
    
    // first convert to lat/long if necessary:
    if ( !isGeographic() )
        transform( input.x(), input.y(), getGeographicSRS(), lon, lat );

    // then convert to ECEF.
    double z = input.z();
    getGeographicSRS()->getEllipsoid()->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( lat ), osg::DegreesToRadians( lon ), z,
        output.x(), output.y(), output.z() );

    return true;
}

bool 
SpatialReference::transformToECEF(osg::Vec3dArray*    points,
                                  bool                ignoreErrors ) const
{
    if ( !points) return false;

    const SpatialReference* geoSRS = getGeographicSRS();
    const osg::EllipsoidModel* ellipsoid = geoSRS->getEllipsoid();

    for( unsigned i=0; i<points->size(); ++i )
    {
        osg::Vec3d& p = (*points)[i];

        if ( !isGeographic() )
            transform( p.x(), p.y(), geoSRS, p.x(), p.y() );

        ellipsoid->convertLatLongHeightToXYZ(
            osg::DegreesToRadians( p.y() ), osg::DegreesToRadians( p.x() ), p.z(),
            p.x(), p.y(), p.z() );
    }

    return true;
}

bool 
SpatialReference::transformFromECEF(const osg::Vec3d& input,
                                    osg::Vec3d&       output ) const
{
    // transform to lat/long:
    osg::Vec3d geo;

    getGeographicSRS()->getEllipsoid()->convertXYZToLatLongHeight(
        input.x(), input.y(), input.z(),
        geo.y(), geo.x(), geo.z() );

    // then convert to the local SRS.
    if ( isGeographic() )
    {
        output.set( osg::RadiansToDegrees(geo.x()), osg::RadiansToDegrees(geo.y()), geo.z() );
    }
    else
    {
        getGeographicSRS()->transform( 
            osg::RadiansToDegrees(geo.x()), osg::RadiansToDegrees(geo.y()),
            this,
            output.x(), output.y() );
        output.z() = geo.z();
    }

    return true;
}

bool 
SpatialReference::transformFromECEF(osg::Vec3dArray* points,
                                    bool             ignoreErrors ) const
{
    bool ok = true;

    // first convert all the points to lat/long (in place):
    for( unsigned i=0; i<points->size(); ++i )
    {
        osg::Vec3d& p = (*points)[i];
        osg::Vec3d geo;
        getGeographicSRS()->getEllipsoid()->convertXYZToLatLongHeight(
            p.x(), p.y(), p.z(),
            geo.y(), geo.x(), geo.z() );
        geo.x() = osg::RadiansToDegrees( geo.x() );
        geo.y() = osg::RadiansToDegrees( geo.y() );
        p = geo;
    }

    // then convert them all to the local SRS if necessary.
    if ( !isGeographic() )
    {
        ok = getGeographicSRS()->transformPoints( this, points, 0L, ignoreErrors );
    }

    return ok;
}

bool
SpatialReference::transformExtent(const SpatialReference* to_srs,
                                  double&                 in_out_xmin,
                                  double&                 in_out_ymin,
                                  double&                 in_out_xmax,
                                  double&                 in_out_ymax,
                                  void*                   context ) const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();

    int oks = 0;

    //Transform all points and take the maximum bounding rectangle the resulting points
    double llx, lly;
    double ulx, uly;
    double urx, ury;
    double lrx, lry;

    //Lower Left
    oks += transform( in_out_xmin, in_out_ymin, to_srs, llx, lly, context ) == true;

    //Upper Left
    oks += transform( in_out_xmin, in_out_ymax, to_srs, ulx, uly, context ) == true;

    //Upper Right
    oks += transform( in_out_xmax, in_out_ymax, to_srs, urx, ury, context ) == true;

    //Lower Right
    oks += transform( in_out_xmax, in_out_ymin, to_srs, lrx, lry, context ) == true;


    if (oks == 4)
    {
        in_out_xmin = osg::minimum(llx, ulx);
        in_out_xmax = osg::maximum(lrx, urx);
        in_out_ymin = osg::minimum(lly, lry);
        in_out_ymax = osg::maximum(uly, ury);
        return true;
    }
    return false;
}

bool SpatialReference::transformExtentPoints(
            const SpatialReference* to_srs,
            double in_xmin, double in_ymin,
            double in_xmax, double in_ymax,
            double* x, double *y,
            unsigned int numx, unsigned int numy,
            void* context, bool ignore_errors ) const
{
    const double dx = (in_xmax - in_xmin) / (numx - 1);
    const double dy = (in_ymax - in_ymin) / (numy - 1);

    unsigned int pixel = 0;
    double fc = 0.0;
    for (unsigned int c = 0; c < numx; ++c, ++fc)
    {
        const double dest_x = in_xmin + fc * dx;
        double fr = 0.0;
        for (unsigned int r = 0; r < numy; ++r, ++fr)
        {
            const double dest_y = in_ymin + fr * dy;

            x[pixel] = dest_x;
            y[pixel] = dest_y;
            pixel++;     
        }
    }
    return transformPoints(to_srs, x, y, numx * numy, context, ignore_errors);
}

void
SpatialReference::init()
{
    GDAL_SCOPED_LOCK;

    // always double-check the _initialized flag after obtaining the lock.
    if ( !_initialized )
    {
        // calls the internal version, which can be overriden by the developer.
        // therefore do not call init() from the constructor!
        _init();
    }
}

void
SpatialReference::_init()
{
    // set defaults:
    _is_user_defined = false; 
    _is_contiguous = true;   
    _is_cube = false;
    _is_geographic = OSRIsGeographic( _handle ) != 0;

    // extract the ellipsoid parameters:
    int err;
    double semi_major_axis = OSRGetSemiMajor( _handle, &err );
    double semi_minor_axis = OSRGetSemiMinor( _handle, &err );
    _ellipsoid = new osg::EllipsoidModel( semi_major_axis, semi_minor_axis );

    // extract the projection:
    if ( _name.empty() || _name == "unnamed" )
    {
        _name = _is_geographic? 
            getOGRAttrValue( _handle, "GEOGCS", 0 ) : 
            getOGRAttrValue( _handle, "PROJCS", 0 );
    }
    std::string proj = getOGRAttrValue( _handle, "PROJECTION", 0, true );

    // check for the Mercator projection:
    _is_mercator = !proj.empty() && proj.find("mercator")==0;

    // check for the Polar projection:
    if ( !proj.empty() && proj.find("polar_stereographic") != std::string::npos )
    {
        double lat = as<double>( getOGRAttrValue( _handle, "latitude_of_origin", 0, true ), -90.0 );
        _is_north_polar = lat > 0.0;
        _is_south_polar = lat < 0.0;
    }
	else
	{
		_is_north_polar = false;
		_is_south_polar = false;
	}

    // Give the SRS a name if it doesn't have one:
    if ( _name == "unnamed" || _name.empty() )
    {
        _name =
            _is_geographic? "Geographic CS" :
            _is_mercator? "Mercator CS" :
            ( !proj.empty()? proj : "Projected CS" );
    }

    // Try to extract the OGC well-known-text (WKT) string:
    char* wktbuf;
    if ( OSRExportToWkt( _handle, &wktbuf ) == OGRERR_NONE )
    {
        _wkt = wktbuf;
        OGRFree( wktbuf );
    }

    // If the user did not specify and initialization string, use the WKT.
    if ( _init_str.empty() )
    {
        _init_str = _wkt;
        _init_type = "WKT";
    }
    
    // Try to extract the PROJ4 initialization string:
    char* proj4buf;
    if ( OSRExportToProj4( _handle, &proj4buf ) == OGRERR_NONE )
    {
        _proj4 = proj4buf;
        OGRFree( proj4buf );
    }

    _initialized = true;
}
