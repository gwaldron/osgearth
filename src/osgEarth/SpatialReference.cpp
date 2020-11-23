/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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

#include <osgEarth/SpatialReference>
#include <osgEarth/Registry>
#include <osgEarth/Cube>
#include <osgEarth/LocalTangentPlane>
#include <ogr_spatialref.h>
#include <cpl_conv.h>

#define LC "[SpatialReference] "

using namespace osgEarth;

namespace
{
    std::string
    getOGRAttrValue( void* _handle, const std::string& name, int child_num, bool lowercase =false)
    {
        const char* val = OSRGetAttrValue( _handle, name.c_str(), child_num );
        if ( val )
        {
            return lowercase ? toLower(val) : val;
        }
        return "";
    } 

    void geodeticToGeocentric(std::vector<osg::Vec3d>& points, const osg::EllipsoidModel* em)
    {
        for( unsigned i=0; i<points.size(); ++i )
        {
            double x, y, z;
            em->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( points[i].y() ), osg::DegreesToRadians( points[i].x() ), points[i].z(),
                x, y, z );
            points[i].set( x, y, z );
        }
    }

    void geocentricToGeodetic(std::vector<osg::Vec3d>& points, const osg::EllipsoidModel* em)
    {
        for( unsigned i=0; i<points.size(); ++i )
        {
            double lat, lon, alt;
            em->convertXYZToLatLongHeight(
                points[i].x(), points[i].y(), points[i].z(),
                lat, lon, alt );

            // deal with bug in OSG 3.4.x in which convertXYZToLatLongHeight can return
            // NANs when converting from (0,0,0) with a spherical ellipsoid -gw 2/5/2019
            if (osg::isNaN(lon)) lon = 0.0;
            if (osg::isNaN(lat)) lat = 0.0;
            if (osg::isNaN(alt)) alt = 0.0;

            points[i].set( osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), alt );
        }
    }

    // Make a MatrixTransform suitable for use with a Locator object based on the given extents.
    // Calling Locator::setTransformAsExtents doesn't work with OSG 2.6 due to the fact that the
    // _inverse member isn't updated properly.  Calling Locator::setTransform works correctly.
    osg::Matrixd
    getTransformFromExtents(double minX, double minY, double maxX, double maxY)
    {
        osg::Matrixd transform;
        transform.set(
            maxX - minX, 0.0, 0.0, 0.0,
            0.0, maxY - minY, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            minX, minY, 0.0, 1.0);
        return transform;
    }
}

//------------------------------------------------------------------------

SpatialReference::ThreadLocal::ThreadLocal() :
    _handle(nullptr),
    _workspace(nullptr),
    _workspaceSize(0u),
    _threadId(std::this_thread::get_id())
{
    //nop
}

SpatialReference::ThreadLocal::~ThreadLocal()
{
    if (_workspace)
        delete [] _workspace;

    for(auto& xformEntry : _xformCache)
    {
        optional<TransformInfo>& ti = xformEntry.second;
        if (ti.isSet() && ti->_handle != nullptr)
            OCTDestroyCoordinateTransformation(ti->_handle);
    }

    if (_handle)
    {
        OSRDestroySpatialReference(_handle);
    }
}

SpatialReference*
SpatialReference::create( const std::string& horiz, const std::string& vert )
{
    osg::ref_ptr<SpatialReference> srs = Registry::instance()->getOrCreateSRS( Key(horiz, vert) );
    return srs.release();
}

SpatialReference*
SpatialReference::createFromKey(const SpatialReference::Key& key)
{
    if (key.horizLower == "unified-cube")
        return new Contrib::CubeSpatialReference(key);
    else
        return new SpatialReference(key);
}

SpatialReference::SpatialReference(void* handle) :
    _valid(true),
    _initialized(false),
    _domain(GEOGRAPHIC),
    _is_mercator(false),
    _is_north_polar(false),
    _is_south_polar(false),
    _is_cube(false),
    _is_user_defined(false),
    _is_ltp(false),
    _is_spherical_mercator(false),
    _ellipsoidId(0u),
    _local("OE.SRS.Local"),
    _mutex("OE.SRS")
{
    _setup.srcHandle = handle;

    // force handle creation
    init();
}

SpatialReference::SpatialReference(const Key& key) :
    _valid(true),
    _initialized(false),
    _domain(GEOGRAPHIC),
    _is_mercator(false),
    _is_north_polar(false),
    _is_south_polar(false),
    _is_cube(false),
    _is_user_defined(false),
    _is_ltp(false),
    _is_spherical_mercator(false),
    _ellipsoidId(0u),
    _local("OE.SRS.Local"),
    _mutex("OE.SRS")
{
    // shortcut for spherical-mercator:
    // https://wiki.openstreetmap.org/wiki/EPSG:3857
    if (key.horizLower == "spherical-mercator" ||
        key.horizLower == "global-mercator"    ||
        key.horizLower == "web-mercator"       ||
        key.horizLower == "epsg:3857"          ||
        key.horizLower == "epsg:900913"        ||
        key.horizLower == "epsg:102100"        ||
        key.horizLower == "epsg:102113"        ||
        key.horizLower == "epsg:3785"          ||
        key.horizLower == "epsg:3587"          ||
        key.horizLower == "osgeo:41001")
    {
        // note the use of nadgrids=@null (see http://proj.maptools.org/faq.html)
        _setup.name = "Spherical Mercator";
        _setup.type = INIT_PROJ;
        _setup.horiz = "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +towgs84=0,0,0,0,0,0,0 +wktext +no_defs";
        _setup.vert = key.vertLower;
    }

    // true ellipsoidal ("world") mercator:
    // https://epsg.io/3395
    // https://gis.stackexchange.com/questions/259121/transformation-functions-for-epsg3395-projection-vs-epsg3857
    else if (
        key.horizLower == "world-mercator" ||
        key.horizLower == "epsg:3395" ||
        key.horizLower == "epsg:54004" ||
        key.horizLower == "epsg:9804" ||
        key.horizLower == "epsg:3832")
    {
        _setup.name = "World Mercator (WGS84)";
        _setup.type = INIT_PROJ;
        _setup.horiz = "+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
        _setup.vert = key.vertLower;
    }

    // common WGS84:
    else if(
        key.horizLower == "epsg:4326" ||
        key.horizLower == "wgs84")
    {
        _setup.name = "WGS84";
        _setup.type = INIT_PROJ;
        _setup.horiz = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";
        _setup.vert = key.vertLower;
    }

    // WGS84 Plate Carre:
    else if (
        key.horizLower == "plate-carre" || 
        key.horizLower == "plate-carree")
    {
        // https://proj4.org/operations/projections/eqc.html
        _setup.name = "Plate Carree";
        _setup.type = INIT_PROJ;
        _setup.horiz = "+proj=eqc +lat_ts=0 +lat_0=0 +lon_0=0 +x_0=0 +y_0=0 +units=m +ellps=WGS84 +datum=WGS84 +no_defs";
        _setup.vert = key.vertLower;
    }

    // custom srs for the unified cube
    else if (
        key.horizLower == "unified-cube" )
    {
        _setup.name = "Unified Cube";
        _setup.type = INIT_USER;
        _setup.horiz = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";
        _is_cube = true;
    }

    else if (
        key.horizLower.find( '+' ) == 0 )
    {
        //_setup.name = key.horiz;
        _setup.type = INIT_PROJ;
        _setup.horiz = key.horizLower;
    }
    else if (
        key.horizLower.find( "epsg:" )  == 0 ||
        key.horizLower.find( "osgeo:" ) == 0 )
    {
        _setup.name = key.horiz;
        _setup.type = INIT_PROJ;
        _setup.horiz = std::string("+init=") + key.horizLower;
    }
    else if (
        key.horizLower.find( "projcs" ) == 0 || 
        key.horizLower.find( "geogcs" ) == 0 )
    {
        //_setup.name = key.horiz;
        _setup.type = INIT_WKT;
        _setup.horiz = key.horiz;
    }
    else
    {
        // Try to set it from the user input.  This will handle things like CRS:84
        // createFromUserInput will actually handle all valid inputs from GDAL,
        // so we might be able to simplify this function and just call createFromUserInput.
        //_setup.name = key.horiz;
        _setup.type = INIT_USER;
        _setup.horiz = key.horiz;
    }

    // next, resolve the vertical SRS:
    if ( !key.vert.empty() && !ciEquals(key.vert, "geodetic") )
    {
        _vdatum = VerticalDatum::get( key.vert );
        if ( !_vdatum.valid() )
        {
            OE_WARN << LC << "Failed to locate vertical datum \"" << key.vert << "\"" << std::endl;
        }
    }

    _key = key;

    // create a handle for this thread and establish validity.
    init();
}

SpatialReference::ThreadLocal&
SpatialReference::getLocal() const
{
    ThreadLocal& local = _local.get();

    if (local._handle == nullptr)
    {
        local._threadId = std::this_thread::get_id();

        if (_setup.srcHandle != nullptr)
        {
            local._handle = OSRClone(_setup.srcHandle);
            if (!local._handle)
            {
                OE_WARN << LC << "Failed to clone an existing handle" << std::endl;
                _valid = false;
            }
        }

        else
        {
            local._handle = OSRNewSpatialReference(nullptr);
            OGRErr error = OGRERR_INVALID_HANDLE;

            if (_setup.type == INIT_PROJ)
            {
                error = OSRImportFromProj4(local._handle, _setup.horiz.c_str());
            }
            else if (_setup.type == INIT_WKT)
            {
                char buf[8192];
                char* buf_ptr = &buf[0];
                if (_setup.horiz.length() < 8192)
                {
                    strcpy(buf, _setup.horiz.c_str());

                    error = OSRImportFromWkt(local._handle, &buf_ptr);

                    if (error == OGRERR_NONE)
                    {
                        //TODO
                        //fixWKT();
                    }
                }
                else
                {
                    OE_WARN << LC << "BUFFER OVERFLOW - INTERNAL ERROR\n";
                    _valid = false;
                }
            }
            else
            {
                error = OSRSetFromUserInput(local._handle, _setup.horiz.c_str());
            }

            if (error == OGRERR_NONE)
            {
            }
            else
            {
                OE_WARN << LC << "Failed to create SRS from \"" << _setup.horiz << "\"" << std::endl;
                OSRDestroySpatialReference(local._handle);
                local._handle = nullptr;
                _valid = false;
            }
        }
    }

    if (local._threadId != std::this_thread::get_id())
    {
        OE_WARN << LC << "Thread Safety Violation at line " OE_STRINGIFY(__LINE__) << std::endl;
    }

    return local;
}

bool
SpatialReference::getBounds(Bounds& output) const
{
    output = _bounds;
    return _bounds.isValid();
}

void*
SpatialReference::getHandle() const
{
    return getLocal()._handle;
}

SpatialReference*
SpatialReference::createFromHandle(void* ogrHandle)
{
    OE_SOFT_ASSERT_AND_RETURN(ogrHandle!=nullptr, __func__, nullptr);

    return new SpatialReference(ogrHandle);

    //ThreadLocal& local = _local.get();
    //local._handle = OSRClone(ogrHandle);
    //if (!local._handle)
    //{
    //    OE_WARN << LC << "Internal error: createFromHandle() failed to clone" << std::endl;
    //    return 0L;
    //}

    //return new SpatialReference(clonedHandle);
}

#if 0
SpatialReference*
SpatialReference::fixWKT()
{
    std::string proj = getOGRAttrValue( _handle, "PROJECTION", 0 );

    // fix invalid ESRI LCC projections:
    if ( ciEquals( proj, "Lambert_Conformal_Conic" ) )
    {
        bool has_2_sps =
            !getOGRAttrValue( _handle, "Standard_Parallel_2", 0 ).empty() ||
            !getOGRAttrValue( _handle, "standard_parallel_2", 0 ).empty();

        std::string new_wkt = getWKT();
        if ( has_2_sps )
        {
            ciReplaceIn( new_wkt, "Lambert_Conformal_Conic", "Lambert_Conformal_Conic_2SP" );
        }
        else 
        {
            ciReplaceIn( new_wkt, "Lambert_Conformal_Conic", "Lambert_Conformal_Conic_1SP" );
        }

        OE_INFO << LC << "Morphing Lambert_Conformal_Conic to 1SP/2SP" << std::endl;
        
        return createFromWKT( new_wkt, _name );
    }

    // fixes for ESRI Plate_Carree and Equidistant_Cylindrical projections:
    else if ( proj == "Plate_Carree" )
    {
        std::string new_wkt = getWKT();
        ciReplaceIn( new_wkt, "Plate_Carree", "Equirectangular" );
        OE_INFO << LC << "Morphing Plate_Carree to Equirectangular" << std::endl;
        return createFromWKT( new_wkt, _name ); //, input->getReferenceFrame() );
    }
    else if ( proj == "Equidistant_Cylindrical" )
    {
        std::string new_wkt = getWKT();
        OE_INFO << LC << "Morphing Equidistant_Cylindrical to Equirectangular" << std::endl;
        ciReplaceIn( new_wkt, "Equidistant_Cylindrical", "Equirectangular" );
        return createFromWKT( new_wkt, _name );
    }

    // no changes.
    return this;
}
#endif


/****************************************************************************/

SpatialReference::~SpatialReference()
{
    //nop
}

bool
SpatialReference::isGeographic() const 
{
    return _domain == GEOGRAPHIC;
}

bool
SpatialReference::isGeodetic() const 
{
    return isGeographic() && !_vdatum.valid();
}

bool
SpatialReference::isProjected() const
{
    return _domain == PROJECTED;
}

bool
SpatialReference::isGeocentric() const 
{
    return _domain == GEOCENTRIC;
}

const std::string&
SpatialReference::getName() const
{
    return _name;
}

const osg::EllipsoidModel*
SpatialReference::getEllipsoid() const
{
    return _ellipsoid.get();
}

const std::string&
SpatialReference::getDatumName() const
{
    return _datum;
}

const Units&
SpatialReference::getUnits() const
{
    return _units;
}

double
SpatialReference::getReportedLinearUnits() const
{
    return _reportedLinearUnits;
}

const std::string&
SpatialReference::getWKT() const 
{
    return _wkt;
}

const VerticalDatum*
SpatialReference::getVerticalDatum() const
{
    return _vdatum.get();
}

const SpatialReference::Key&
SpatialReference::getKey() const
{
    return _key;
}

const std::string&
SpatialReference::getHorizInitString() const 
{ 
    return _key.horiz;
}

const std::string&
SpatialReference::getVertInitString() const 
{ 
    return _key.vert;
}

bool
SpatialReference::isEquivalentTo( const SpatialReference* rhs ) const
{
    return _isEquivalentTo( rhs, true );
}

bool
SpatialReference::isHorizEquivalentTo( const SpatialReference* rhs ) const
{
    return _isEquivalentTo( rhs, false );
}

bool
SpatialReference::isVertEquivalentTo( const SpatialReference* rhs ) const
{
    // vertical equivalence means the same vertical datum and the same
    // reference ellipsoid.
    return
        _vdatum.get() == rhs->_vdatum.get() &&
        _ellipsoidId  == rhs->_ellipsoidId;
}

bool
SpatialReference::_isEquivalentTo( const SpatialReference* rhs, bool considerVDatum ) const
{
    if ( !rhs )
        return false;

    if ( this == rhs )
        return true;

    if (isGeographic()  != rhs->isGeographic()  ||
        isMercator()    != rhs->isMercator()    ||
        isGeocentric()  != rhs->isGeocentric()  ||
        isSphericalMercator() != rhs->isSphericalMercator() ||
        isNorthPolar()  != rhs->isNorthPolar()  ||
        isSouthPolar()  != rhs->isSouthPolar()  ||
        isUserDefined() != rhs->isUserDefined() ||
        isCube()        != rhs->isCube()        ||
        isLTP()         != rhs->isLTP() )
    {
        return false;
    }

    if (isGeocentric() && rhs->isGeocentric())
        return true;

    if ( considerVDatum && (_vdatum.get() != rhs->_vdatum.get()) )
        return false;

    if (_key.horizLower == rhs->_key.horizLower &&
        (!considerVDatum || (_key.vertLower == rhs->_key.vertLower) ) )
    {
        return true;
    }

    if ( _proj4 == rhs->_proj4 )
        return true;

    if ( _wkt == rhs->_wkt )
        return true;

    if (this->isGeographic() && rhs->isGeographic())
    {
        return
            osg::equivalent( getEllipsoid()->getRadiusEquator(), rhs->getEllipsoid()->getRadiusEquator() ) &&
            osg::equivalent( getEllipsoid()->getRadiusPolar(), rhs->getEllipsoid()->getRadiusPolar() );
    }

    // last resort, since it requires the lock
    void* myHandle = getHandle();
    void* rhsHandle = rhs->getHandle();

    return 
        myHandle &&
        rhsHandle &&
        OSRIsSame(myHandle, rhsHandle) == TRUE;
}

const SpatialReference*
SpatialReference::getGeographicSRS() const
{
    if ( isGeographic() )
        return this;

    if ( _is_spherical_mercator )
        return get("wgs84", _key.vertLower);

    if ( !_geo_srs.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);

        if ( !_geo_srs.valid() ) // double-check pattern
        {
            // temporary SRS to build the WKT
            void* temp_handle = OSRNewSpatialReference(NULL);
            int err = OSRCopyGeogCSFrom(temp_handle, getHandle());
            if (err == OGRERR_NONE)
            {
                char* wktbuf;
                if (OSRExportToWkt(temp_handle, &wktbuf) == OGRERR_NONE)
                {
                    Key key(std::string(wktbuf), _key.vertLower);
                    _geo_srs = new SpatialReference(key);
                    CPLFree(wktbuf);
                }
            }
            OSRDestroySpatialReference(temp_handle);
        }
    }

    return _geo_srs.get();
}

const SpatialReference*
SpatialReference::getGeodeticSRS() const
{
    if ( isGeodetic() )
        return this;

    if ( !_geodetic_srs.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);

        if ( !_geodetic_srs.valid() ) // double check pattern
        {
            // temporary SRS to build the WKT
            void* temp_handle = OSRNewSpatialReference(NULL);
            int err = OSRCopyGeogCSFrom(temp_handle, getHandle());
            if (err == OGRERR_NONE)
            {
                char* wktbuf;
                if (OSRExportToWkt(temp_handle, &wktbuf) == OGRERR_NONE)
                {
                    Key key(std::string(wktbuf), "");
                    _geodetic_srs = new SpatialReference(key);
                    CPLFree(wktbuf);
                }
            }
            OSRDestroySpatialReference(temp_handle);
        }
    }

    return _geodetic_srs.get();
}

const SpatialReference*
SpatialReference::getGeocentricSRS() const
{
    if ( isGeocentric() )
        return this;

    if ( !_geocentric_srs.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);

        if ( !_geocentric_srs.valid() ) // double-check pattern
        {
            // temporary SRS to build the WKT
            void* temp_handle = OSRNewSpatialReference(NULL);
            int err = OSRCopyGeogCSFrom(temp_handle, getHandle());
            if (err == OGRERR_NONE)
            {
                char* wktbuf;
                if (OSRExportToWkt(temp_handle, &wktbuf) == OGRERR_NONE)
                {
                    Key key(std::string(wktbuf), ""); // to vdatum in ECEF
                    _geocentric_srs = new SpatialReference(key);
                    _geocentric_srs->_domain = GEOCENTRIC;
                    CPLFree(wktbuf);
                }
            }
            OSRDestroySpatialReference(temp_handle);
        }
    }

    return _geocentric_srs.get();
}

const SpatialReference*
SpatialReference::createTangentPlaneSRS(const osg::Vec3d& origin) const
{
    osg::Vec3d lla;
    const SpatialReference* srs = getGeographicSRS();
    if ( srs && transform(origin, srs, lla) )
    {
        const Key& key = srs->getKey();
        SpatialReference* ltp = new TangentPlaneSpatialReference(key, lla);
        return ltp;
    }
    else
    {
        OE_WARN << LC << "Unable to create LTP SRS" << std::endl;
        return nullptr;
    }
}

const SpatialReference*
SpatialReference::createTransMercFromLongitude( const Angle& lon ) const
{
    // note. using tmerc with +lat_0 <> 0 is sloooooow.
    std::string datum = getDatumName();
    std::string horiz = Stringify()
        << "+proj=tmerc +lat_0=0"
        << " +lon_0=" << lon.as(Units::DEGREES)
        << " +datum=" << (!datum.empty() ? "wgs84" : datum);

    return SpatialReference::create( horiz, getVertInitString() );
}

const SpatialReference*
SpatialReference::createUTMFromLonLat(const Angle& lon, const Angle& lat) const
{
    // note. UTM is up to 10% faster than TMERC for the same meridian.
    unsigned zone = 1 + (unsigned)floor((lon.as(Units::DEGREES)+180.0)/6.0);
    std::string datum = getDatumName();
    std::string horiz = Stringify()
        << "+proj=utm +zone=" << zone
        << (lat.as(Units::DEGREES) < 0 ? " +south" : "")
        << " +datum=" << (!datum.empty() ? "wgs84" : datum);

    return SpatialReference::create(horiz, getVertInitString());
}

const SpatialReference*
SpatialReference::createEquirectangularSRS() const
{
    return SpatialReference::create(
        "+proj=eqc +units=m +no_defs", 
        getVertInitString());
}

bool
SpatialReference::isMercator() const
{
    return _is_mercator;
}

bool
SpatialReference::isSphericalMercator() const
{
    return _is_spherical_mercator;
}

bool 
SpatialReference::isNorthPolar() const
{
    return _is_north_polar;
}

bool 
SpatialReference::isSouthPolar() const
{
    return _is_south_polar;
}

bool
SpatialReference::isUserDefined() const
{
    return _is_user_defined;
}

bool
SpatialReference::isCube() const
{
    return _is_cube;
}

bool
SpatialReference::populateCoordinateSystemNode( osg::CoordinateSystemNode* csn ) const
{
    OE_SOFT_ASSERT_AND_RETURN(csn!=nullptr, __func__, false);

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

    csn->setEllipsoidModel( _ellipsoid.get() );
    
    return true;
}

bool
SpatialReference::createLocalToWorld(const osg::Vec3d& xyz, osg::Matrixd& out_local2world ) const
{
    if ( isProjected() && !isCube() )
    {
        osg::Vec3d world;
        if ( !transformToWorld( xyz, world ) )
            return false;
        out_local2world = osg::Matrix::translate(world);
    }
    else if ( isGeocentric() )
    {
        _ellipsoid->computeLocalToWorldTransformFromXYZ(xyz.x(), xyz.y(), xyz.z(), out_local2world);
    }
    else
    {
        // convert to ECEF:
        osg::Vec3d ecef;
        if ( !transform(xyz, getGeocentricSRS(), ecef) )
            return false;

        // and create the matrix.
        _ellipsoid->computeLocalToWorldTransformFromXYZ(ecef.x(), ecef.y(), ecef.z(), out_local2world);
    }
    return true;
}

bool
SpatialReference::createWorldToLocal(const osg::Vec3d& xyz, osg::Matrixd& out_world2local ) const
{
    osg::Matrixd local2world;
    if ( !createLocalToWorld(xyz, local2world) )
        return false;
    out_world2local.invert(local2world);
    return true;
}


bool
SpatialReference::transform(const osg::Vec3d&       input,
                            const SpatialReference* outputSRS,
                            osg::Vec3d&             output) const
{
    OE_SOFT_ASSERT_AND_RETURN(outputSRS!=nullptr, __func__, false);

    std::vector<osg::Vec3d> v(1, input);

    if ( transform(v, outputSRS) )
    {
        output = v[0];
        return true;
    }
    return false;
}


bool
SpatialReference::transform(std::vector<osg::Vec3d>& points,
                            const SpatialReference*  outputSRS) const
{
    OE_SOFT_ASSERT_AND_RETURN(outputSRS!=nullptr, __func__, false);

    // trivial equivalency:
    if ( isEquivalentTo(outputSRS) )
        return true;
    
    bool success = false;

    // do the pre-transformation pass:
    const SpatialReference* inputSRS = preTransform( points );
    if ( !inputSRS )
        return false;
        
    if ( inputSRS->isGeocentric() && !outputSRS->isGeocentric() )
    {
        const SpatialReference* outputGeoSRS = outputSRS->getGeodeticSRS();
        geocentricToGeodetic(points, outputGeoSRS->getEllipsoid());
        return outputGeoSRS->transform(points, outputSRS);
    }

    else if ( !inputSRS->isGeocentric() && outputSRS->isGeocentric() )
    {
        const SpatialReference* outputGeoSRS = outputSRS->getGeodeticSRS();
        success = inputSRS->transform(points, outputGeoSRS);
        geodeticToGeocentric(points, outputGeoSRS->getEllipsoid());
        return success;
    }

    // if the points are starting as geographic, do the Z's first to avoid an unneccesary
    // transformation in the case of differing vdatums.
    bool z_done = false;
    if ( inputSRS->isGeographic() )
    {
        z_done = inputSRS->transformZ( points, outputSRS, true );
    }

    ThreadLocal& local = getLocal();

    // move the xy data into straight arrays that OGR can use
    unsigned count = points.size();

    if (count*2 > local._workspaceSize)
    {
        if (local._workspace)
            delete [] local._workspace;
        local._workspace = new double[count*2];
        local._workspaceSize = count*2;
    }

    double* x = local._workspace;
    double* y = local._workspace + count;

    for( unsigned i=0; i<count; i++ )
    {
        x[i] = points[i].x();
        y[i] = points[i].y();
    }

    success = inputSRS->transformXYPointArrays( local, x, y, count, outputSRS );

    if ( success )
    {
        if ( inputSRS->isProjected() && outputSRS->isGeographic() )
        {
            // special case: when going from projected to geographic, clamp the 
            // points to the maximum geographic extent. Sometimes the conversion from
            // a global/projected SRS (like mercator) will result in *slightly* invalid
            // geographic points (like long=180.000003), so this addresses that issue.
            for( unsigned i=0; i<count; i++ )
            {
                points[i].x() = osg::clampBetween( x[i], -180.0, 180.0 );
                points[i].y() = osg::clampBetween( y[i],  -90.0,  90.0 );
            }
        }
        else
        {
            for( unsigned i=0; i<count; i++ )
            {
                points[i].x() = x[i];
                points[i].y() = y[i];
            }
        }
    }

    if (success)
    {
        // calculate the Zs if we haven't already done so
        if ( !z_done )
        {
            z_done = inputSRS->transformZ( points, outputSRS, outputSRS->isGeographic() );
        }   

        // run the user post-transform code
        outputSRS->postTransform( points );
    }

    return success;
}


bool 
SpatialReference::transform2D(double x, double y,
                              const SpatialReference* outputSRS,
                              double& out_x, double& out_y ) const
{
    OE_SOFT_ASSERT_AND_RETURN(outputSRS!=nullptr, __func__, false);

    osg::Vec3d temp(x,y,0);
    bool ok = transform(temp, outputSRS, temp);
    if ( ok ) {
        out_x = temp.x();
        out_y = temp.y();
    }
    return ok;
}


bool
SpatialReference::transformXYPointArrays(
    ThreadLocal& local,
    double*  x,
    double*  y,
    unsigned count,
    const SpatialReference* out_srs) const
{  
    OE_SOFT_ASSERT_AND_RETURN(out_srs!=nullptr, __func__, false);

    // Transform the X and Y values inside an exclusive GDAL/OGR lock
    optional<TransformInfo>& xform = local._xformCache[out_srs->getWKT()];
    if (!xform.isSet())
    {
        xform->_handle = OCTNewCoordinateTransformation(local._handle, out_srs->getHandle());

        if ( xform->_handle == nullptr )
        {
            OE_WARN << LC
                << "SRS xform not possible:" << std::endl
                << "    From => " << getName() << std::endl
                << "    To   => " << out_srs->getName() << std::endl;

            OE_WARN << LC << "INPUT: " << getWKT() << std::endl
                << "OUTPUT: " << out_srs->getWKT() << std::endl;

            const char* errmsg = CPLGetLastErrorMsg();
            OE_WARN << LC << "ERROR: " << (errmsg? errmsg : "do not know") << std::endl;

            xform->_handle = nullptr;
            xform->_failed = true;

            return false;
        }
    }

    if (xform->_failed)
    {
        return false;
    }

    return OCTTransform(xform->_handle, count, x, y, 0L) > 0;
}


bool
SpatialReference::transformZ(std::vector<osg::Vec3d>& points,
                             const SpatialReference*  outputSRS,
                             bool                     pointsAreLatLong) const
{
    OE_SOFT_ASSERT_AND_RETURN(outputSRS!=nullptr, __func__, false);

    const VerticalDatum* outVDatum = outputSRS->getVerticalDatum();

    // same vdatum, no xformation necessary.
    if ( _vdatum.get() == outVDatum )
        return true;

    Units inUnits = _vdatum.valid() ? _vdatum->getUnits() : Units::METERS;
    Units outUnits = outVDatum ? outVDatum->getUnits() : inUnits;

    if ( isGeographic() || pointsAreLatLong )
    {
        for( unsigned i=0; i<points.size(); ++i )
        {
            if ( _vdatum.valid() )
            {
                // to HAE:
                points[i].z() = _vdatum->msl2hae( points[i].y(), points[i].x(), points[i].z() );
            }

            // do the units conversion:
            points[i].z() = inUnits.convertTo(outUnits, points[i].z());

            if ( outVDatum )
            {
                // to MSL:
                points[i].z() = outVDatum->hae2msl( points[i].y(), points[i].x(), points[i].z() );
            }
        }
    }

    else // need to xform input points
    {
        // copy the points and convert them to geographic coordinates (lat/long with the same Z):
        std::vector<osg::Vec3d> geopoints(points);
        transform( geopoints, getGeographicSRS() );

        for( unsigned i=0; i<geopoints.size(); ++i )
        {
            if ( _vdatum.valid() )
            {
                // to HAE:
                points[i].z() = _vdatum->msl2hae( geopoints[i].y(), geopoints[i].x(), points[i].z() );
            }

            // do the units conversion:
            points[i].z() = inUnits.convertTo(outUnits, points[i].z());

            if ( outVDatum )
            {
                // to MSL:
                points[i].z() = outVDatum->hae2msl( geopoints[i].y(), geopoints[i].x(), points[i].z() );
            }
        }
    }

    return true;
}

bool 
SpatialReference::transformToWorld(const osg::Vec3d& input,
                                   osg::Vec3d&       output ) const
{
    if ( isGeographic() || isCube() )
    {
        return transform(input, getGeocentricSRS(), output);
    }
    else // isProjected
    {
        output = input;
        if ( _vdatum.valid() )
        {
            osg::Vec3d geo(input);
            if ( !transform(input, getGeographicSRS(), geo) )
                return false;

            output.z() = _vdatum->msl2hae( geo.y(), geo.x(), input.z() );
        }
        return true;
    }
}

bool 
SpatialReference::transformFromWorld(const osg::Vec3d& world,
                                     osg::Vec3d&       output,
                                     double*           out_haeZ ) const
{
    if ( isGeographic() || isCube() )
    {
        bool ok = getGeocentricSRS()->transform(world, this, output);
        if ( ok && out_haeZ )
        {
            if ( _vdatum.valid() )
                *out_haeZ = _vdatum->msl2hae(output.y(), output.x(), output.z());
            else
                *out_haeZ = output.z();
        }
        return ok;
    }
    else // isProjected
    {
        output = world;

        if (out_haeZ)
            *out_haeZ = world.z();

        if ( _vdatum.valid() )
        {
            // get the geographic coords by converting x/y/hae -> lat/long/msl:
            osg::Vec3d lla;
            if (!transform(world, getGeographicSRS(), lla) )
                return false;

            output.z() = lla.z();
        }

        return true;
    }
}

double
SpatialReference::transformUnits(double                  input,
                                 const SpatialReference* outSRS,
                                 double                  latitude) const
{
    OE_SOFT_ASSERT_AND_RETURN(outSRS!=nullptr, __func__, input);

    if ( this->isProjected() && outSRS->isGeographic() )
    {
        double metersPerEquatorialDegree = (outSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double inputDegrees = getUnits().convertTo(Units::METERS, input) / (metersPerEquatorialDegree * cos(osg::DegreesToRadians(latitude)));
        return Units::DEGREES.convertTo( outSRS->getUnits(), inputDegrees );
    }
    else if ( this->isGeocentric() && outSRS->isGeographic() )
    {
        double metersPerEquatorialDegree = (outSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double inputDegrees = input / (metersPerEquatorialDegree * cos(osg::DegreesToRadians(latitude)));
        return Units::DEGREES.convertTo( outSRS->getUnits(), inputDegrees );
    }
    else if ( this->isGeographic() && outSRS->isProjected() )
    {
        double metersPerEquatorialDegree = (outSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double inputMeters = getUnits().convertTo(Units::DEGREES, input) * (metersPerEquatorialDegree * cos(osg::DegreesToRadians(latitude)));
        return Units::METERS.convertTo( outSRS->getUnits(), inputMeters );
    }
    else if ( this->isGeographic() && outSRS->isGeocentric() )
    {
        double metersPerEquatorialDegree = (outSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        return getUnits().convertTo(Units::DEGREES, input) * (metersPerEquatorialDegree * cos(osg::DegreesToRadians(latitude)));
    }
    else // both projected or both geographic.
    {
        return getUnits().convertTo( outSRS->getUnits(), input );
    }
}

double
SpatialReference::transformUnits(const Distance&         distance,
                                 const SpatialReference* outSRS,
                                 double                  latitude)
{
    OE_SOFT_ASSERT_AND_RETURN(outSRS!=nullptr, __func__, distance.getValue());

    if ( distance.getUnits().isLinear() && outSRS->isGeographic() )
    {
        double metersPerEquatorialDegree = (outSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double inputDegrees = distance.as(Units::METERS) / (metersPerEquatorialDegree * cos(osg::DegreesToRadians(latitude)));
        return Units::DEGREES.convertTo( outSRS->getUnits(), inputDegrees );
    }
    else if ( distance.getUnits().isAngular() && outSRS->isProjected() )
    {
        double metersPerEquatorialDegree = (outSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double inputMeters = distance.as(Units::DEGREES) * (metersPerEquatorialDegree * cos(osg::DegreesToRadians(latitude)));
        return Units::METERS.convertTo( outSRS->getUnits(), inputMeters );
    }
    else // both projected or both geographic.
    {
        return distance.as( outSRS->getUnits() );
    }
}

bool
SpatialReference::transformExtentToMBR(
    const SpatialReference* to_srs,
    double&                 in_out_xmin,
    double&                 in_out_ymin,
    double&                 in_out_xmax,
    double&                 in_out_ymax) const
{
    OE_SOFT_ASSERT_AND_RETURN(to_srs!=nullptr, __func__, false);

    // Transform all points and take the maximum bounding rectangle the resulting points
    std::vector<osg::Vec3d> v;

    double height = in_out_ymax - in_out_ymin;
    double width = in_out_xmax - in_out_xmin;
    v.push_back( osg::Vec3d(in_out_xmin, in_out_ymin, 0) ); // ll    
    v.push_back( osg::Vec3d(in_out_xmin, in_out_ymax, 0) ); // ul
    v.push_back( osg::Vec3d(in_out_xmax, in_out_ymax, 0) ); // ur
    v.push_back( osg::Vec3d(in_out_xmax, in_out_ymin, 0) ); // lr

    //We also sample along the edges of the bounding box and include them in the 
    //MBR computation in case you are dealing with a projection that will cause the edges
    //of the bounding box to be expanded.  This was first noticed when dealing with converting
    //Hotline Oblique Mercator to WGS84
   
    //Sample the edges
    unsigned int numSamples = 5;    
    double dWidth  = width / (numSamples - 1 );
    double dHeight = height / (numSamples - 1 );
    
    //Left edge
    for (unsigned int i = 0; i < numSamples; i++)
    {
        v.push_back( osg::Vec3d(in_out_xmin, in_out_ymin + dHeight * (double)i, 0) );
    }

    //Right edge
    for (unsigned int i = 0; i < numSamples; i++)
    {
        v.push_back( osg::Vec3d(in_out_xmax, in_out_ymin + dHeight * (double)i, 0) );
    }

    //Top edge
    for (unsigned int i = 0; i < numSamples; i++)
    {
        v.push_back( osg::Vec3d(in_out_xmin + dWidth * (double)i, in_out_ymax, 0) );
    }

    //Bottom edge
    for (unsigned int i = 0; i < numSamples; i++)
    {
        v.push_back( osg::Vec3d(in_out_xmin + dWidth * (double)i, in_out_ymin, 0) );
    }
    
    if ( transform(v, to_srs) )
    {
        bool swapXValues = ( isGeographic() && in_out_xmin > in_out_xmax );
        in_out_xmin = DBL_MAX;
        in_out_ymin = DBL_MAX;
        in_out_xmax = -DBL_MAX;
        in_out_ymax = -DBL_MAX;

        for (unsigned int i = 0; i < v.size(); i++)
        {
            in_out_xmin = osg::minimum( v[i].x(), in_out_xmin );
            in_out_ymin = osg::minimum( v[i].y(), in_out_ymin );
            in_out_xmax = osg::maximum( v[i].x(), in_out_xmax );
            in_out_ymax = osg::maximum( v[i].y(), in_out_ymax );
        }

        if ( swapXValues )
            std::swap( in_out_xmin, in_out_xmax );

        return true;
    }

    return false;
}

bool 
SpatialReference::transformExtentPoints(
    const SpatialReference* to_srs,
    double in_xmin, double in_ymin,
    double in_xmax, double in_ymax,
    double* x, double* y,
    unsigned int numx, unsigned int numy ) const
{
    OE_SOFT_ASSERT_AND_RETURN(to_srs!=nullptr, __func__, false);

    std::vector<osg::Vec3d> points;

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

            points.push_back(osg::Vec3d(dest_x, dest_y, 0));
            pixel++;     
        }
    }

    if ( transform( points, to_srs ) )
    {
        for( unsigned i=0; i<points.size(); ++i )
        {
            x[i] = points[i].x();
            y[i] = points[i].y();
        }
        return true;
    }
    return false;
}

void
SpatialReference::init()
{
    void* handle = getHandle();
    
    if (!handle)
    {
        _valid = false;
        return;
    }

    if (_is_ltp)
    {
        _is_user_defined = true;
        _domain = PROJECTED;
    }
    else if (_is_cube)
    {
        _domain = PROJECTED;
    }
    else if (!isGeocentric())
    {
        _domain = OSRIsGeographic(handle) != 0 ?
            GEOGRAPHIC : PROJECTED;
    }

    // Give the SRS a name if it doesn't have one:
    if ( !_setup.name.empty())
    {
        _name = _setup.name;
    }

    // extract the ellipsoid parameters:
    int err;
    double semi_major_axis = OSRGetSemiMajor( handle, &err );
    double semi_minor_axis = OSRGetSemiMinor( handle, &err );
    _ellipsoid = new osg::EllipsoidModel( semi_major_axis, semi_minor_axis );

    // unique ID for comparing ellipsoids quickly:
    _ellipsoidId = hashString( Stringify() 
        << std::fixed << std::setprecision(10) 
        << _ellipsoid->getRadiusEquator() << ";" << _ellipsoid->getRadiusPolar() );

    // try to get an ellipsoid name:
    _ellipsoid->setName( getOGRAttrValue(handle, "SPHEROID", 0, true) );

    // extract the projection:
    if ( _name.empty() || _name == "unnamed" )
    {
        _name = isGeographic()? 
            getOGRAttrValue( handle, "GEOGCS", 0 ) : 
            getOGRAttrValue( handle, "PROJCS", 0 );
    }
    std::string proj = getOGRAttrValue( handle, "PROJECTION", 0, true );

    // check for the Mercator projection:
    _is_mercator = !proj.empty() && proj.find("mercator")==0;

    // check for spherical mercator (a special case)
    _is_spherical_mercator = _is_mercator && osg::equivalent(semi_major_axis, semi_minor_axis);

    // check for the Polar projection:
    if ( !proj.empty() && proj.find("polar_stereographic") != std::string::npos )
    {
        double lat = as<double>( getOGRAttrValue( handle, "latitude_of_origin", 0, true ), -90.0 );
        _is_north_polar = lat > 0.0;
        _is_south_polar = lat < 0.0;
    }
    else
    {
      _is_north_polar = false;
      _is_south_polar = false;
    }

    // Try to extract the horizontal datum
    _datum = getOGRAttrValue( handle, "DATUM", 0, true );

    // Extract the base units:
    std::string units = getOGRAttrValue( handle, "UNIT", 0, true );
    double unitMultiplier = osgEarth::Util::as<double>( getOGRAttrValue( handle, "UNIT", 1, true ), 1.0 );
    if ( isGeographic() )
        _units = Units(units, units, Units::TYPE_ANGULAR, unitMultiplier);
    else
        _units = Units(units, units, Units::TYPE_LINEAR, unitMultiplier);

    _reportedLinearUnits = OSRGetLinearUnits(handle, nullptr);
    
    // Try to extract the PROJ4 initialization string:
    char* proj4buf;
    if ( OSRExportToProj4( handle, &proj4buf ) == OGRERR_NONE )
    {
        _proj4 = proj4buf;
        CPLFree( proj4buf );
    }

    // Try to extract the OGC well-known-text (WKT) string:
    char* wktbuf;
    if ( OSRExportToWkt( handle, &wktbuf ) == OGRERR_NONE )
    {
        _wkt = wktbuf;
        CPLFree( wktbuf );
    }

    if ( _name == "unnamed" || _name.empty() )
    {
        _name =
            isGeographic()? "Geographic" :
            isGeocentric()? "Geocentric" :
            isCube() ? "Unified Cube" :
            isLTP() ? "Tangent Plane" :
            _is_spherical_mercator ? "Spherical Mercator" :
            _is_mercator? "Mercator" :
            ( !_proj4.empty()? _proj4 : "Projected" );
    }

    // Build a 'normalized' initialization key.
    if ( !_proj4.empty() )
    {
        _key.horiz = _proj4;
        _key.horizLower = toLower(_key.horiz);
    }
    else if ( !_wkt.empty() )
    {
        _key.horiz = _wkt;
        _key.horizLower = toLower(_key.horiz);
    }
    if ( _vdatum.valid() )
    {
        _key.vert = _vdatum->getInitString();
        _key.vertLower = toLower(_key.vert);
    }

    // Guess the appropriate bounds for this SRS.
    if (isGeographic() || isGeocentric())
    {
        _bounds.set(-180.0, -90.0, 180.0, 90.0);
    }

    if (isMercator() || isSphericalMercator())
    {
        _bounds.set(MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY);
    }

    int isNorth;
    if (OSRGetUTMZone(handle, &isNorth))
    {
        if (isNorth)
            _bounds.set(166000, 0, 834000, 9330000);
        else
            _bounds.set(166000, 1116915, 834000, 10000000);
    }
}

