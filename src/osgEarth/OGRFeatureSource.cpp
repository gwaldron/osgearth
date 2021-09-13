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
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/OgrUtils>
#include <osgEarth/GeometryUtils>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Filter>

#include <osgEarth/Registry>
#include <osgEarth/StringUtils>

#include <gdal.h>
#include <queue>
#include <list>

#define LC "[OGRFeatureSource] "

using namespace osgEarth;

namespace osgEarth { namespace OGR
{
    // helper function.
    OGRLayerH openLayer(OGRDataSourceH ds, const std::string& layer)
    {
        OGRLayerH h = GDALDatasetGetLayerByName(ds, layer.c_str());
        if ( !h )
        {
            unsigned index = Strings::as<unsigned>(layer, 0);
            h = GDALDatasetGetLayer(ds, index);
        }
        return h;
    }

    /**
     * Determine whether a point is valid or not.  Some shapefiles can have points that are ridiculously big, which are really invalid data
     * but shapefiles have no way of marking the data as invalid.  So instead we check for really large values that are indiciative of something being wrong.
     */
    inline bool isPointValid( const osg::Vec3d& v, double thresh = 1e10 )
    {
        return (!v.isNaN() && osg::absolute( v.x() ) < thresh && osg::absolute( v.y() ) < thresh && osg::absolute( v.z() ) < thresh );
    }

    /**
     * Checks to see if all points in the Geometry are valid, and makes minor repairs
     * if necessary. Returns false if the geometry cannot be validated.
     */
    inline bool validateGeometry( Geometry* geometry )
    {        
        if (!geometry) return false;

        if (!geometry->isValid()) return false;

        for (Geometry::iterator i = geometry->begin(); i != geometry->end(); ++i)
        {
            // a "NaN" Z value is automatically changed to zero:
            if (osg::isNaN(i->z()))
                i->z() = 0.0;

            // then we test for a valid point.
            if (!isPointValid( *i ))
            {
                return false;
            }
        }
        return true;
    }
} }

//........................................................................

OGR::OGRFeatureCursor::OGRFeatureCursor(
    OGRDataSourceH dsHandle,
    OGRLayerH layerHandle,
    const FeatureSource* source,
    const FeatureProfile* profile,
    const Query& query,
    const FeatureFilterChain* filters,
    bool rewindPolygons,
    unsigned chunkSize,
    ProgressCallback* progress) :

FeatureCursor     ( progress ),
_source           ( source ),
_dsHandle         ( dsHandle ),
_layerHandle      ( layerHandle ),
_resultSetHandle  ( 0L ),
_spatialFilter    ( 0L ),
_query            ( query ),
_chunkSize        ( chunkSize == 0u ? 500u : chunkSize ),
_nextHandleToQueue( 0L ),
_resultSetEndReached(false),
_profile          ( profile ),
_filters          ( filters ),
_rewindPolygons   ( rewindPolygons )
{
    std::string expr;
    std::string from = OGR_FD_GetName(OGR_L_GetLayerDefn(_layerHandle));

    std::string driverName = OGR_Dr_GetName(OGR_DS_GetDriver(dsHandle));

    // Quote the layer name if it is a shapefile, so we can handle any weird filenames like those with spaces or hyphens.
    // Or quote any layers containing spaces for PostgreSQL
    if (driverName == "ESRI Shapefile" || driverName == "VRT" ||
        from.find(' ') != std::string::npos)
    {
        std::string delim = "\"";
        from = delim + from + delim;
    }

    if (_query.expression().isSet())
    {
        // build the SQL: allow the Query to include either a full SQL statement or
        // just the WHERE clause.
        expr = _query.expression().value();

        // if the expression is just a where clause, expand it into a complete SQL expression.
        std::string temp = osgEarth::toLower(expr);

        if (temp.find("select") != 0)
        {
            std::stringstream buf;
            buf << "SELECT * FROM " << from << " WHERE " << expr;
            std::string bufStr;
            bufStr = buf.str();
            expr = bufStr;
        }
    }
    else
    {
        std::stringstream buf;
        buf << "SELECT * FROM " << from;
        expr = buf.str();
    }

    //Include the order by clause if it's set
    if (_query.orderby().isSet())
    {
        std::string orderby = _query.orderby().value();

        std::string temp = osgEarth::toLower(orderby);

        if (temp.find("order by") != 0)
        {
            std::stringstream buf;
            buf << "ORDER BY " << orderby;
            std::string bufStr;
            bufStr = buf.str();
            orderby = buf.str();
        }
        expr += (" " + orderby);
    }

    // if the tilekey is set, convert it to feature profile coords
    if (_query.tileKey().isSet() && !_query.bounds().isSet() && profile)
    {
        GeoExtent localEx = _query.tileKey()->getExtent().transform(profile->getSRS());
        _query.bounds() = localEx.bounds();
    }

    // if there's a spatial extent in the query, build the spatial filter:
    if (_query.bounds().isSet())
    {
        OGRGeometryH ring = OGR_G_CreateGeometry(wkbLinearRing);
        OGR_G_AddPoint(ring, _query.bounds()->xMin(), _query.bounds()->yMin(), 0);
        OGR_G_AddPoint(ring, _query.bounds()->xMin(), _query.bounds()->yMax(), 0);
        OGR_G_AddPoint(ring, _query.bounds()->xMax(), _query.bounds()->yMax(), 0);
        OGR_G_AddPoint(ring, _query.bounds()->xMax(), _query.bounds()->yMin(), 0);
        OGR_G_AddPoint(ring, _query.bounds()->xMin(), _query.bounds()->yMin(), 0);

        _spatialFilter = OGR_G_CreateGeometry(wkbPolygon);
        OGR_G_AddGeometryDirectly(_spatialFilter, ring);
        // note: "Directly" above means _spatialFilter takes ownership if ring handle
    }


    OE_DEBUG << LC << "SQL: " << expr << std::endl;
    _resultSetHandle = GDALDatasetExecuteSQL(_dsHandle, expr.c_str(), _spatialFilter, 0L);

    if (_resultSetHandle)
    {
        OGR_L_ResetReading(_resultSetHandle);
    }

    readChunk();
}

OGR::OGRFeatureCursor::OGRFeatureCursor(OGRLayerH resultSetHandle, const FeatureProfile* profile) :
    FeatureCursor(NULL),
    _resultSetHandle(resultSetHandle),
    _profile(profile),
    _dsHandle(NULL),
    _layerHandle(NULL),
    _spatialFilter(0L),
    _chunkSize(500),
    _nextHandleToQueue(0L),
    _resultSetEndReached(false)
{
    if (_resultSetHandle)
    {
        OGR_L_ResetReading(_resultSetHandle);
    }

    readChunk();
}

OGR::OGRFeatureCursor::~OGRFeatureCursor()
{
    if ( _nextHandleToQueue )
        OGR_F_Destroy( _nextHandleToQueue );

    if ( _dsHandle && _resultSetHandle != _layerHandle )
        OGR_DS_ReleaseResultSet( _dsHandle, _resultSetHandle );

    if ( _spatialFilter )
        OGR_G_DestroyGeometry( _spatialFilter );

    if ( _dsHandle )
        OGRReleaseDataSource( _dsHandle );
}

bool
OGR::OGRFeatureCursor::hasMore() const
{
    return _resultSetHandle && _queue.size() > 0;
}

Feature*
OGR::OGRFeatureCursor::nextFeature()
{
    if ( !hasMore() )
        return 0L;

    if ( _queue.size() == 1u )
        readChunk();

    // do this in order to hold a reference to the feature we return, so the caller
    // doesn't have to. This lets us avoid requiring the caller to use a ref_ptr when 
    // simply iterating over the cursor, making the cursor move conventient to use.
    _lastFeatureReturned = _queue.front();
    _queue.pop();

    return _lastFeatureReturned.get();
}

// reads a chunk of features into a memory cache; do this for performance
// and to avoid needing the OGR Mutex every time
void
OGR::OGRFeatureCursor::readChunk()
{
    if ( !_resultSetHandle )
        return;
    
    while( _queue.size() < _chunkSize && !_resultSetEndReached )
    {
        FeatureList filterList;
        while( filterList.size() < _chunkSize && !_resultSetEndReached )
        {
            OGRFeatureH handle = OGR_L_GetNextFeature( _resultSetHandle );
            if ( handle )
            {
                /*
                // Crop the geometry by the spatial filter.  Could be useful for tiling.
                if (_spatialFilter)
                {
                    OGRGeometryH geomRef = OGR_F_GetGeometryRef(handle);
                    OGRGeometryH intersection = OGR_G_Intersection(geomRef, _spatialFilter);
                    OGR_F_SetGeometry(handle, intersection);
                }
                */
                osg::ref_ptr<Feature> feature = OgrUtils::createFeature( handle, _profile.get(), _rewindPolygons);

                if (feature.valid())
                {
                    if (_source == NULL || !_source->isBlacklisted(feature->getFID()))
                    {
                        if (validateGeometry( feature->getGeometry() ))
                        {
                            filterList.push_back( feature.release() );
                        }
                        else
                        {
                            OE_DEBUG << LC << "Invalid geometry found at feature " << feature->getFID() << std::endl;
                        }
                    }
                    else
                    {
                        OE_DEBUG << LC << "Blacklisted feature " << feature->getFID() << " skipped" << std::endl;
                    }
                }
                else
                {
                    OE_DEBUG << LC << "Skipping NULL feature" << std::endl;
                }
                OGR_F_Destroy( handle );
            }
            else
            {
                _resultSetEndReached = true;
            }
        }

        // preprocess the features using the filter list:
        if ( _filters.valid() && !_filters->empty() )
        {
            FilterContext cx;
            cx.setProfile( _profile.get() );
            if (_query.bounds().isSet())
            {
                cx.extent() = GeoExtent(_profile->getSRS(), _query.bounds().get());
            }
            else
            {
                cx.extent() = _profile->getExtent();
            }

            for( FeatureFilterChain::const_iterator i = _filters->begin(); i != _filters->end(); ++i )
            {
                FeatureFilter* filter = i->get();
                cx = filter->push( filterList, cx );
            }
        }

        for(FeatureList::const_iterator i = filterList.begin(); i != filterList.end(); ++i)
        {
            _queue.push( i->get() );
        }
    }

    if (_chunkSize == ~0)
    {
        OGR_L_ResetReading(_resultSetHandle);
    }
}

//........................................................................

Config
OGRFeatureSource::Options::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    conf.set("url", _url);
    conf.set("connection", _connection);
    conf.set("ogr_driver", _ogrDriver);
    conf.set("build_spatial_index", _buildSpatialIndex);
    conf.set("force_rebuild_spatial_index", _forceRebuildSpatialIndex);
    conf.set("geometry", _geometryConfig);
    conf.set("geometry_url", _geometryUrl);
    conf.set("layer", _layer);
    conf.set("query", _query);
    return conf;
}

void
OGRFeatureSource::Options::fromConfig(const Config& conf)
{
    conf.get("url", _url);
    conf.get("connection", _connection);
    conf.get("ogr_driver", _ogrDriver);
    conf.get("build_spatial_index", _buildSpatialIndex);
    conf.get("force_rebuild_spatial_index", _forceRebuildSpatialIndex);
    conf.get("geometry", _geometryConfig);
    conf.get("geometry_url", _geometryUrl);
    conf.get("layer", _layer);
    conf.get("query", _query);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(ogrfeatures, OGRFeatureSource);

OE_LAYER_PROPERTY_IMPL(OGRFeatureSource, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(OGRFeatureSource, std::string, Connection, connection);
OE_LAYER_PROPERTY_IMPL(OGRFeatureSource, bool, BuildSpatialIndex, buildSpatialIndex);
OE_LAYER_PROPERTY_IMPL(OGRFeatureSource, std::string, OGRDriver, ogrDriver);
OE_LAYER_PROPERTY_IMPL(OGRFeatureSource, URI, GeometryURL, geometryUrl);
OE_LAYER_PROPERTY_IMPL(OGRFeatureSource, std::string, Layer, layer);
OE_LAYER_PROPERTY_IMPL(OGRFeatureSource, Query, Query, query);

void
OGRFeatureSource::init()
{
    FeatureSource::init();
    _dsHandle = 0L;
    _layerHandle = 0L;
    _ogrDriverHandle = 0L;
    _featureCount = -1;
    _needsSync = false;
    _writable = false;
    _geometryType = Geometry::TYPE_UNKNOWN;
}

Status
OGRFeatureSource::closeImplementation()
{
    if (_layerHandle)
    {
        if (_needsSync)
        {
            OGR_L_SyncToDisk(_layerHandle); // for writing only
            const char* name = OGR_FD_GetName(OGR_L_GetLayerDefn(_layerHandle));
            std::stringstream buf;
            buf << "REPACK " << name;
            std::string bufStr;
            bufStr = buf.str();
            OE_DEBUG << LC << "SQL: " << bufStr << std::endl;
            OGR_DS_ExecuteSQL(_dsHandle, bufStr.c_str(), 0L, 0L);
        }
        _layerHandle = 0L;
    }

    if (_dsHandle)
    {
        OGRReleaseDataSource(_dsHandle);
        _dsHandle = 0L;
    }

    init();

    return FeatureSource::closeImplementation();
}

OGRFeatureSource::~OGRFeatureSource()
{
    close();
}

Status
OGRFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    // Data source at a URL?
    if (options().url().isSet())
    {
        _source = options().url()->full();

        // ..inside a zip file?
        if (osgEarth::endsWith(_source, ".zip", false) || _source.find(".zip/") != std::string::npos)
        {
            _source = Stringify() << "/vsizip/" << _source;
        }
    }

    // ..or database connection?
    else if (options().connection().isSet())
    {
        _source = options().connection().value();
    }

    else if (!_geometry.valid())
    {
        // ..or inline geometry?
        _geometry =
            options().geometryConfig().isSet() ? parseGeometry(*options().geometryConfig()) :
            options().geometryUrl().isSet() ? parseGeometryUrl(*options().geometryUrl(), getReadOptions()) :
            0L;
    }

    // If nothing was set, we're done
    if (_source.empty() && !_geometry.valid())
    {
        return Status(Status::ConfigurationError, "No URL, connection, or inline geometry provided");
    }

    // Try to open the datasource and establish a feature profile.        
    FeatureProfile* featureProfile = 0L;

    // see if we have a custom profile.
    if (options().profile().isSet() && !_profile.valid())
    {
        _profile = Profile::create(*options().profile());
    }

    if (_geometry.valid())
    {
        // if the user specified explicit geometry, use that and the calculated
        // extent of the geometry to derive a profile.
        GeoExtent ex;
        if (_profile.valid())
        {
            ex = GeoExtent(_profile->getSRS(), _geometry->getBounds());
        }

        if (!ex.isValid())
        {
            // default to WGS84 Lat/Long
            osg::ref_ptr<const Profile> gg = Profile::create(Profile::GLOBAL_GEODETIC);
            ex = gg->getExtent();
        }

        featureProfile = new FeatureProfile(ex);
    }

    else if (!_source.empty())
    {
        // otherwise, assume we're loading from the URL/connection:

        // remember the thread so we don't use the handles illegaly.
        _dsHandleThreadId = osgEarth::Threading::getCurrentThreadId();

        // If the user request a particular driver, set that up now:
        std::string driverName = options().ogrDriver().value();
        
        const char* driverList[2] = {
            driverName.c_str(),
            nullptr
        };

        // always opening a vector source:
        int openFlags = GDAL_OF_VECTOR;

        // whether it's read-only or writable:
        if (options().openWrite().isSetTo(true))
            openFlags |= GDAL_OF_UPDATE;
        else
            openFlags |= GDAL_OF_READONLY;

        if (osgEarth::getNotifyLevel() >= osg::INFO)
            openFlags |= GDAL_OF_VERBOSE_ERROR;

        // this handle may ONLY be used from this thread!
        // https://github.com/OSGeo/gdal/blob/v2.4.1/gdal/gcore/gdaldataset.cpp#L2577
        _dsHandle = GDALOpenEx(
            _source.c_str(),
            openFlags,
            driverName.empty() ? nullptr : driverList,
            nullptr,
            nullptr);

        //int openMode = options().openWrite().isSet() && options().openWrite().value() ? 1 : 0;

        //_dsHandle = OGROpenShared(_source.c_str(), openMode, &_ogrDriverHandle);

        if (!_dsHandle)
        {
            return Status(Status::ResourceUnavailable, Stringify() << "Failed to open \"" << _source << "\"");
        }

        if (openFlags & GDAL_OF_UPDATE)
        //if (openMode == 1)
        {
            _writable = true;
        }

        // Open a specific layer within the data source, if applicable:
        _layerHandle = OGR::openLayer(_dsHandle, options().layer().value());
        if (!_layerHandle)
        {
            return Status(Status::ResourceUnavailable, Stringify() << "Failed to open layer \"" << options().layer().get() << "\" from \"" << _source << "\"");
        }

        // if the user provided a profile, use that:
        if (_profile.valid())
        {
            featureProfile = new FeatureProfile(_profile->getExtent());
        }

        // otherwise extract one from the layer:
        else
        {
            // extract the SRS and Extent:                
            OGRSpatialReferenceH srHandle = OGR_L_GetSpatialRef(_layerHandle);
            if (!srHandle)
            {
                return Status(Status::ResourceUnavailable, Stringify() << "No spatial reference found in \"" << _source << "\"");
            }

            osg::ref_ptr<SpatialReference> srs = SpatialReference::createFromHandle(srHandle);
            if (!srs.valid())
            {
                return Status(Status::ResourceUnavailable, Stringify() << "Unrecognized SRS found in \"" << _source << "\"");
            }

            // extract the full extent of the layer:
            OGREnvelope env;
            if (OGR_L_GetExtent(_layerHandle, &env, 1) != OGRERR_NONE)
            {
                return Status(Status::ResourceUnavailable, Stringify() << "Invalid extent returned from \"" << _source << "\"");
            }

            GeoExtent extent(srs.get(), env.MinX, env.MinY, env.MaxX, env.MaxY);
            if (!extent.isValid())
            {
                return Status(Status::ResourceUnavailable, Stringify() << "Invalid extent returned from \"" << _source << "\"");
            }

            // Made it!
            featureProfile = new FeatureProfile(extent);
        }


        // assuming we successfully opened the layer, build a spatial index if requested.
        if (options().buildSpatialIndex() == true)
        {
            if ((options().forceRebuildSpatialIndex() == true) || (OGR_L_TestCapability(_layerHandle, OLCFastSpatialFilter) == 0))
            {
                OE_INFO << LC << "Building spatial index for " << getName() << std::endl;
                std::stringstream buf;
                const char* name = OGR_FD_GetName(OGR_L_GetLayerDefn(_layerHandle));
                buf << "CREATE SPATIAL INDEX ON " << name;
                std::string bufStr;
                bufStr = buf.str();
                OE_DEBUG << LC << "SQL: " << bufStr << std::endl;
                OGR_DS_ExecuteSQL(_dsHandle, bufStr.c_str(), 0L, 0L);
            }
            else
            {
                OE_INFO << LC << "Use existing spatial index for " << getName() << std::endl;
            }
        }


        //Get the feature count
        _featureCount = OGR_L_GetFeatureCount(_layerHandle, 1);

        // establish the feature schema:
        initSchema();

        // establish the geometry type for this feature layer:
        OGRwkbGeometryType wkbType = OGR_FD_GetGeomType(OGR_L_GetLayerDefn(_layerHandle));
        if (
            wkbType == wkbPolygon ||
            wkbType == wkbPolygon25D)
        {
            _geometryType = Geometry::TYPE_POLYGON;
        }
        else if (
            wkbType == wkbLineString ||
            wkbType == wkbLineString25D)
        {
            _geometryType = Geometry::TYPE_LINESTRING;
        }
        else if (
            wkbType == wkbLinearRing)
        {
            _geometryType = Geometry::TYPE_RING;
        }
        else if (
            wkbType == wkbPoint ||
            wkbType == wkbPoint25D)
        {
            _geometryType = Geometry::TYPE_POINT;
        }
        else if (
            wkbType == wkbMultiPoint ||
            wkbType == wkbMultiPoint25D)
        {
            _geometryType = Geometry::TYPE_POINTSET;
        }
        else if (
            wkbType == wkbGeometryCollection ||
            wkbType == wkbGeometryCollection25D ||
            wkbType == wkbMultiLineString ||
            wkbType == wkbMultiLineString25D ||
            wkbType == wkbMultiPolygon ||
            wkbType == wkbMultiPolygon25D)
        {
            _geometryType = Geometry::TYPE_MULTI;
        }
    }

    // finally, if we're establish a profile, set it for this source.
    if (featureProfile)
    {
        if (options().geoInterp().isSet())
        {
            featureProfile->geoInterp() = options().geoInterp().get();
        }
        setFeatureProfile(featureProfile);
    }

    else
    {
        return Status(Status::ResourceUnavailable, "Failed to establish a valid feature profile");
    }

    OE_INFO << LC << getName() << " : opened OK" << std::endl;

    return Status::NoError;
}

const Status&
OGRFeatureSource::create(const FeatureProfile* profile,
                         const FeatureSchema& schema,
                         const Geometry::Type& geometryType,
                         const osgDB::Options* readOptions)
{
    if (FeatureSource::openImplementation().isError())
        return getStatus();

    profile = setFeatureProfile(profile);

    _schema = schema;

    // Data source at a URL?
    if (options().url().isSet())
    {
        _source = options().url()->full();

        // ..inside a zip file?
        if (osgEarth::endsWith(_source, ".zip", false) || _source.find(".zip/") != std::string::npos)
        {
            _source = Stringify() << "/vsizip/" << _source;
        }
    }
    // ..or database connection?
    else if (options().connection().isSet())
    {
        _source = options().connection().value();
    }

    // ..or inline geometry?
    _geometry =
        options().geometryConfig().isSet() ? parseGeometry(*options().geometryConfig()) :
        options().geometryUrl().isSet() ? parseGeometryUrl(*options().geometryUrl(), readOptions) :
        0L;

    // If nothing was set, we're done
    if (_source.empty() && !_geometry.valid())
    {
        setStatus(Status(Status::ConfigurationError, "No URL, connection, or inline geometry provided"));
        return getStatus();
    }

    std::string driverName = options().ogrDriver().value();
    if (driverName.empty())
        driverName = "ESRI Shapefile";

    _ogrDriverHandle = OGRGetDriverByName(driverName.c_str());

    _dsHandleThreadId = osgEarth::Threading::getCurrentThreadId();

    // this handle may ONLY be used from this thread!
    // https://github.com/OSGeo/gdal/blob/v2.4.1/gdal/gcore/gdaldataset.cpp#L2577
    _dsHandle = OGR_Dr_CreateDataSource(_ogrDriverHandle, _source.c_str(), NULL);

    if (!_dsHandle)
    {
        std::string msg = CPLGetLastErrorMsg();
        setStatus(Status(Status::ResourceUnavailable, Stringify() << "Failed to create \"" << _source << "\" ... " << msg));
        return getStatus();
    }

    OGRwkbGeometryType ogrGeomType = OgrUtils::getOGRGeometryType(geometryType);

    OGRSpatialReferenceH ogrSRS = profile->getSRS()->getHandle();

    _layerHandle = OGR_DS_CreateLayer(_dsHandle, "", ogrSRS, ogrGeomType, NULL);

    if (!_layerHandle)
    {
        setStatus(Status(Status::ResourceUnavailable, Stringify() << "Failed to create layer \"" << options().layer().get() << "\" from \"" << _source << "\""));
        return getStatus();
    }

    for (FeatureSchema::const_iterator i = _schema.begin(); i != _schema.end(); ++i)
    {
        OGRFieldType type =
            i->second == ATTRTYPE_DOUBLE ? OFTReal :
            i->second == ATTRTYPE_INT ? OFTInteger :
            OFTString;
        OGRFieldDefnH fdef = OGR_Fld_Create(i->first.c_str(), type);
        OGR_L_CreateField(_layerHandle, fdef, TRUE);
    }

    _featureCount = 0;

    _geometryType = geometryType;

    setStatus(Status::NoError);
    return getStatus();
}

void
OGRFeatureSource::buildSpatialIndex()
{
   if (_dsHandle &&
       _layerHandle && 
       OGR_L_TestCapability(_layerHandle, OLCFastSpatialFilter) == 0 &&
       _dsHandleThreadId == osgEarth::Threading::getCurrentThreadId())
   {
       std::stringstream buf;
       const char* name = OGR_FD_GetName(OGR_L_GetLayerDefn(_layerHandle));
       buf << "CREATE SPATIAL INDEX ON " << name;
       std::string bufStr;
       bufStr = buf.str();
       OGR_DS_ExecuteSQL(_dsHandle, bufStr.c_str(), 0L, 0L);
   }
}

FeatureCursor*
OGRFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress)
{
    if (_geometry.valid())
    {
        return new GeometryFeatureCursor(
            _geometry->clone(),
            getFeatureProfile(),
            getFilters());
    }
    else
    {
        OGRDataSourceH dsHandle = 0L;
        OGRLayerH layerHandle = 0L;

        const char* openOptions[2] = {
            "OGR_GPKG_INTEGRITY_CHECK=NO",
            nullptr
        };

        dsHandle = GDALOpenEx(
            _source.c_str(),
            GDAL_OF_VECTOR | GDAL_OF_READONLY,
            nullptr,
            nullptr, //openOptions,
            nullptr);

        // open the handles safely:
        // Each cursor requires its own DS handle so that multi-threaded access will work.
        // The cursor impl will dispose of the new DS handle.
        //dsHandle = OGROpenShared(_source.c_str(), 0, &_ogrDriverHandle);
        if (dsHandle)
        {
            layerHandle = OGR::openLayer(dsHandle, options().layer().get());
        }

        if (dsHandle && layerHandle)
        {
            Query newQuery(query);
            if (options().query().isSet())
            {
                newQuery = options().query()->combineWith(query);
            }

            OE_DEBUG << newQuery.getConfig().toJSON(true) << std::endl;

            // cursor is responsible for the OGR handles.
            return new OGR::OGRFeatureCursor(
                dsHandle,
                layerHandle,
                this,
                getFeatureProfile(),
                newQuery,
                getFilters(),
                _options->rewindPolygons().get(),
                0, // default chunksize
                progress
                );
        }
        else
        {
            if (dsHandle)
            {
                OGRReleaseDataSource(dsHandle);
            }

            return 0L;
        }
    }
}

bool
OGRFeatureSource::deleteFeature(FeatureID fid)
{
    if (_writable && _layerHandle)
    {
        if (OGR_L_DeleteFeature(_layerHandle, fid) == OGRERR_NONE)
        {
            _needsSync = true;
            return true;
        }
    }
    return false;
}

int
OGRFeatureSource::getFeatureCount() const
{
    return _featureCount;
}

bool
OGRFeatureSource::supportsGetFeature() const
{
    return true;
}

Feature*
OGRFeatureSource::getFeature(FeatureID fid)
{
    Feature* result = NULL;

    if (_layerHandle && !isBlacklisted(fid))
    {
        OGRFeatureH handle = OGR_L_GetFeature(_layerHandle, fid);
        if (handle)
        {
            result = OgrUtils::createFeature(handle, getFeatureProfile(), *_options->rewindPolygons());
            OGR_F_Destroy(handle);
        }
    }
    return result;
}

bool
OGRFeatureSource::isWritable() const
{
    return _writable;
}

const FeatureSchema&
OGRFeatureSource::getSchema() const
{
    return _schema;
}

bool
OGRFeatureSource::insertFeature(Feature* feature)
{
    OGRFeatureH feature_handle = OGR_F_Create(OGR_L_GetLayerDefn(_layerHandle));
    if (feature_handle)
    {
        const AttributeTable& attrs = feature->getAttrs();

        // assign the attributes:
        int num_fields = OGR_F_GetFieldCount(feature_handle);
        for (int i = 0; i < num_fields; i++)
        {
            OGRFieldDefnH field_handle_ref = OGR_F_GetFieldDefnRef(feature_handle, i);
            std::string name = OGR_Fld_GetNameRef(field_handle_ref);
            int field_index = OGR_F_GetFieldIndex(feature_handle, name.c_str());

            AttributeTable::const_iterator a = attrs.find(toLower(name));
            if (a != attrs.end())
            {
                switch (OGR_Fld_GetType(field_handle_ref))
                {
                case OFTInteger:
                    OGR_F_SetFieldInteger(feature_handle, field_index, a->second.getInt(0));
                    break;
                case OFTReal:
                    OGR_F_SetFieldDouble(feature_handle, field_index, a->second.getDouble(0.0));
                    break;
                case OFTString:
                    OGR_F_SetFieldString(feature_handle, field_index, a->second.getString().c_str());
                    break;
                default:break;
                }
            }
        }

        // assign the geometry:
        OGRFeatureDefnH def = ::OGR_L_GetLayerDefn(_layerHandle);

        OGRwkbGeometryType reported_type = OGR_FD_GetGeomType(def);

        OGRGeometryH ogr_geometry = OgrUtils::createOgrGeometry(feature->getGeometry(), reported_type);
        if (OGR_F_SetGeometryDirectly(feature_handle, ogr_geometry) != OGRERR_NONE)
        {
            OE_WARN << LC << "OGR_F_SetGeometryDirectly failed!" << std::endl;
        }

        OGRErr err = OGR_L_CreateFeature(_layerHandle, feature_handle);
        if (err != OGRERR_NONE)
        {
            //TODO: handle error better
            OE_WARN << LC << "OGR_L_CreateFeature failed! err=" << err << "; " << CPLGetLastErrorMsg() << std::endl;
            OGR_F_Destroy(feature_handle);
            return false;
        }

        // clean up the feature
        OGR_F_Destroy(feature_handle);
    }
    else
    {
        //TODO: handle error better
        OE_WARN << LC << "OGR_F_Create failed." << std::endl;
        return false;
    }

    dirty();

    return true;
}

osgEarth::Geometry::Type
OGRFeatureSource::getGeometryType() const
{
    return _geometryType;
}


// parses an explicit WKT geometry string into a Geometry.
Geometry*
OGRFeatureSource::parseGeometry(const Config& geomConf)
{
    return GeometryUtils::geometryFromWKT(geomConf.value(), *_options->rewindPolygons());
}

// read the WKT geometry from a URL, then parse into a Geometry.
Geometry*
OGRFeatureSource::parseGeometryUrl(const URI& geomUrl, const osgDB::Options* dbOptions)
{
    ReadResult r = geomUrl.readString(dbOptions);
    if (r.succeeded())
    {
        Config conf("geometry", r.getString());
        return parseGeometry(conf);
    }
    return 0L;
}

void
OGRFeatureSource::initSchema()
{
    OGRFeatureDefnH layerDef = OGR_L_GetLayerDefn(_layerHandle);
    for (int i = 0; i < OGR_FD_GetFieldCount(layerDef); i++)
    {
        OGRFieldDefnH fieldDef = OGR_FD_GetFieldDefn(layerDef, i);
        std::string name;
        name = std::string(OGR_Fld_GetNameRef(fieldDef));
        OGRFieldType ogrType = OGR_Fld_GetType(fieldDef);
        _schema[name] = OgrUtils::getAttributeType(ogrType);
    }
}
