/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Filter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ScaleFilter>
#include <osgEarthFeatures/GeometryUtils>
#include "OGRFeatureOptions"
#include "FeatureCursorOGR"
#include <osgEarthFeatures/OgrUtils>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <gdal_priv.h>
#include <ogr_api.h>
#include <cpl_error.h>

#define LC "[OGR FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

namespace
{
    // helper function.
    OGRLayerH openLayer(OGRDataSourceH ds, const std::string& layer)
    {
        OGRLayerH h = OGR_DS_GetLayerByName(ds, layer.c_str());
        if ( !h )
        {
            unsigned index = osgEarth::as<unsigned>(layer, 0);
            h = OGR_DS_GetLayer(ds, index);
        }
        return h;
    }
}


/**
 * A FeatureSource that reads features from an OGR driver.
 *
 * This FeatureSource does NOT support styling.
 */
class OGRFeatureSource : public FeatureSource
{
public:
    OGRFeatureSource( const OGRFeatureOptions& options ) : FeatureSource( options ),
      _dsHandle( 0L ),
      _layerHandle( 0L ),
      _ogrDriverHandle( 0L ),
      _options( options ),
      _featureCount(-1),
      _needsSync(false),
      _writable(false),
      _geometryType(Geometry::TYPE_UNKNOWN)
    {
        //nop
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~OGRFeatureSource()
    {       
        OGR_SCOPED_LOCK;

        if ( _layerHandle )
        {
            if (_needsSync)
            {
                OGR_L_SyncToDisk( _layerHandle ); // for writing only
                const char* name = OGR_FD_GetName( OGR_L_GetLayerDefn( _layerHandle ) );
                std::stringstream buf;
                buf << "REPACK " << name; 
                std::string bufStr;
                bufStr = buf.str();
                OE_DEBUG << LC << "SQL: " << bufStr << std::endl;
                OGR_DS_ExecuteSQL( _dsHandle, bufStr.c_str(), 0L, 0L );
            }
            _layerHandle = 0L;
        }

        if ( _dsHandle )
        {
            OGRReleaseDataSource( _dsHandle );
            _dsHandle = 0L;
        }
    }

    const Status& create(const FeatureProfile* profile,
                          const FeatureSchema& schema,
                          const Geometry::Type& geometryType,
                          const osgDB::Options* dbOptions)
    {
        setFeatureProfile(profile);
        _schema = schema;

        // Data source at a URL?
        if (_options.url().isSet())
        {
            _source = _options.url()->full();

            // ..inside a zip file?
            if (osgEarth::endsWith(_source, ".zip", false) || _source.find(".zip/") != std::string::npos)
            {
                _source = Stringify() << "/vsizip/" << _source;
            }
        }
        // ..or database connection?
        else if (_options.connection().isSet())
        {
            _source = _options.connection().value();
        }

        // ..or inline geometry?
        _geometry =
            _options.geometry().valid() ? _options.geometry().get() :
            _options.geometryConfig().isSet() ? parseGeometry(*_options.geometryConfig()) :
            _options.geometryUrl().isSet() ? parseGeometryUrl(*_options.geometryUrl(), dbOptions) :
            0L;

        // If nothing was set, we're done
        if (_source.empty() && !_geometry.valid())
        {
            setStatus(Status(Status::ConfigurationError, "No URL, connection, or inline geometry provided"));
            return getStatus();
        }

        std::string driverName = _options.ogrDriver().value();
        if (driverName.empty())
            driverName = "ESRI Shapefile";

        _ogrDriverHandle = OGRGetDriverByName(driverName.c_str());

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
            setStatus(Status(Status::ResourceUnavailable, Stringify() << "Failed to create layer \"" << _options.layer().get() << "\" from \"" << _source << "\""));
            return getStatus();
        }

        for(FeatureSchema::const_iterator i = _schema.begin(); i != _schema.end(); ++i)
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

        return getStatus();
    }

    //override
    Status initialize(const osgDB::Options* dbOptions)
    {
        // Data source at a URL?
        if ( _options.url().isSet() )
        {
            _source = _options.url()->full();

            // ..inside a zip file?
            if (osgEarth::endsWith(_source, ".zip", false) || _source.find(".zip/") != std::string::npos)
            {
                _source = Stringify() << "/vsizip/" << _source;
            }
        }

        // ..or database connection?
        else if ( _options.connection().isSet() )
        {
            _source = _options.connection().value();
        }
        
        // ..or inline geometry?
        _geometry = 
            _options.geometry().valid()       ? _options.geometry().get() :
            _options.geometryConfig().isSet() ? parseGeometry( *_options.geometryConfig() ) :
            _options.geometryUrl().isSet()    ? parseGeometryUrl( *_options.geometryUrl(), dbOptions ) :
            0L;

        // If nothing was set, we're done
        if (_source.empty() && !_geometry.valid())
        {
            return Status::Error(Status::ConfigurationError, "No URL, connection, or inline geometry provided");
        }

        // Try to open the datasource and establish a feature profile.        
        FeatureProfile* featureProfile = 0L;

        // see if we have a custom profile.
        osg::ref_ptr<const Profile> profile;
        if ( _options.profile().isSet() )
        {
            profile = Profile::create( *_options.profile() );
        }

        if ( _geometry.valid() )
        {
            // if the user specified explicit geometry, use that and the calculated
            // extent of the geometry to derive a profile.
            GeoExtent ex;
            if ( profile.valid() )
            {                
                ex = GeoExtent(profile->getSRS(), _geometry->getBounds());
            }

            if ( !ex.isValid() )
            {
                // default to WGS84 Lat/Long
                ex = osgEarth::Registry::instance()->getGlobalGeodeticProfile()->getExtent();
            }

            featureProfile = new FeatureProfile( ex );
        }

        else if ( !_source.empty() )
        {
            // otherwise, assume we're loading from the URL/connection:
            OGR_SCOPED_LOCK;

            // load up the driver, defaulting to shapefile if unspecified.
            std::string driverName = _options.ogrDriver().value();
            if ( driverName.empty() )
                driverName = "ESRI Shapefile";

            _ogrDriverHandle = OGRGetDriverByName( driverName.c_str() );

            // attempt to open the dataset:
            int openMode = _options.openWrite().isSet() && _options.openWrite().value() ? 1 : 0;

            _dsHandle = OGROpenShared( _source.c_str(), openMode, &_ogrDriverHandle );

            if ( !_dsHandle )
            {
                std::string msg = CPLGetLastErrorMsg();
                return Status::Error(Status::ResourceUnavailable, Stringify() << "Failed to open \"" << _source << "\" ... " << msg);
            }

            if (openMode == 1)
            {
                _writable = true;
            }
                
            // Open a specific layer within the data source, if applicable:
            if (!_layerHandle)
            {
                _layerHandle = openLayer(_dsHandle, _options.layer().value());
            }

            if (!_layerHandle)
            {
                return Status::Error(Status::ResourceUnavailable, Stringify() << "Failed to open layer \"" << _options.layer().get() << "\" from \"" << _source << "\"");
            }


            // if the user provided a profile, use that:
            if ( profile.valid() )
            {
                featureProfile = new FeatureProfile( profile->getExtent() );
            }

            // otherwise extract one from the layer:
            else
            {
                // extract the SRS and Extent:                
                OGRSpatialReferenceH srHandle = OGR_L_GetSpatialRef( _layerHandle );
                if (!srHandle)
                    return Status::Error(Status::ResourceUnavailable, Stringify() << "No spatial reference found in \"" << _source << "\"");

                osg::ref_ptr<SpatialReference> srs = SpatialReference::createFromHandle(srHandle);
                if (!srs.valid())
                    return Status::Error(Status::ResourceUnavailable, Stringify() << "Unrecognized SRS found in \"" << _source << "\"");

                // extract the full extent of the layer:
                OGREnvelope env;
                if ( OGR_L_GetExtent( _layerHandle, &env, 1 ) != OGRERR_NONE )
                    return Status::Error(Status::ResourceUnavailable, Stringify() << "Invalid extent returned from \"" << _source << "\"");

                GeoExtent extent( srs.get(), env.MinX, env.MinY, env.MaxX, env.MaxY );
                if ( !extent.isValid() )
                    return Status::Error(Status::ResourceUnavailable, Stringify() << "Invalid extent returned from \"" << _source << "\"");

                // Made it!
                featureProfile = new FeatureProfile(extent);
            }


            // assuming we successfully opened the layer, build a spatial index if requested.
            if ( _options.buildSpatialIndex() == true )
            {
                if ( (_options.forceRebuildSpatialIndex() == true) || (OGR_L_TestCapability(_layerHandle, OLCFastSpatialFilter) == 0) )
                {
                    OE_INFO << LC << "Building spatial index for " << getName() << std::endl;
                    std::stringstream buf;
                    const char* name = OGR_FD_GetName( OGR_L_GetLayerDefn( _layerHandle ) );
                    buf << "CREATE SPATIAL INDEX ON " << name;
                    std::string bufStr;
                    bufStr = buf.str();
                    OE_DEBUG << LC << "SQL: " << bufStr << std::endl;
                    OGR_DS_ExecuteSQL( _dsHandle, bufStr.c_str(), 0L, 0L );
                }
                else
                {
                    OE_INFO << LC << "Use existing spatial index for " << getName() << std::endl;
                }
            }


            //Get the feature count
            _featureCount = OGR_L_GetFeatureCount( _layerHandle, 1 );

            // establish the feature schema:
            initSchema();

            // establish the geometry type for this feature layer:
            OGRwkbGeometryType wkbType = OGR_FD_GetGeomType( OGR_L_GetLayerDefn( _layerHandle ) );
            if (
                wkbType == wkbPolygon ||
                wkbType == wkbPolygon25D )
            {
                _geometryType = Geometry::TYPE_POLYGON;
            }
            else if (
                wkbType == wkbLineString ||
                wkbType == wkbLineString25D )
            {
                _geometryType = Geometry::TYPE_LINESTRING;
            }
            else if (
                wkbType == wkbLinearRing )
            {
                _geometryType = Geometry::TYPE_RING;
            }
            else if ( 
                wkbType == wkbPoint ||
                wkbType == wkbPoint25D )
            {
                _geometryType = Geometry::TYPE_POINTSET;
            }
            else if (
                wkbType == wkbGeometryCollection ||
                wkbType == wkbGeometryCollection25D ||
                wkbType == wkbMultiPoint ||
                wkbType == wkbMultiPoint25D ||
                wkbType == wkbMultiLineString ||
                wkbType == wkbMultiLineString25D ||
                wkbType == wkbMultiPolygon ||
                wkbType == wkbMultiPolygon25D )
            {
                _geometryType = Geometry::TYPE_MULTI;
            }
        }

        // finally, if we're establish a profile, set it for this source.
        if ( featureProfile )
        {
            if ( _options.geoInterp().isSet() )
            {
                featureProfile->geoInterp() = _options.geoInterp().get();
            }
            setFeatureProfile(featureProfile);
        }

        else
        {
            return Status::Error(Status::ResourceUnavailable, "Failed to establish a valid feature profile");
        }

        return Status::OK();
    }

    //override
    FeatureCursor* createFeatureCursor(const Symbology::Query& query, ProgressCallback* progress)
    {
        if ( _geometry.valid() )
        {
            return new GeometryFeatureCursor(
                _geometry.get(),
                getFeatureProfile(),
                getFilters() );
        }
        else
        {
            OGRDataSourceH dsHandle = 0L;
            OGRLayerH layerHandle = 0L;

            // open the handles safely:
            {
                OGR_SCOPED_LOCK;

                // Each cursor requires its own DS handle so that multi-threaded access will work.
                // The cursor impl will dispose of the new DS handle.
                dsHandle = OGROpenShared( _source.c_str(), 0, &_ogrDriverHandle );
                if ( dsHandle )
                {
                    layerHandle = openLayer(dsHandle, _options.layer().get());
                }
            }

            if ( dsHandle && layerHandle )
            {
                Query newQuery(query);
                if (_options.query().isSet())
                {
                    newQuery = _options.query()->combineWith(query);
                }

                OE_DEBUG << newQuery.getConfig().toJSON(true) << std::endl;

                // cursor is responsible for the OGR handles.
                return new FeatureCursorOGR( 
                    dsHandle,
                    layerHandle, 
                    this,
                    getFeatureProfile(),
                    newQuery,
                    getFilters(),
                    progress);
            }
            else
            {
                if ( dsHandle )
                {
                    OGR_SCOPED_LOCK;
                    OGRReleaseDataSource( dsHandle );
                }

                return 0L;
            }
        }
    }

    virtual bool deleteFeature(FeatureID fid)
    {
        if (_writable && _layerHandle)
        {
            OGR_SCOPED_LOCK;
            if (OGR_L_DeleteFeature( _layerHandle, fid ) == OGRERR_NONE)
            {
                _needsSync = true;
                return true;
            }            
        }
        return false;
    }

    virtual int getFeatureCount() const
    {
        return _featureCount;
    }

    bool supportsGetFeature() const
    {
        return true;
    }

    virtual Feature* getFeature( FeatureID fid )
    {
        Feature* result = NULL;

        if ( _layerHandle && !isBlacklisted(fid) )
        {
            OGR_SCOPED_LOCK;
            OGRFeatureH handle = OGR_L_GetFeature( _layerHandle, fid);
            if (handle)
            {
                result = OgrUtils::createFeature( handle, getFeatureProfile() );
                OGR_F_Destroy( handle );
            }
        }
        return result;
    }

    virtual bool isWritable() const
    {
        return _writable;
    }

    const FeatureSchema& getSchema() const
    {
        return _schema;
    } 

    virtual bool insertFeature(Feature* feature)
    {
        OGR_SCOPED_LOCK;
        OGRFeatureH feature_handle = OGR_F_Create( OGR_L_GetLayerDefn( _layerHandle ) );
        if ( feature_handle )
        {
            const AttributeTable& attrs = feature->getAttrs();

            // assign the attributes:
            int num_fields = OGR_F_GetFieldCount( feature_handle );
            for( int i=0; i<num_fields; i++ )
            {
                OGRFieldDefnH field_handle_ref = OGR_F_GetFieldDefnRef( feature_handle, i );
                std::string name = OGR_Fld_GetNameRef( field_handle_ref );
                int field_index = OGR_F_GetFieldIndex( feature_handle, name.c_str() );

                AttributeTable::const_iterator a = attrs.find( toLower(name) );
                if ( a != attrs.end() )
                {
                    switch( OGR_Fld_GetType(field_handle_ref) )
                    {
                    case OFTInteger:
                        OGR_F_SetFieldInteger( feature_handle, field_index, a->second.getInt(0) );
                        break;
                    case OFTReal:
                        OGR_F_SetFieldDouble( feature_handle, field_index, a->second.getDouble(0.0) );
                        break;
                    case OFTString:
                        OGR_F_SetFieldString( feature_handle, field_index, a->second.getString().c_str() );
                        break;
                    default:break;
                    }
                }
            }

            // assign the geometry:
            OGRFeatureDefnH def = ::OGR_L_GetLayerDefn( _layerHandle );

            OGRwkbGeometryType reported_type = OGR_FD_GetGeomType( def );

            OGRGeometryH ogr_geometry = OgrUtils::createOgrGeometry( feature->getGeometry(), reported_type );
            if ( OGR_F_SetGeometryDirectly( feature_handle, ogr_geometry ) != OGRERR_NONE )
            {
                OE_WARN << LC << "OGR_F_SetGeometryDirectly failed!" << std::endl;
            }

            if ( OGR_L_CreateFeature( _layerHandle, feature_handle ) != OGRERR_NONE )
            {
                //TODO: handle error better
                OE_WARN << LC << "OGR_L_CreateFeature failed!" << std::endl;
                OGR_F_Destroy( feature_handle );
                return false;
            }

            // clean up the feature
            OGR_F_Destroy( feature_handle );
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

    virtual osgEarth::Symbology::Geometry::Type getGeometryType() const
    {
        return _geometryType;
    }

protected:

    // parses an explicit WKT geometry string into a Geometry.
    Symbology::Geometry* parseGeometry( const Config& geomConf )
    {
        return GeometryUtils::geometryFromWKT( geomConf.value() );
    }

    // read the WKT geometry from a URL, then parse into a Geometry.
    Symbology::Geometry* parseGeometryUrl( const std::string& geomUrl, const osgDB::Options* dbOptions )
    {
        ReadResult r = URI(geomUrl).readString( dbOptions );
        if ( r.succeeded() )
        {
            Config conf( "geometry", r.getString() );
            return parseGeometry( conf );
        }
        return 0L;
    }

    void initSchema()
    {
        OGRFeatureDefnH layerDef =  OGR_L_GetLayerDefn( _layerHandle );
        for (int i = 0; i < OGR_FD_GetFieldCount( layerDef ); i++)
        {
            OGRFieldDefnH fieldDef = OGR_FD_GetFieldDefn( layerDef, i );
            std::string name;
            name = std::string( OGR_Fld_GetNameRef( fieldDef ) );
            OGRFieldType ogrType = OGR_Fld_GetType( fieldDef );
            _schema[ name ] = OgrUtils::getAttributeType( ogrType );
        }
    }





private:
    std::string _source;
    OGRDataSourceH _dsHandle;
    OGRLayerH _layerHandle;
    OGRSFDriverH _ogrDriverHandle;
    osg::ref_ptr<Symbology::Geometry> _geometry; // explicit geometry.
    const OGRFeatureOptions _options;
    int _featureCount;
    bool _needsSync;
    bool _writable;
    FeatureSchema _schema;
    Geometry::Type _geometryType;
};


class OGRFeatureSourceFactory : public FeatureSourceDriver
{
public:
    OGRFeatureSourceFactory()
    {
        supportsExtension( "osgearth_feature_ogr", "OGR feature driver for osgEarth" );
    }

    virtual const char* className() const
    {
        return "OGR Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new OGRFeatureSource( getFeatureSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_ogr, OGRFeatureSourceFactory)

