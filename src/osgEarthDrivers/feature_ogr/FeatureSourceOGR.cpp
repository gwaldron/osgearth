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

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
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
#include <ogr_api.h>
#include <cpl_error.h>

#define LC "[OGR FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

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
      _layerIndex( 0 ),
      _ogrDriverHandle( 0L ),
      _options( options ),
      _featureCount(-1),
      _needsSync(false),
      _writable(false)
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

    //override
    void initialize( const osgDB::Options* dbOptions )
    {
        if ( _options.url().isSet() )
        {
            _source = _options.url()->full();
        }
        else if ( _options.connection().isSet() )
        {
            _source = _options.connection().value();
        }
        
        // establish the geometry:
        _geometry = 
            _options.geometry().valid()       ? _options.geometry().get() :
            _options.geometryConfig().isSet() ? parseGeometry( *_options.geometryConfig() ) :
            _options.geometryUrl().isSet()    ? parseGeometryUrl( *_options.geometryUrl(), dbOptions ) :
            0L;
    }

    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        FeatureProfile* result = 0L;

        // see if we have a custom profile.
        osg::ref_ptr<const Profile> profile;
        if ( _options.profile().isSet() )
        {
            profile = Profile::create( *_options.profile() );
        }

        if ( _geometry.valid() )
        {
            // if the user specified explicit geometry/profile, use that:
            GeoExtent ex;
            if ( profile.valid() )
            {
                ex = profile->getExtent();
            }

            if ( !ex.isValid() )
            {
                // default to WGS84 Lat/Long
                ex = osgEarth::Registry::instance()->getGlobalGeodeticProfile()->getExtent();
            }
            result = new FeatureProfile( ex );
        }

        else if ( !_source.empty() )
        {
            // otherwise, assume we're loading from the URL:
            OGR_SCOPED_LOCK;

            // load up the driver, defaulting to shapefile if unspecified.
            std::string driverName = _options.ogrDriver().value();
            if ( driverName.empty() )
                driverName = "ESRI Shapefile";
            _ogrDriverHandle = OGRGetDriverByName( driverName.c_str() );

            // attempt to open the dataset:
            int openMode = _options.openWrite().isSet() && _options.openWrite().value() ? 1 : 0;

            _dsHandle = OGROpen( _source.c_str(), openMode, &_ogrDriverHandle );
            if ( _dsHandle )
            {
                if (openMode == 1) _writable = true;
                
                if ( _options.layer().isSet() )
                {
                    _layerIndex = _options.layer().value();
                }                

                _layerHandle = OGR_DS_GetLayer( _dsHandle, _layerIndex );
                if ( _layerHandle )
                {                                     
                    GeoExtent extent;

                    // if the user provided a profile, user that:
                    if ( profile.valid() )
                    {
                        result = new FeatureProfile( profile->getExtent() );
                    }

                    else
                    {
                        // extract the SRS and Extent:                
                        OGRSpatialReferenceH srHandle = OGR_L_GetSpatialRef( _layerHandle );
                        if ( srHandle )
                        {
                            osg::ref_ptr<SpatialReference> srs = SpatialReference::createFromHandle( srHandle, false );
                            if ( srs.valid() )
                            {
                                // extract the full extent of the layer:
                                OGREnvelope env;
                                if ( OGR_L_GetExtent( _layerHandle, &env, 1 ) == OGRERR_NONE )
                                {
                                    GeoExtent extent( srs.get(), env.MinX, env.MinY, env.MaxX, env.MaxY );
                                    
                                    // got enough info to make the profile!
                                    result = new FeatureProfile( extent );
                                }
                            }
                        }
                    }

                    // assuming we successfully opened the layer, build a spatial index if requested.
                    if ( _options.buildSpatialIndex() == true )
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

                    //Get the feature count
                    _featureCount = OGR_L_GetFeatureCount( _layerHandle, 1 );

                    initSchema();

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
            }
            else
            {
                OE_INFO << LC << "failed to open dataset at \"" << _source << "\" error " << CPLGetLastErrorMsg() << std::endl;
            }
        }
        else
        {
            OE_INFO << LC 
                << "Feature Source: no valid source data available" << std::endl;
        }

        return result;
    }


    //override
    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
    {
        if ( _geometry.valid() )
        {
            return new GeometryFeatureCursor(
                _geometry.get(),
                getFeatureProfile(),
                _options.filters() );
                //getFilters() );
        }
        else
        {
            OGR_SCOPED_LOCK;

            // Each cursor requires its own DS handle so that multi-threaded access will work.
            // The cursor impl will dispose of the new DS handle.

            OGRDataSourceH dsHandle = OGROpenShared( _source.c_str(), 0, &_ogrDriverHandle );
            if ( dsHandle )
            {
                OGRLayerH layerHandle = OGR_DS_GetLayer( dsHandle, _layerIndex );

                return new FeatureCursorOGR( 
                    dsHandle,
                    layerHandle, 
                    this,
                    getFeatureProfile(),
                    query, 
                    _options.filters() );
            }
            else
            {
                return 0L;
            }
        }
    }

    virtual bool deleteFeature(FeatureID fid)
    {
        if (_writable && _layerHandle)
        {
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

    virtual Feature* getFeature( FeatureID fid )
    {
        Feature* result = NULL;

        if ( !isBlacklisted(fid) )
        {
            OGR_SCOPED_LOCK;
            OGRFeatureH handle = OGR_L_GetFeature( _layerHandle, fid);
            if (handle)
            {
                const FeatureProfile* p = getFeatureProfile();
                const SpatialReference* srs = p ? p->getSRS() : 0L;
                result = OgrUtils::createFeature( handle, srs );
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
    unsigned int _layerIndex;
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

    virtual const char* className()
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

