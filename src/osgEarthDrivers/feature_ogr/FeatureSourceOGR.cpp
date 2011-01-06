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

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Filter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ScaleFilter>
#include "OGRFeatureOptions"
#include "FeatureCursorOGR"
#include "GeometryUtils"
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <ogr_api.h>

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
    OGRFeatureSource( const FeatureSourceOptions& options ) : FeatureSource( options ),
      _dsHandle( 0L ),
      _layerHandle( 0L ),
      _ogrDriverHandle( 0L ),
      _options( options )
    {

        _geometry = 
            _options.geometry().valid() ? _options.geometry().get() :
            _options.geometryConfig().isSet() ? parseGeometry( _options.geometryConfig().value() ) :
            0L;
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~OGRFeatureSource()
    {       
        OGR_SCOPED_LOCK;

        if ( _layerHandle )
        {
            // OGR_L_SyncToDisk( _layerHandle ); // for writing only
            _layerHandle = 0L;
        }

        if ( _dsHandle )
        {
            OGRReleaseDataSource( _dsHandle );
            _dsHandle = 0L;
        }
    }

    //override
    void initialize( const std::string& referenceURI )
    {
        if ( _options.url().isSet() )
        {
            _absUrl = osgEarth::getFullPath( referenceURI, _options.url().value() );
        }
    }

    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        FeatureProfile* result = 0L;

        if ( _geometry.valid() )
        {
            // if the user specified explicit geometry/profile, use that:
            GeoExtent ex;
            if ( _options.geometryProfileOptions().isSet() )
            {
                osg::ref_ptr<const Profile> _profile = Profile::create( 
                    ProfileOptions(_options.geometryProfileOptions().value())  );

                if ( _profile.valid() )
                    ex = _profile->getExtent();
            }

            if ( !ex.isValid() )
            {
                // default to WGS84 Lat/Long
                ex = osgEarth::Registry::instance()->getGlobalGeodeticProfile()->getExtent();
            }
            result = new FeatureProfile( ex );
        }
        else if ( !_absUrl.empty() )
        {
            // otherwise, assume we're loading from the URL:
            OGR_SCOPED_LOCK;

            // load up the driver, defaulting to shapefile if unspecified.
            std::string driverName = _options.ogrDriver().value();
            if ( driverName.empty() )
                driverName = "ESRI Shapefile";
            _ogrDriverHandle = OGRGetDriverByName( driverName.c_str() );

            // attempt to open the dataset:
	        _dsHandle = OGROpenShared( _absUrl.c_str(), 0, &_ogrDriverHandle );
	        if ( _dsHandle )
	        {
		        _layerHandle = OGR_DS_GetLayer( _dsHandle, 0 ); // default to layer 0 for now
                if ( _layerHandle )
                {
                    GeoExtent extent;

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

                    // assuming we successfully opened the layer, build a spatial index if requested.
                    if ( _options.buildSpatialIndex() == true )
                    {
                        OE_INFO << LC << "Building spatial index for " << getName() << " ..." << std::flush;

                        std::stringstream buf;
                        const char* name = OGR_FD_GetName( OGR_L_GetLayerDefn( _layerHandle ) );
                        buf << "CREATE SPATIAL INDEX ON " << name; 
					    std::string bufStr;
					    bufStr = buf.str();
                        OGR_DS_ExecuteSQL( _dsHandle, bufStr.c_str(), 0L, 0L );

                        OE_INFO << LC << "...done." << std::endl;
                    }
                }
	        }
            else
            {
                OE_INFO << LC << "failed to open dataset at " << _absUrl << std::endl;
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

	        OGRDataSourceH dsHandle = OGROpenShared( _absUrl.c_str(), 0, &_ogrDriverHandle );
	        if ( dsHandle )
	        {
                OGRLayerH layerHandle = OGR_DS_GetLayer( dsHandle, 0 );

                return new FeatureCursorOGR( 
                    dsHandle,
                    layerHandle, 
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

protected:

    // closes any open OGR objects and releases the handles
    bool cleanup()
    {
        OGR_SCOPED_LOCK;

        if ( _layerHandle )
        {
            // OGR_L_SyncToDisk( _layerHandle ); // for writing only
            _layerHandle = 0L;
        }

        if ( _dsHandle )
        {
            OGRReleaseDataSource( _dsHandle );
            _dsHandle = 0L;
        }

        return true;
    }

    // parses an explicit WKT geometry string into a Geometry.
    Symbology::Geometry* parseGeometry( const Config& geomConf )
    {
        return GeometryUtils::createGeometryFromWKT( geomConf.value() );
    }

private:
    std::string _absUrl;
    OGRDataSourceH _dsHandle;
    OGRLayerH _layerHandle;
    OGRSFDriverH _ogrDriverHandle;
    osg::ref_ptr<Symbology::Geometry> _geometry; // explicit geometry.
    const OGRFeatureOptions _options;
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

