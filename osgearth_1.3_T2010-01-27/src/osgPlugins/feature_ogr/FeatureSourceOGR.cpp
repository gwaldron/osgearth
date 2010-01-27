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

#include <osgEarth/Registry>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Filter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ScaleFilter>
#include "FeatureCursorOGR"
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <ogr_api.h>

using namespace osgEarth;
using namespace osgEarth::Features;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

#define PROP_URL                     "url"
#define PROP_OGR_DRIVER              "ogr_driver"
#define PROP_OGR_BUILD_SPATIAL_INDEX "build_spatial_index"

/**
 * A FeatureSource that reads features from an OGR driver.
 *
 * This FeatureSource does NOT support styling.
 */
class FeatureSourceOGR : public FeatureSource
{
public:
    FeatureSourceOGR( const PluginOptions* options ) : FeatureSource( options ),
      _ready( false ),
      _dsHandle( 0L ),
      _layerHandle( 0L ),
      _supportsRandomRead( false ),
      _buildSpatialIndex( false )
    {
        const Config& conf = getOptions()->config();

        _url = conf.value( PROP_URL );
        _ogrDriver = conf.value( PROP_OGR_DRIVER );
        _buildSpatialIndex = conf.value( PROP_OGR_BUILD_SPATIAL_INDEX ) == "true";
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~FeatureSourceOGR()
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

    void initialize( const std::string& referenceURI, const Profile* overrideProfile )
    {
        // never called????
    }

    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    FeatureProfile* createFeatureProfile()
    {
        FeatureProfile* result = 0L;

        OGR_SCOPED_LOCK;

        OGRSFDriverH ogrDriverHandle = 0L;
        if ( !_ogrDriver.empty() )
        {
            ogrDriverHandle = OGRGetDriverByName( _ogrDriver.c_str() );
        }

	    _dsHandle = OGROpenShared( _url.c_str(), 0, &ogrDriverHandle ); // last param is the driver spec..TODO!!
	    if ( _dsHandle )
	    {
		    _layerHandle = OGR_DS_GetLayer( _dsHandle, 0 ); // default to layer 0 for now
            if ( _layerHandle )
            {
                _supportsRandomRead = OGR_L_TestCapability( _layerHandle, OLCRandomRead ) == TRUE;

                // WARN the user if we load an ESRI-style LCC SRS, in which the PROJECTION["Lambert_Conformal_Conic"]
                // should really be Lambert_Conformal_Conic_1SP or _2SP.
                //OGR_SpatialReference* ogr_srs = dynamic_cast<OGR_SpatialReference*>( getSRS() );
                //if ( ogr_srs )
                //{
                //    if ( ogr_srs->getAttrValue( "PROJECTION", 0 ) == "Lambert_Conformal_Conic" )
                //    {
                //        osgGIS::notify(osg::WARN)
                //            << std::endl
                //            << "***WANRING*** SRS has an invalid ESRI-style LCC projection ... transformations may not work"
                //            << std::endl
                //            << "SRS = " << ogr_srs->getWKT() << std::endl
                //            << std::endl;
                //    }
                //}           
                
                // read the data extent.
                GeoExtent extent;


                // extract the SRS and Extent:                
                OGRSpatialReferenceH srHandle = OGR_L_GetSpatialRef( _layerHandle );
                if ( srHandle )
                {
                    osg::ref_ptr<SpatialReference> srs = SpatialReference::createFromHandle( srHandle );
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
                if ( _buildSpatialIndex )
                {
                    osg::notify(osg::NOTICE) <<
                        "[osgEarth] Building spatial index for " << getName() << " ..." << std::flush;

                    std::stringstream buf;
                    const char* name = OGR_FD_GetName( OGR_L_GetLayerDefn( _layerHandle ) );
                    buf << "CREATE SPATIAL INDEX ON " << name; 
					std::string bufStr;
					bufStr = buf.str();
                    OGR_DS_ExecuteSQL( _dsHandle, bufStr.c_str(), 0L, 0L );

                    osg::notify(osg::NOTICE) <<  "done." << std::endl;
                }
            }
	    }

        return result;
    }


    //override
    FeatureCursor* createFeatureCursor( const Query& query )
    {
        return new FeatureCursorOGR( 
            _dsHandle, 
            _layerHandle, 
            getFeatureProfile(),
            query, 
            getFilters() );
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

private:
    std::string _url;
    std::string _ogrDriver;
    OGRDataSourceH _dsHandle;
    OGRLayerH _layerHandle;
    bool _supportsRandomRead;
    bool _ready;
    bool _buildSpatialIndex;
};


class ReaderWriterFeatureSourceOGR : public osgDB::ReaderWriter
{
public:
    ReaderWriterFeatureSourceOGR()
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

        return new FeatureSourceOGR( static_cast<const PluginOptions*>(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_ogr, ReaderWriterFeatureSourceOGR)

