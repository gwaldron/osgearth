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
#include <osgEarthUtil/WFS>
#include "WFSFeatureOptions"
#include "GeometryUtils"
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <stdio.h>

#include <ogr_api.h>

#define LC "[WFS FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

/**
 * A FeatureSource that reads features from a WFS layer
 *
 * This FeatureSource does NOT support styling.
 */
class WFSFeatureSource : public FeatureSource
{
public:
    WFSFeatureSource( const WFSFeatureOptions& options ) : FeatureSource( options ),      
      _options( options )
    {        
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~WFSFeatureSource()
    {               
    }

    //override
    void initialize( const std::string& referenceURI )
    {
        char sep = _options.url()->find_first_of('?') == std::string::npos? '?' : '&';

        std::string capUrl;
        if ( capUrl.empty() )
        {
            capUrl = 
                _options.url().value() + 
                sep + 
                "SERVICE=WFS&VERSION=1.0.0&REQUEST=GetCapabilities";
        }

        _capabilities = WFSCapabilitiesReader::read( capUrl, 0L );
        if ( !_capabilities.valid() )
        {
            OE_WARN << "[osgEarth::WFS] Unable to read WFS GetCapabilities." << std::endl;
            //return;
        }
        else
        {
            OE_NOTICE << "[osgEarth::WFS] Got capabilities from " << capUrl << std::endl;
        }
    }

    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        FeatureProfile* result = NULL;
        if (_capabilities.valid())
        {
            //Find the feature type by name
            osg::ref_ptr< WFSFeatureType > featureType = _capabilities->getFeatureTypeByName( _options.typeName().get() );
            if (featureType.valid())
            {
                if (featureType->getExtent().isValid())
                {
                    result = new FeatureProfile(featureType->getExtent());
                }
            }
        }

        if (!result)
        {
            result = new FeatureProfile(GeoExtent(SpatialReference::create( "epsg:4326" ), -180, -90, 180, 90));
        }
        return result;        
    }

    void saveResponse(HTTPResponse& response, const std::string& filename)
    {
        std::ofstream fout;
        fout.open(filename.c_str(), std::ios::out | std::ios::binary);

        std::istream& input_stream = response.getPartStream(0);
        input_stream.seekg (0, std::ios::end);
        int length = input_stream.tellg();
        input_stream.seekg (0, std::ios::beg);

        char *buffer = new char[length];
        input_stream.read(buffer, length);
        fout.write(buffer, length);
        delete[] buffer;
        fout.close();
    }

    Feature* createFeature( OGRFeatureH handle )
    {
        long fid = OGR_F_GetFID( handle );

        Feature* feature = new Feature( fid );

        OGRGeometryH geomRef = OGR_F_GetGeometryRef( handle );	
        if ( geomRef )
        {
            Symbology::Geometry* geom = GeometryUtils::createGeometry( geomRef );
            feature->setGeometry( geom );
        }

        int numAttrs = OGR_F_GetFieldCount(handle); 
        for (int i = 0; i < numAttrs; ++i) 
        { 
            void* field_handle_ref = OGR_F_GetFieldDefnRef( handle, i ); 
            const char* field_name = OGR_Fld_GetNameRef( field_handle_ref ); 
            const char* field_value= OGR_F_GetFieldAsString(handle, i); 
            std::string name = std::string( field_name ); 
            std::string value = std::string( field_value); 
            //Make the name lower case 
            std::transform( name.begin(), name.end(), name.begin(), ::tolower ); 
            feature->setAttr(name, value); 
        } 

        return feature;
    }

    void getFeatures(HTTPResponse &response, FeatureList& features)
    {
        //TODO:  Handle more than just geojson...
        std::string ext = ".json";
        //Save the response to a temp file            
        char *tmpname = tmpnam( NULL );
        std::string name(tmpname);
        name += ext;
        saveResponse(response, name );

        OGRDataSourceH ds = OGROpen(name.c_str(), FALSE, NULL);            
        if (!ds)
        {
            OE_NOTICE << "Error opening data with contents " << std::endl
                << response.getPartAsString(0) << std::endl;
        }

        OGRLayerH layer = OGR_DS_GetLayer(ds, 0);
        //Read all the features
        if (layer)
        {
            OGR_L_ResetReading(layer);                                
            OGRFeatureH feat_handle;
            while ((feat_handle = OGR_L_GetNextFeature( layer )) != NULL)
            {
                if ( feat_handle )
                {
                    Feature* f = createFeature( feat_handle );
                    if ( f ) 
                    {
                        features.push_back( f );
                    }
                    OGR_F_Destroy( feat_handle );
                }
            }
        }

        //Destroy the datasource
        OGR_DS_Destroy( ds );
        //Remove the temporary file
        remove( name.c_str() );            
    }

    std::string createURL(const Symbology::Query& query)
    {
        std::stringstream buf;
        buf << _options.url().get() << "?SERVICE=WFS&VERSION=1.0.0&REQUEST=getfeature";
        buf << "&TYPENAME=" << _options.typeName().get();
        
        std::string outputFormat = "geojson";
        if (_options.outputFormat().isSet()) outputFormat = _options.outputFormat().get();
        buf << "&OUTPUTFORMAT=" << outputFormat;

        if (_options.maxFeatures().isSet())
        {
            buf << "&MAXFEATURES=" << _options.maxFeatures().get();
        }

        if (query.bounds().isSet())
        {
            buf << "&BBOX=" << query.bounds().get().xMin() << "," << query.bounds().get().yMin() << ","
                            << query.bounds().get().xMax() << "," << query.bounds().get().yMax();
        }
        return buf.str();
    }

    //override
    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
    {
        std::string url = createURL( query );
        HTTPResponse response = HTTPClient::get(url);                
        if (response.isOK())
        {
            FeatureList features;
            getFeatures(response, features);            
            std::string data = response.getPartAsString(0);
            return new FeatureListCursor( features );
        }
        return 0L;
    }

private:
    const WFSFeatureOptions _options;  
    osg::ref_ptr< WFSCapabilities > _capabilities;
};


class WFSFeatureSourceFactory : public FeatureSourceDriver
{
public:
    WFSFeatureSourceFactory()
    {
        supportsExtension( "osgearth_feature_wfs", "WFS feature driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "WFS Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new WFSFeatureSource( getFeatureSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_wfs, WFSFeatureSourceFactory)

