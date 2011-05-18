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
#include <osgEarthFeatures/OgrUtils>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <stdio.h>
#include <stdlib.h>

#include <ogr_api.h>

#ifdef WIN32
#include <windows.h>
#endif

#define LC "[WFS FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

std::string getTempPath()
{
#if defined(WIN32)  && !defined(__CYGWIN__)
    BOOL fSuccess  = FALSE;

    TCHAR lpTempPathBuffer[MAX_PATH];    

    //  Gets the temp path env string (no guarantee it's a valid path).
    DWORD dwRetVal = ::GetTempPath(MAX_PATH,          // length of the buffer
                                   lpTempPathBuffer); // buffer for path     

    if (dwRetVal > MAX_PATH || (dwRetVal == 0))
    {
        OE_NOTICE << "GetTempPath failed" << std::endl;
        return ".";
    }

    return std::string(lpTempPathBuffer);
#else
    return "/tmp/";
#endif
}

std::string getTempName(const std::string& prefix="", const std::string& suffix="")
{
    //tmpname is kind of busted on Windows, it always returns a file of the form \blah which gets put in your root directory but
    //oftentimes can't get opened by some drivers b/c it doesn't have a drive letter in front of it.
    bool valid = false;
    while (!valid)
    {
        std::stringstream ss;
        ss << prefix << "~" << rand() << suffix;
        if (!osgDB::fileExists(ss.str())) return ss.str();
    }
    return "";
}

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

                    if (featureType->getTiled())
                    {                        
                        result->setTiled( true );
                        result->setMaxLevel( featureType->getMaxLevel() );
                        result->setProfile( osgEarth::Profile::create(osgEarth::SpatialReference::create("epsg:4326"), featureType->getExtent().xMin(), featureType->getExtent().yMin(), featureType->getExtent().xMax(), featureType->getExtent().yMax(), 0, 1, 1) );
                    }
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
    
    std::string getExtensionForMimeType(const std::string& mime)
    {
        //OGR is particular sometimes about the extension of files when it's reading them so it's good to have
        //the temp file have an appropriate extension
        if ((mime.compare("text/xml") == 0) ||
            (mime.compare("text/xml; subtype=gml/2.1.2") == 0) ||
            (mime.compare("text/xml; subtype=gml/3.1.1") == 0)
            )
        {
            return ".xml";
        }        
        else if ((mime.compare("application/json") == 0) ||
                 (mime.compare("json") == 0) ||            

                 (mime.compare("application/x-javascript") == 0) ||
                 (mime.compare("text/javascript") == 0) ||
                 (mime.compare("text/x-javascript") == 0) ||
                 (mime.compare("text/x-json") == 0)                 
                )
        {
            return ".json";
        }        
        return "";
    }

    void getFeatures(HTTPResponse &response, FeatureList& features)
    {        
        //OE_NOTICE << "mimetype=" << response.getMimeType() << std::endl;
        //TODO:  Handle more than just geojson...
        std::string ext = getExtensionForMimeType(response.getMimeType());
        //Save the response to a temp file            
        std::string tmpPath = getTempPath();        
        std::string name = getTempName(tmpPath, ext);
        saveResponse(response, name );
        //OE_NOTICE << "Saving to " << name << std::endl;        

        //OGRDataSourceH ds = OGROpen(name.c_str(), FALSE, &driver);            
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
                    Feature* f = OgrUtils::createFeature( feat_handle );
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

        if (query.tileKey().isSet())
        {
            buf << "&Z=" << query.tileKey().get().getLevelOfDetail() << 
                   "&X=" << query.tileKey().get().getTileX() <<
                   "&Y=" << query.tileKey().get().getTileY();
        }
        else if (query.bounds().isSet())
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
        FeatureList features;
        if (response.isOK())
        {
            getFeatures(response, features);            
            std::string data = response.getPartAsString(0);        
        }
        else
        {
            OE_INFO << "Error getting url " << url << std::endl;
        }
        return new FeatureListCursor( features );        
    }

    virtual bool deleteFeature(FeatureID fid)
    {
        return false;
    }

    virtual int getFeatureCount() const
    {
        return -1;
    }

    virtual bool insertFeature(Feature* feature)
    {
        return false;
    }

    /**
    * Gets the Feature with the given FID
    * @returns
    *     The Feature with the given FID or NULL if not found.
    */
    virtual Feature* getFeature( FeatureID fid )
    {
        return 0;
    }

    virtual bool isWritable() const
    {
        return false;
    }

    virtual const FeatureSchema& getSchema() const
    {
        //TODO:  Populate the schema from the DescribeFeatureType call
        return _schema;
    }

    virtual osgEarth::Symbology::Geometry::Type getGeometryType() const
    {
        return Geometry::TYPE_UNKNOWN;
    }



private:
    const WFSFeatureOptions _options;  
    osg::ref_ptr< WFSCapabilities > _capabilities;
    FeatureSchema _schema;
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

