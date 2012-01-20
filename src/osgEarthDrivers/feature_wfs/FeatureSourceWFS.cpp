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
#include "WFSFeatureOptions"

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Filter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ScaleFilter>
#include <osgEarthUtil/WFS>
#include <osgEarthFeatures/OgrUtils>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <stdio.h>
#include <stdlib.h>

#include <ogr_api.h>



//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

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
    WFSFeatureSource(const WFSFeatureOptions& options ) :
      FeatureSource( options ),
      _options     ( options )
    {        
        _geojsonDriver = OGRGetDriverByName( "GeoJSON" );
        _gmlDriver     = OGRGetDriverByName( "GML" );
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~WFSFeatureSource()
    {               
        //nop
    }

    //override
    void initialize( const osgDB::Options* dbOptions )
    {
        _dbOptions = dbOptions ? osg::clone(dbOptions) : 0L;
        if ( _dbOptions.valid() )
        {
            // Set up a Custom caching bin for this source:
            Cache* cache = Cache::get( _dbOptions.get() );
            if ( cache )
            {
                Config optionsConf = _options.getConfig();

                std::string binId = Stringify() << std::hex << hashString(optionsConf.toJSON()) << "_wfs";
                _cacheBin = cache->addBin( binId );
                
                // write a metadata record just for reference purposes.. we don't actually use it
                Config metadata = _cacheBin->readMetadata();
                if ( metadata.empty() )
                {
                    _cacheBin->writeMetadata( optionsConf );
                }

                if ( _cacheBin.valid() )
                {
                    _cacheBin->store( _dbOptions.get() );
                }
            }
        }

        std::string capUrl;

        if ( _options.url().isSet() )
        {
            char sep = _options.url()->full().find_first_of('?') == std::string::npos? '?' : '&';

            capUrl = 
                _options.url()->full() +
                sep + 
                "SERVICE=WFS&VERSION=1.0.0&REQUEST=GetCapabilities";
        }

        _capabilities = WFSCapabilitiesReader::read( capUrl, _dbOptions.get() );
        if ( !_capabilities.valid() )
        {
            OE_WARN << "[osgEarth::WFS] Unable to read WFS GetCapabilities." << std::endl;
            //return;
        }
        else
        {
            OE_INFO << "[osgEarth::WFS] Got capabilities from " << capUrl << std::endl;
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
                        result->setFirstLevel( featureType->getFirstLevel() );
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


    bool getFeatures( const std::string& buffer, const std::string& mimeType, FeatureList& features )
    {
        // find the right driver for the given mime type
        OGRSFDriverH ogrDriver =
            isJSON(mimeType) ? _geojsonDriver :
            isGML(mimeType)  ? _gmlDriver :
            0L;

        // fail if we can't find an appropriate OGR driver:
        if ( !ogrDriver )
        {
            OE_WARN << LC << "Error reading WFS response; cannot grok content-type \"" << mimeType << "\""
                << std::endl;
            return false;
        }

        OGRDataSourceH ds = OGROpen( buffer.c_str(), FALSE, &ogrDriver );
        
        if ( !ds )
        {
            OE_WARN << LC << "Error reading WFS response" << std::endl;
            return false;
        }

        // read the feature data.
        OGRLayerH layer = OGR_DS_GetLayer(ds, 0);
        if ( layer )
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

        // Destroy the datasource
        OGR_DS_Destroy( ds );
        
        return true;
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

    bool isGML( const std::string& mime ) const
    {
        return
            startsWith(mime, "text/xml");
    }


    bool isJSON( const std::string& mime ) const
    {
        return
            (mime.compare("application/json") == 0)         ||
            (mime.compare("json") == 0)                     ||            

            (mime.compare("application/x-javascript") == 0) ||
            (mime.compare("text/javascript") == 0)          ||
            (mime.compare("text/x-javascript") == 0)        ||
            (mime.compare("text/x-json") == 0);
    }

    std::string createURL(const Symbology::Query& query)
    {
        std::stringstream buf;
        buf << _options.url()->full() << "?SERVICE=WFS&VERSION=1.0.0&REQUEST=getfeature";
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
        std::string str;
        str = buf.str();
        return str;
    }

    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
    {
        FeatureCursor* result = 0L;

        std::string url = createURL( query );

        // check the blacklist:
        if ( Registry::instance()->isBlacklisted(url) )
            return 0L;

        OE_DEBUG << LC << url << std::endl;
        URI uri(url);

        // read the data:
        ReadResult r = uri.readString( _dbOptions.get() );

        const std::string& buffer = r.getString();
        const Config&      meta   = r.metadata();

        bool dataOK = false;

        FeatureList features;
        if ( !buffer.empty() )
        {
            // Get the mime-type from the metadata record if possible
            const std::string& mimeType = r.metadata().value( IOMetadata::CONTENT_TYPE );
            dataOK = getFeatures( buffer, mimeType, features );
        }

        if ( dataOK )
        {
            OE_DEBUG << LC << "Read " << features.size() << " features" << std::endl;
        }

        //result = new FeatureListCursor(features);
        result = dataOK ? new FeatureListCursor( features ) : 0L;

        if ( !result )
            Registry::instance()->blacklist( url );

        return result;
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
    const WFSFeatureOptions         _options;  
    osg::ref_ptr< WFSCapabilities > _capabilities;
    FeatureSchema                   _schema;
    osg::ref_ptr<CacheBin>          _cacheBin;
    osg::ref_ptr<osgDB::Options>    _dbOptions;
    OGRSFDriverH                    _geojsonDriver, _gmlDriver;
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

