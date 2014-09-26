/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
                _cacheBin->setHashKeys(true);
                
                // write a metadata record just for reference purposes.. we don't actually use it
                Config metadata = _cacheBin->readMetadata();
                if ( metadata.empty() )
                {
                    _cacheBin->writeMetadata( optionsConf );
                }

                if ( _cacheBin.valid() )
                {
                    _cacheBin->apply( _dbOptions.get() );
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

    void saveResponse(const std::string buffer, const std::string& filename)
    {
        std::ofstream fout;
        fout.open(filename.c_str(), std::ios::out | std::ios::binary);        
        fout.write(buffer.c_str(), buffer.size());        
        fout.close();
    }


    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        if ( !_featureProfile.valid() )
        {
            static Threading::Mutex s_mutex;
            Threading::ScopedMutexLock lock(s_mutex);
            
            if ( !_featureProfile.valid() )
            {
                FeatureProfile* result = 0L;

                if (_capabilities.valid())
                {
                    //Find the feature type by name
                    osg::ref_ptr< WFSFeatureType > featureType = _capabilities->getFeatureTypeByName( _options.typeName().get() );
                    if (featureType.valid())
                    {
                        if (featureType->getExtent().isValid())
                        {
                            result = new FeatureProfile(featureType->getExtent());

                            bool disableTiling = _options.disableTiling().isSet() && *_options.disableTiling();

                            if (featureType->getTiled() && !disableTiling)
                            {                        
                                result->setTiled( true );
                                result->setFirstLevel( featureType->getFirstLevel() );
                                result->setMaxLevel( featureType->getMaxLevel() );
                                result->setProfile( osgEarth::Profile::create(osgEarth::SpatialReference::create("epsg:4326"), featureType->getExtent().xMin(), featureType->getExtent().yMin(), featureType->getExtent().xMax(), featureType->getExtent().yMax(), 1, 1) );
                            }
                        }
                    }
                }

                if (!result)
                {
                    result = new FeatureProfile(GeoExtent(SpatialReference::create( "epsg:4326" ), -180, -90, 180, 90));
                }
                
                _featureProfile = result;
            }
        }

        return _featureProfile.get();
    }

    FeatureProfile* getFeatureProfile()
    {
        if ( !_featureProfile.valid() )
        {
            createFeatureProfile();
        }
        return _featureProfile.get();
    }

    bool getFeatures( const std::string& buffer, const std::string& mimeType, FeatureList& features )
    {
        OGR_SCOPED_LOCK;        

        bool json = isJSON( mimeType );
        bool gml  = isGML( mimeType );

        // find the right driver for the given mime type
        OGRSFDriverH ogrDriver =
            json ? OGRGetDriverByName( "GeoJSON" ) :
            gml  ? OGRGetDriverByName( "GML" ) :
            0L;        

        // fail if we can't find an appropriate OGR driver:
        if ( !ogrDriver )
        {
            OE_WARN << LC << "Error reading WFS response; cannot grok content-type \"" << mimeType << "\""
                << std::endl;
            return false;
        }

        std::string tmpName;

        OGRDataSourceH ds = 0;
        //GML needs to be saved to a temp file to load from disk.  GeoJSON can be loaded directly from memory
        if (gml)
        {
            std::string ext = getExtensionForMimeType( mimeType );
            //Save the response to a temp file            
            std::string tmpPath = getTempPath();        
            tmpName = getTempName(tmpPath, ext);
            saveResponse(buffer, tmpName );
            ds = OGROpen( tmpName.c_str(), FALSE, &ogrDriver );
        }
        else if (json)
        {
            //Open GeoJSON directly from memory
            ds = OGROpen( buffer.c_str(), FALSE, &ogrDriver );
        }        

        
        if ( !ds )
        {
            OE_WARN << LC << "Error reading WFS response" << std::endl;
            return false;
        }

        // read the feature data.
        OGRLayerH layer = OGR_DS_GetLayer(ds, 0);
        if ( layer )
        {
            FeatureProfile* fp = getFeatureProfile();
            const SpatialReference* srs = fp ? fp->getSRS() : 0L;

            OGR_L_ResetReading(layer);                                
            OGRFeatureH feat_handle;
            while ((feat_handle = OGR_L_GetNextFeature( layer )) != NULL)
            {
                if ( feat_handle )
                {
                    osg::ref_ptr<Feature> f = OgrUtils::createFeature( feat_handle, srs );
                    if ( f.valid() && !isBlacklisted(f->getFID()) )
                    {
                        features.push_back( f.release() );
                    }
                    OGR_F_Destroy( feat_handle );
                }
            }
        }

        // Destroy the datasource
        OGR_DS_Destroy( ds );

        //Delete the temp file if one was created
        if (!tmpName.empty())
        {
            remove( tmpName.c_str() );
        }
        
        return true;
    }

    
    std::string getExtensionForMimeType(const std::string& mime)
    {
        //OGR is particular sometimes about the extension of files when it's reading them so it's good to have
        //the temp file have an appropriate extension
        if (isGML(mime))
        {
            return ".xml";
        }        
		else if (isJSON(mime))
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
            startsWith(mime, "application/json") ||
            startsWith(mime, "json") ||            
            startsWith(mime, "application/x-javascript") ||
            startsWith(mime, "text/javascript") ||
            startsWith(mime, "text/x-javascript") ||
            startsWith(mime, "text/x-json");
    }

    std::string createURL(const Symbology::Query& query)
    {
        std::stringstream buf;
        buf << _options.url()->full() << "?SERVICE=WFS&VERSION=1.0.0&REQUEST=GetFeature";
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
            double buffer = *_options.buffer();            
            buf << "&BBOX=" << std::setprecision(16)
                            << query.bounds().get().xMin() - buffer << ","
                            << query.bounds().get().yMin() - buffer << ","
                            << query.bounds().get().xMax() + buffer << ","
                            << query.bounds().get().yMax() + buffer;
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

        //If we have any filters, process them here before the cursor is created
        if (!_options.filters().empty())
        {
            // preprocess the features using the filter list:
            if ( features.size() > 0 )
            {
                FilterContext cx;
                cx.setProfile( getFeatureProfile() );

                for( FeatureFilterList::const_iterator i = _options.filters().begin(); i != _options.filters().end(); ++i )
                {
                    FeatureFilter* filter = i->get();
                    cx = filter->push( features, cx );
                }
            }
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
    osg::ref_ptr< FeatureProfile >  _featureProfile;
    FeatureSchema                   _schema;
    osg::ref_ptr<CacheBin>          _cacheBin;
    osg::ref_ptr<osgDB::Options>    _dbOptions;    
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

