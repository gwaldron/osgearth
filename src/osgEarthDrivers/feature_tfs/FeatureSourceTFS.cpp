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
#include "TFSFeatureOptions"

#include <osgEarth/Registry>
#include <osgEarth/XmlUtils>
#include <osgEarth/FileUtils>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Filter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ScaleFilter>
#include <osgEarthFeatures/OgrUtils>
#include <osgEarthUtil/TFS>
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

#define LC "[TFS FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

/**
 * A FeatureSource that reads features from a TFS layer
 * 
 */
class TFSFeatureSource : public FeatureSource
{
public:
    TFSFeatureSource(const TFSFeatureOptions& options ) :
      FeatureSource( options ),
      _options     ( options ),
      _layerValid(false)
    {                
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~TFSFeatureSource()
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

                std::string binId = Stringify() << std::hex << hashString(optionsConf.toJSON()) << "_tfs";
                _cacheBin = cache->addBin( binId );
                
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
        _layerValid = TFSReaderWriter::read(_options.url().get(), _dbOptions.get(), _layer);
        if (_layerValid)
        {
            OE_INFO << LC <<  "Read layer TFS " << _layer.getTitle() << " " << _layer.getAbstract() << " " << _layer.getFirstLevel() << " " << _layer.getMaxLevel() << " " << _layer.getExtent().toString() << std::endl;
        }
    }


    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        FeatureProfile* result = NULL;
        if (_layerValid)
        {
            result = new FeatureProfile(_layer.getExtent());
            result->setTiled( true );
            result->setFirstLevel( _layer.getFirstLevel());
            result->setMaxLevel( _layer.getMaxLevel());
            result->setProfile( osgEarth::Profile::create(_layer.getSRS(), _layer.getExtent().xMin(), _layer.getExtent().yMin(), _layer.getExtent().xMax(), _layer.getExtent().yMax(), 1, 1) );
        }
        return result;        
    }


    bool getFeatures( const std::string& buffer, const std::string& mimeType, FeatureList& features )
    {        
        // find the right driver for the given mime type
        OGR_SCOPED_LOCK;
                
        // find the right driver for the given mime type
        OGRSFDriverH ogrDriver =
            isJSON(mimeType) ? OGRGetDriverByName( "GeoJSON" ) :
            isGML(mimeType)  ? OGRGetDriverByName( "GML" ) :
            0L;

        // fail if we can't find an appropriate OGR driver:
        if ( !ogrDriver )
        {
            OE_WARN << LC << "Error reading TFS response; cannot grok content-type \"" << mimeType << "\""
                << std::endl;
            return false;
        }

        OGRDataSourceH ds = OGROpen( buffer.c_str(), FALSE, &ogrDriver );
        
        if ( !ds )
        {
            OE_WARN << LC << "Error reading TFS response" << std::endl;
            return false;
        }

        // read the feature data.
        OGRLayerH layer = OGR_DS_GetLayer(ds, 0);
        if ( layer )
        {
            const SpatialReference* srs = _layer.getSRS();

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
        if (query.tileKey().isSet())
        {
            std::stringstream buf;
            std::string path = osgDB::getFilePath(_options.url()->full());
            buf << path << "/" << query.tileKey().get().getLevelOfDetail() << "/"
                               << query.tileKey().get().getTileX() << "/"
                               << query.tileKey().get().getTileY()
                               << "." << _options.format().get();            
            OE_DEBUG << "TFS url " << buf.str() << std::endl;
            return buf.str();
        }
        return "";                       
    }

    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
    {
        FeatureCursor* result = 0L;

        std::string url = createURL( query );
        if (url.empty()) return 0;

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
            std::string mimeType = r.metadata().value( IOMetadata::CONTENT_TYPE );
            //If the mimetype is empty then try to set it from the format specification
            if (mimeType.empty())
            {
                if (_options.format().value() == "json") mimeType = "json";
                else if (_options.format().value().compare("gml") == 0) mimeType = "text/xml";
            }
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
                cx.profile() = getFeatureProfile();

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
    const TFSFeatureOptions         _options;    
    FeatureSchema                   _schema;
    osg::ref_ptr<CacheBin>          _cacheBin;
    osg::ref_ptr<osgDB::Options>    _dbOptions;    
    TFSLayer                        _layer;
    bool                            _layerValid;
};


class TFSFeatureSourceFactory : public FeatureSourceDriver
{
public:
    TFSFeatureSourceFactory()
    {
        supportsExtension( "osgearth_feature_tfs", "TFS feature driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "TFS Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new TFSFeatureSource( getFeatureSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_tfs, TFSFeatureSourceFactory)

