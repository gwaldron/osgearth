/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
    Status initialize(const osgDB::Options* readOptions)
    {
        // initialize the base class
        //FeatureSource::initialize(readOptions);

        // store a reference to the read options so we can pass them along to
        // later requests.
        _readOptions = readOptions;

        // parse the WFS capabilities URL
        std::string capUrl;
        if ( _options.url().isSet() )
        {
            char sep = _options.url()->full().find_first_of('?') == std::string::npos? '?' : '&';

            capUrl = 
                _options.url()->full() +
                sep + 
                "SERVICE=WFS&VERSION=1.0.0&REQUEST=GetCapabilities";
        }        

        // read the WFS capabilities:
        _capabilities = WFSCapabilitiesReader::read( capUrl, _readOptions.get() );
        if ( !_capabilities.valid() )
        {
            return Status::Error(Status::ResourceUnavailable, Stringify()<<"Failed to read WFS GetCapabilities from \"" << capUrl << "\"");
        }
        else
        {
            OE_INFO << "[osgEarth::WFS] Got capabilities from " << capUrl << std::endl;
        }

        // establish a feature profile
        FeatureProfile* fp = 0L;

        //Find the feature type by name
        osg::ref_ptr< WFSFeatureType > featureType = _capabilities->getFeatureTypeByName( _options.typeName().get() );
        if (featureType.valid())
        {
            if (featureType->getExtent().isValid())
            {
                fp = new FeatureProfile(featureType->getExtent());

                bool disableTiling = _options.disableTiling().isSetTo(true);

                if (featureType->getTiled() && !disableTiling)
                {                        
                    fp->setTiled( true );
                    fp->setFirstLevel( featureType->getFirstLevel() );
                    fp->setMaxLevel( featureType->getMaxLevel() );
                    fp->setProfile(osgEarth::Profile::create(
                        osgEarth::SpatialReference::create("epsg:4326"), 
                        featureType->getExtent().xMin(), featureType->getExtent().yMin(), 
                        featureType->getExtent().xMax(), featureType->getExtent().yMax(), 
                        1, 1) );
                }
            }
        }

        // if nothing else, fall back on a global geodetic feature profile.
        if ( !fp )
        {
            fp = new FeatureProfile(GeoExtent(SpatialReference::create( "epsg:4326" ), -180, -90, 180, 90));
        }
             
        if (_options.geoInterp().isSet())
        {
            fp->geoInterp() = _options.geoInterp().get();
        }

        setFeatureProfile( fp );

        return Status::OK();
    }

    void saveResponse(const std::string buffer, const std::string& filename)
    {
        std::ofstream fout;
        fout.open(filename.c_str(), std::ios::out | std::ios::binary);        
        fout.write(buffer.c_str(), buffer.size());        
        fout.close();
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
            OGR_L_ResetReading(layer);                                
            OGRFeatureH feat_handle;
            while ((feat_handle = OGR_L_GetNextFeature( layer )) != NULL)
            {
                if ( feat_handle )
                {
                    osg::ref_ptr<Feature> f = OgrUtils::createFeature( feat_handle, getFeatureProfile() );
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

            unsigned int tileX = query.tileKey().get().getTileX();
            unsigned int tileY = query.tileKey().get().getTileY();
            unsigned int level = query.tileKey().get().getLevelOfDetail();
            
            // Tiled WFS follows the same protocol as TMS, with the origin in the lower left of the profile.
            // osgEarth TileKeys are upper left origin, so we need to invert the tilekey to request the correct key.
            unsigned int numRows, numCols;
            query.tileKey().get().getProfile()->getNumTiles(level, numCols, numRows);
            tileY  = numRows - tileY - 1;

            buf << "&Z=" << level << 
                   "&X=" << tileX <<
                   "&Y=" << tileY;
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
        ReadResult r = uri.readString( _readOptions.get() );

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
        if (!getFilters().empty())
        {
            // preprocess the features using the filter list:
            if ( features.size() > 0 )
            {
                FilterContext cx;
                cx.setProfile( getFeatureProfile() );

                for( FeatureFilterList::const_iterator i = getFilters().begin(); i != getFilters().end(); ++i )
                {
                    FeatureFilter* filter = i->get();
                    cx = filter->push( features, cx );
                }
            }
        }

        // If we have any features and we have an fid attribute, override the fid of the features
        if (_options.fidAttribute().isSet())
        {
            for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
            {
                std::string attr = itr->get()->getString(_options.fidAttribute().get());                
                FeatureID fid = as<long>(attr, 0);
                itr->get()->setFID( fid );
            }
        }

        //result = new FeatureListCursor(features);
        result = dataOK ? new FeatureListCursor( features ) : 0L;

        if ( !result )
            Registry::instance()->blacklist( url );

        return result;
    }

    virtual bool supportsGetFeature() const
    {
        return false;
    }

    virtual Feature* getFeature( FeatureID fid )
    {
        // not supported for WFS
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
    const WFSFeatureOptions            _options;  
    osg::ref_ptr< WFSCapabilities >    _capabilities;
    osg::ref_ptr< FeatureProfile >     _featureProfile;
    FeatureSchema                      _schema;
    osg::ref_ptr<const osgDB::Options> _readOptions;
};


class WFSFeatureSourceFactory : public FeatureSourceDriver
{
public:
    WFSFeatureSourceFactory()
    {
        supportsExtension( "osgearth_feature_wfs", "WFS feature driver for osgEarth" );
    }

    virtual const char* className() const
    {
        return "WFS Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const osgDB::Options* readOptions) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new WFSFeatureSource( getFeatureSourceOptions(readOptions) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_wfs, WFSFeatureSourceFactory)

