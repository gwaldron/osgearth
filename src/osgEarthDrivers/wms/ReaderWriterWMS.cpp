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

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Registry>
#include <osgEarth/TimeControl>
#include <osgEarth/XmlUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Containers>
#include <osgEarthUtil/WMS>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/ImageSequence>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <iomanip>

#include "TileService"
#include "WMSOptions"

#define LC "[WMS] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers;

//----------------------------------------------------------------------------

namespace
{
    // All looping ImageSequences deriving from this class will be in sync due to
    // a shared reference time.
    struct SyncImageSequence : public osg::ImageSequence
    {
        SyncImageSequence() : osg::ImageSequence() { }

        virtual void update(osg::NodeVisitor* nv)
        {
            setReferenceTime( 0.0 );
            osg::ImageSequence::update( nv );
        }
    };
}

//----------------------------------------------------------------------------

class WMSSource : public TileSource, public SequenceControl
{
public:
	WMSSource( const TileSourceOptions& options ) : TileSource( options ), _options(options)
    {
        _isPlaying     = false;

        if ( _options.times().isSet() )
        {
            StringTokenizer( *_options.times(), _timesVec, ",", "", false, true );
            OE_INFO << LC << "WMS-T: found " << _timesVec.size() << " times." << std::endl;

            for( unsigned i=0; i<_timesVec.size(); ++i )
            {
                _seqFrameInfoVec.push_back(SequenceFrameInfo());
                _seqFrameInfoVec.back().timeIdentifier = _timesVec[i];
            }
        }

        // localize it since we might override them:
        _formatToUse = _options.format().value();
        _srsToUse = _options.wmsVersion().value() == "1.3.0" ? _options.crs().value() : _options.srs().value();
        if (_srsToUse.empty())
        {
            //If they didn't specify a CRS, see if they specified an SRS and try to use that
            _srsToUse = _options.srs().value();
        }
    }

    /** override */
    Status initialize( const osgDB::Options* dbOptions )
    {
        osg::ref_ptr<const Profile> result;

        char sep = _options.url()->full().find_first_of('?') == std::string::npos? '?' : '&';

        URI capUrl = _options.capabilitiesUrl().value();
        if ( capUrl.empty() )
        {
            capUrl = URI(
                _options.url()->full() + 
                sep + 
                std::string("SERVICE=WMS") +
                std::string("&VERSION=") + _options.wmsVersion().value() +
                std::string("&REQUEST=GetCapabilities") );
        }

        //Try to read the WMS capabilities
        osg::ref_ptr<WMSCapabilities> capabilities = WMSCapabilitiesReader::read( capUrl.full(), dbOptions );
        if ( !capabilities.valid() )
        {
            return Status::Error( Status::ResourceUnavailable, "Unable to read WMS GetCapabilities." );
        }
        else
        {
            OE_INFO << LC << "Got capabilities from " << capUrl.full() << std::endl;
        }

        if ( _formatToUse.empty() && capabilities.valid() )
        {
            _formatToUse = capabilities->suggestExtension();
            OE_INFO << LC << "No format specified, capabilities suggested extension " << _formatToUse << std::endl;
        }

        if ( _formatToUse.empty() )
            _formatToUse = "png";
       
        if ( _srsToUse.empty() )
            _srsToUse = "EPSG:4326";

        std::string wmsFormatToUse = _options.wmsFormat().value();

        //Initialize the WMS request prototype
        std::stringstream buf;

        // first the mandatory keys:
        buf
            << std::fixed << _options.url()->full() << sep
	    << "SERVICE=WMS"
            << "&VERSION=" << _options.wmsVersion().value()
            << "&REQUEST=GetMap"
            << "&LAYERS=" << _options.layers().value()
            << "&FORMAT=" << ( wmsFormatToUse.empty() ? std::string("image/") + _formatToUse : wmsFormatToUse )
            << "&STYLES=" << _options.style().value()
            << (_options.wmsVersion().value() == "1.3.0" ? "&CRS=" : "&SRS=") << _srsToUse            
            << "&WIDTH="<< _options.tileSize().value()
            << "&HEIGHT="<< _options.tileSize().value()
            << "&BBOX=%lf,%lf,%lf,%lf";

        // then the optional keys:
        if ( _options.transparent().isSet() )
            buf << "&TRANSPARENT=" << (_options.transparent() == true ? "TRUE" : "FALSE");
            

        _prototype = "";
        _prototype = buf.str();

        //OE_NOTICE << "Prototype " << _prototype << std::endl;

        osg::ref_ptr<SpatialReference> wms_srs = SpatialReference::create( _srsToUse );

        // check for spherical mercator:
        if ( wms_srs.valid() && wms_srs->isEquivalentTo( osgEarth::Registry::instance()->getGlobalMercatorProfile()->getSRS() ) )
        {
            result = osgEarth::Registry::instance()->getGlobalMercatorProfile();
        }
        else if (wms_srs.valid() && wms_srs->isEquivalentTo( osgEarth::Registry::instance()->getGlobalGeodeticProfile()->getSRS()))
        {
            result = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        }

        // Next, try to glean the extents from the layer list
        if ( capabilities.valid() )
        {
            //TODO: "layers" mights be a comma-separated list. need to loop through and
            //combine the extents?? yes
            WMSLayer* layer = capabilities->getLayerByName( _options.layers().value() );
            if ( layer )
            {
                double minx, miny, maxx, maxy;                
                minx = miny = maxx = maxy = 0;

                //Check to see if the profile is equivalent to global-geodetic
                if (wms_srs->isGeographic())
                {
                    //Try to get the lat lon extents if they are provided
                    layer->getLatLonExtents(minx, miny, maxx, maxy);

                    //If we still don't have any extents, just default to global geodetic.
                    if (!result.valid() && minx == 0 && miny == 0 && maxx == 0 && maxy == 0)
                    {
                        result = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
                    }
                }	

                if (minx == 0 && miny == 0 && maxx == 0 && maxy == 0)
                {
                    layer->getExtents(minx, miny, maxx, maxy);
                }


                if (!result.valid())
                {
                    result = Profile::create( _srsToUse, minx, miny, maxx, maxy );
                }

                //Add the layer extents to the list of valid areas
                if (minx != 0 || maxx != 0 || miny != 0 || maxy != 0)
                {
                    GeoExtent extent( result->getSRS(), minx, miny, maxx, maxy);
                    getDataExtents().push_back( DataExtent(extent, 0) );
                }
            }
        }

        // Last resort: create a global extent profile (only valid for global maps)
        if ( !result.valid() && wms_srs->isGeographic())
        {
            result = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        }

        // JPL uses an experimental interface called TileService -- ping to see if that's what
        // we are trying to read:
        URI tsUrl = _options.tileServiceUrl().value();
        if ( tsUrl.empty() )
        {
            tsUrl = URI(_options.url()->full() + sep + std::string("request=GetTileService") );
        }

        OE_INFO << LC << "Testing for JPL/TileService at " << tsUrl.full() << std::endl;
        _tileService = TileServiceReader::read(tsUrl.full(), dbOptions);
        if (_tileService.valid())
        {
            OE_INFO << LC << "Found JPL/TileService spec" << std::endl;
            TileService::TilePatternList patterns;
            _tileService->getMatchingPatterns(
                _options.layers().value(),
                _formatToUse,
                _options.style().value(),
                _srsToUse,
                _options.tileSize().value(),
                _options.tileSize().value(),
                patterns );

            if (patterns.size() > 0)
            {
                result = _tileService->createProfile( patterns );
                _prototype = _options.url()->full() + sep + patterns[0].getPrototype();
            }
        }
        else
        {
            OE_INFO << LC << "No JPL/TileService spec found; assuming standard WMS" << std::endl;
        }

        // Use the override profile if one is passed in.
        if ( getProfile() == 0L )
        {
            setProfile( result.get() );
        }

        if ( getProfile() )
        {
            OE_INFO << LC << "Profile=" << getProfile()->toString() << std::endl;

            // set up the cache options properly for a TileSource.
            _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );            

            return Status::OK();
        }
        else
        {
            return Status::Error( "Unable to establish profile" );
        }
    }

    /* override */
    bool isDynamic() const
    {
        // return TRUE if we are reading WMS-T.
        return _timesVec.size() > 1;
    }

public:

    // fetch a tile image from the WMS service and report any exceptions.
    osg::Image* fetchTileImage(
        const TileKey&     key, 
        const std::string& extraAttrs,
        ProgressCallback*  progress, 
        ReadResult&        out_response )
    {
        osg::ref_ptr<osg::Image> image;

        std::string uri = createURI(key);
        if ( !extraAttrs.empty() )
        {
            std::string delim = uri.find("?") == std::string::npos ? "?" : "&";
            uri = uri + delim + extraAttrs;
        }

        // Try to get the image first
        out_response = URI( uri ).readImage( _dbOptions.get(), progress);

        if ( out_response.succeeded() )
        {
            image = out_response.getImage();
        }
        
#if 0
        if ( !out_response.succeeded() )
        {
            // If it failed, see whether there's any info in the response.
            OE_WARN << LC << "Failed, response:\n" << out_response.metadata().toJSON(true);
            
            // BAD: caches string data in an image cache. -gw
            out_response = URI( uri ).readString( _dbOptions.get(), progress );      

            // get the mime type:
            std::string mt = out_response.metadata().value( IOMetadata::CONTENT_TYPE );

            if ( mt == "application/vnd.ogc.se_xml" || mt == "text/xml" )
            {
                std::istringstream content( out_response.getString() );

                // an XML result means there was a WMS service exception:
                Config se;
                if ( se.fromXML(content) )
                {
                    Config ex = se.child("serviceexceptionreport").child("serviceexception");
                    if ( !ex.empty() )
                    {
                        OE_INFO << LC << "WMS Service Exception: " << ex.toJSON(true) << std::endl;
                    }
                    else
                    {
                        OE_INFO << LC << "WMS Response: " << se.toJSON(true) << std::endl;
                    }
                }
                else
                {
                    OE_INFO << LC << "WMS: unknown error." << std::endl;
                }
            }     
        }
        else
        {
            image = out_response.getImage();
        }
#endif

        return image.release();
    }


    /** override */
    osg::Image* createImage( const TileKey& key, ProgressCallback* progress )
    {
        osg::ref_ptr<osg::Image> image;

        if ( _timesVec.size() > 1 )
        {
            image = createImageSequence( key, progress );
        }
        else
        {
            std::string extras;
            if ( _timesVec.size() == 1 )
                extras = std::string("TIME=") + _timesVec[0];

            ReadResult response;
            image = fetchTileImage( key, extras, progress, response );
        }

        return image.release();
    }

    /** creates a 3D image from timestamped data. */
    osg::Image* createImage3D( const TileKey& key, ProgressCallback* progress )
    {
        osg::ref_ptr<osg::Image> image;

        for( unsigned int r=0; r<_timesVec.size(); ++r )
        {
            std::string extraAttrs = std::string("TIME=") + _timesVec[r];
            ReadResult response;
            
            osg::ref_ptr<osg::Image> timeImage = fetchTileImage( key, extraAttrs, progress, response );

            if ( !image.valid() )
            {
                image = new osg::Image();
                image->allocateImage(
                    timeImage->s(), timeImage->t(), _timesVec.size(),
                    timeImage->getPixelFormat(),
                    timeImage->getDataType(),
                    timeImage->getPacking() );
                image->setInternalTextureFormat( timeImage->getInternalTextureFormat() );
            }

            memcpy( image->data(0,0,r), 
                    timeImage->data(), 
                    osg::minimum(image->getImageSizeInBytes(), timeImage->getImageSizeInBytes()) );
        }

        return image.release();
    }

    /** creates a 3D image from timestamped data. */
    osg::Image* createImageSequence( const TileKey& key, ProgressCallback* progress )
    {
        osg::ref_ptr< osg::ImageSequence > seq = new SyncImageSequence();
        
        seq->setLoopingMode( osg::ImageStream::LOOPING );
        seq->setLength( _options.secondsPerFrame().value() * (double)_timesVec.size() );
        if ( this->isSequencePlaying() )
            seq->play();

        for( unsigned int r=0; r<_timesVec.size(); ++r )
        {
            std::string extraAttrs = std::string("TIME=") + _timesVec[r];

            ReadResult response;
            osg::ref_ptr<osg::Image> image = fetchTileImage( key, extraAttrs, progress, response );
            if ( image.get() )
            {
                seq->addImage( image );
            }
        }

        // Just return an empty image if we didn't get any images
#if OSG_VERSION_LESS_THAN(3,1,4)
        unsigned size = seq->getNumImages();
#else
        unsigned size = seq->getNumImageData();
#endif

        if (size == 0)
        {
            return ImageUtils::createEmptyImage();
        }

        _sequenceCache.insert( seq );
        return seq.release();
    }


    /** override */
    osg::HeightField* createHeightField( const TileKey& key, ProgressCallback* progress)
    {
        osg::Image* image = createImage(key, progress);
        if (!image)
        {
            OE_INFO << "[osgEarth::WMS] Failed to read heightfield from " << createURI(key) << std::endl;
        }

        float scaleFactor = 1;

        //Scale the heightfield to meters
        if ( _options.elevationUnit() == "ft")
        {
            scaleFactor = 0.3048;
        }

        ImageToHeightFieldConverter conv;
        return conv.convert( image, scaleFactor );
    }


    std::string createURI( const TileKey& key ) const
    {
        double minx, miny, maxx, maxy;
        key.getExtent().getBounds( minx, miny, maxx, maxy);
        
        char buf[2048];
        sprintf(buf, _prototype.c_str(), minx, miny, maxx, maxy);
        
        std::string uri(buf);

        // url-ize the uri before returning it
        if ( osgDB::containsServerAddress( uri ) )
            uri = replaceIn(uri, " ", "%20");

        return uri;
    }

    virtual int getPixelsPerTile() const
    {
        return _options.tileSize().value();
    }

    virtual std::string getExtension()  const 
    {
        return _formatToUse;
    }


public: // SequenceControl

    /** Whether the implementation supports these methods */
    bool supportsSequenceControl() const
    {
        return _timesVec.size() > 1;
    }

    /** Starts playback */
    void playSequence()
    {
        //todo
        _isPlaying = true;
    }

    /** Stops playback */
    void pauseSequence()
    {
        //todo
        _isPlaying = false;
    }

    /** Seek to a specific frame */
    void seekToSequenceFrame(unsigned frame)
    {
        //todo
    }

    /** Whether the object is in playback mode */
    bool isSequencePlaying() const
    {
        return _isPlaying;
    }

    /** Gets data about the current frame in the sequence */
    const std::vector<SequenceFrameInfo>& getSequenceFrameInfo() const
    {
        return _seqFrameInfoVec;
    }

    /** Index of current frame */
    int getCurrentSequenceFrameIndex( const osg::FrameStamp* fs ) const
    {
        if ( _seqFrameInfoVec.size() == 0 )
            return 0;

        double len = _options.secondsPerFrame().value() * (double)_timesVec.size();
        double t   = fmod( fs->getSimulationTime(), len ) / len;
        return osg::clampBetween(
            (int)(t * (double)_seqFrameInfoVec.size()), 
            (int)0, 
            (int)_seqFrameInfoVec.size()-1);
    }


private:
    const WMSOptions                 _options;
    std::string                      _formatToUse;
    std::string                      _srsToUse;
    osg::ref_ptr<TileService>        _tileService;
    osg::ref_ptr<const Profile>      _profile;
    std::string                      _prototype;
    std::vector<std::string>         _timesVec;
    osg::ref_ptr<osgDB::Options>     _dbOptions;
    bool                             _isPlaying;
    std::vector<SequenceFrameInfo>   _seqFrameInfoVec;

    mutable ThreadSafeObserverSet<osg::ImageSequence> _sequenceCache;
};


class WMSSourceFactory : public TileSourceDriver
{
    public:
        WMSSourceFactory() {}

        virtual const char* className() const
        {
            return "WMS Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "osgearth_wms" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            return new WMSSource( getTileSourceOptions(opt) );
        }
};

REGISTER_OSGPLUGIN(osgearth_wms, WMSSourceFactory)

