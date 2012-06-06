/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/ImageLayer>
#include <osgEarth/TileSource>
#include <osgEarth/ImageMosaic>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/URI>
#include <osg/Version>
#include <memory.h>
#include <limits.h>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[ImageLayer] "

// TESTING
//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

//------------------------------------------------------------------------

ImageLayerOptions::ImageLayerOptions( const ConfigOptions& options ) :
TerrainLayerOptions(options)
{
    setDefaults();
    fromConfig( _conf );
}

ImageLayerOptions::ImageLayerOptions( const std::string& name, const TileSourceOptions& driverOpt ) :
TerrainLayerOptions(name, driverOpt)
{
    setDefaults();
    fromConfig( _conf );
}

void
ImageLayerOptions::setDefaults()
{
    _opacity.init( 1.0f );
    _transparentColor.init( osg::Vec4ub(0,0,0,0) );
    _hslAdjust.init( osg::Vec3(0,0,0));
    _bcgAdjust.init( osg::Vec3(0,0,0));
    _minRange.init( -FLT_MAX );
    _maxRange.init( FLT_MAX );
    _lodBlending.init( false );
}

void
ImageLayerOptions::mergeConfig( const Config& conf )
{
    TerrainLayerOptions::mergeConfig( conf );
    fromConfig( conf );
}

void
ImageLayerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "nodata_image", _noDataImageFilename );
    conf.getIfSet( "opacity", _opacity );
    conf.getIfSet( "min_range", _minRange );
    conf.getIfSet( "max_range", _maxRange );
    conf.getIfSet( "lod_blending", _lodBlending );
    conf.getIfSet( "hsl_adjust", _hslAdjust );
    conf.getIfSet( "bcg_adjust", _bcgAdjust );

    if ( conf.hasValue( "transparent_color" ) )
        _transparentColor = stringToColor( conf.value( "transparent_color" ), osg::Vec4ub(0,0,0,0));	    
}

Config
ImageLayerOptions::getConfig( bool isolate ) const
{
    Config conf = TerrainLayerOptions::getConfig( isolate );

    conf.updateIfSet( "nodata_image", _noDataImageFilename );
    conf.updateIfSet( "opacity", _opacity );
    conf.updateIfSet( "min_range", _minRange );
    conf.updateIfSet( "max_range", _maxRange );
    conf.updateIfSet( "lod_blending", _lodBlending );
    conf.updateIfSet( "hsl_adjust", _hslAdjust );
    conf.updateIfSet( "bcg_adjust", _bcgAdjust );

    if (_transparentColor.isSet())
        conf.update("transparent_color", colorToString( _transparentColor.value()));
    
    return conf;
}

//------------------------------------------------------------------------

namespace
{
    struct ImageLayerPreCacheOperation : public TileSource::ImageOperation
    {
        void operator()( osg::ref_ptr<osg::Image>& image )
        {
            _processor.process( image );
        }

        ImageLayerTileProcessor _processor;
    };
    
    struct ApplyChromaKey
    {
        osg::Vec4f _chromaKey;
        bool operator()( osg::Vec4f& pixel ) {
            bool equiv = ImageUtils::areRGBEquivalent( pixel, _chromaKey );
            if ( equiv ) pixel.a() = 0.0f;
            return equiv;
        }
    };
}

//------------------------------------------------------------------------

ImageLayerTileProcessor::ImageLayerTileProcessor(const ImageLayerOptions& options)
{
    init( options, 0L, false );
}

void
ImageLayerTileProcessor::init(const ImageLayerOptions& options,
                              const osgDB::Options*    dbOptions, 
                              bool                     layerInTargetProfile )
{
    _options = options;
    _layerInTargetProfile = layerInTargetProfile;

    //if ( _layerInTargetProfile )
    //    OE_DEBUG << LC << "Good, the layer and map have the same profile." << std::endl;

    const osg::Vec4ub& ck= *_options.transparentColor();
    _chromaKey.set( ck.r() / 255.0f, ck.g() / 255.0f, ck.b() / 255.0f, 1.0 );

    if ( _options.noDataImageFilename().isSet() && !_options.noDataImageFilename()->empty() )
    {
        _noDataImage = URI( *_options.noDataImageFilename() ).readImage(dbOptions).getImage();
        if ( !_noDataImage.valid() )
        {
            OE_WARN << "Warning: Could not read nodata image from \"" << _options.noDataImageFilename().value() << "\"" << std::endl;
        }
    }
}

void
ImageLayerTileProcessor::process( osg::ref_ptr<osg::Image>& image ) const
{
    if ( !image.valid() )
        return;

    // Check to see if the image is the nodata image
    if ( _noDataImage.valid() )
    {
        if (ImageUtils::areEquivalent(image.get(), _noDataImage.get()))
        {
            //OE_DEBUG << LC << "Found nodata" << std::endl;
            image = 0L;
            return;
        }
    }

    // If this is a compressed image, uncompress it IF the image is not already in the
    // target profile...becuase if it's not in the target profile, we will have to do
    // some mosaicing...and we can't mosaic a compressed image.
    if (!_layerInTargetProfile &&
        ImageUtils::isCompressed(image.get()) &&
        ImageUtils::canConvert(image.get(), GL_RGBA, GL_UNSIGNED_BYTE) )
    {
        image = ImageUtils::convertToRGBA8( image.get() );
    }

    // Apply a transparent color mask if one is specified
    if ( _options.transparentColor().isSet() )
    {
        if ( !ImageUtils::hasAlphaChannel(image.get()) && ImageUtils::canConvert(image.get(), GL_RGBA, GL_UNSIGNED_BYTE) )
        {
            // if the image doesn't have an alpha channel, we must convert it to
            // a format that does before continuing.
            image = ImageUtils::convertToRGBA8( image.get() );
        }           

        ImageUtils::PixelVisitor<ApplyChromaKey> applyChroma;
        applyChroma._chromaKey = _chromaKey;
        applyChroma.accept( image.get() );
    }

    // protected against multi threaded access. This is a requirement in sequential/preemptive mode, 
    // for example. This used to be in TextureCompositorTexArray::prepareImage.
    // TODO: review whether this affects performance.    
    image->setDataVariance( osg::Object::DYNAMIC );
}

//------------------------------------------------------------------------

ImageLayer::ImageLayer( const ImageLayerOptions& options ) :
TerrainLayer( options, &_runtimeOptions ),
_runtimeOptions( options )
{
    init();
}

ImageLayer::ImageLayer( const std::string& name, const TileSourceOptions& driverOptions ) :
TerrainLayer   ( ImageLayerOptions(name, driverOptions), &_runtimeOptions ),
_runtimeOptions( ImageLayerOptions(name, driverOptions) )
{
    init();
}

ImageLayer::ImageLayer( const ImageLayerOptions& options, TileSource* tileSource ) :
TerrainLayer   ( options, &_runtimeOptions, tileSource ),
_runtimeOptions( options )
{
    init();
}

void
ImageLayer::init()
{
    //nop
}

void
ImageLayer::addCallback( ImageLayerCallback* cb )
{
    _callbacks.push_back( cb );
}

void
ImageLayer::removeCallback( ImageLayerCallback* cb )
{
    ImageLayerCallbackList::iterator i = std::find( _callbacks.begin(), _callbacks.end(), cb );
    if ( i != _callbacks.end() ) 
        _callbacks.erase( i );
}

void
ImageLayer::fireCallback( TerrainLayerCallbackMethodPtr method )
{
    for( ImageLayerCallbackList::const_iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
    {
        ImageLayerCallback* cb = i->get();
        (cb->*method)( this );
    }
}

void
ImageLayer::fireCallback( ImageLayerCallbackMethodPtr method )
{
    for( ImageLayerCallbackList::const_iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
    {
        ImageLayerCallback* cb = i->get();
        (cb->*method)( this );
    }
}

void
ImageLayer::setOpacity( float value ) 
{
    _runtimeOptions.opacity() = osg::clampBetween( value, 0.0f, 1.0f );
    fireCallback( &ImageLayerCallback::onOpacityChanged );
}

void
ImageLayer::setHSLAdjust( const osg::Vec3f& hsl )
{
    _runtimeOptions.hslAdjust() = hsl;
    fireCallback( &ImageLayerCallback::onHSLChanged );
    OE_NOTICE << "Setting HSL to " << hsl.x() << ", " << hsl.y() << ", " << hsl.z()  << std::endl;
}

void
ImageLayer::setBCGAdjust( const osg::Vec3f& bcg )
{
    _runtimeOptions.bcgAdjust() = bcg;
    fireCallback( &ImageLayerCallback::onBCGChanged );
    OE_NOTICE << "Setting BCG to " << bcg.x() << ", " << bcg.y() << ", " << bcg.z()  << std::endl;
}

void 
ImageLayer::disableLODBlending()
{
    _runtimeOptions.lodBlending() = false;
}

void
ImageLayer::setTargetProfileHint( const Profile* profile )
{
    TerrainLayer::setTargetProfileHint( profile );

    // if we've already constructed the pre-cache operation, reinitialize it.
    if ( _preCacheOp.valid() )
        initPreCacheOp();
}

void
ImageLayer::initTileSource()
{
    // call superclass first.
    TerrainLayer::initTileSource();

    // install the pre-caching image processor operation.
    initPreCacheOp();
}

void
ImageLayer::initPreCacheOp()
{
    bool layerInTargetProfile = 
        _targetProfileHint.valid() &&
        getProfile()               &&
        _targetProfileHint->isEquivalentTo( getProfile() );

    ImageLayerPreCacheOperation* op = new ImageLayerPreCacheOperation();    
    op->_processor.init( _runtimeOptions, _dbOptions.get(), layerInTargetProfile );

    _preCacheOp = op;
}


CacheBin*
ImageLayer::getCacheBin( const Profile* profile )
{
    // specialize ImageLayer to only consider the horizontal signature (ignore vertical
    // datum component for images)
    std::string binId = *_runtimeOptions.cacheId() + "_" + profile->getHorizSignature();
    return TerrainLayer::getCacheBin( profile, binId );
}


GeoImage
ImageLayer::createImage( const TileKey& key, ProgressCallback* progress, bool forceFallback )
{
    bool isFallback;
    return createImageInKeyProfile( key, progress, forceFallback, isFallback);
}


GeoImage
ImageLayer::createImageInNativeProfile( const TileKey& key, ProgressCallback* progress, bool forceFallback, bool& out_isFallback)
{
    out_isFallback = false;

    const Profile* nativeProfile = getProfile();
    if ( !nativeProfile )
    {
        OE_WARN << LC << "Could not establish the profile for Layer \"" << getName() << "\"" << std::endl;
        return GeoImage::INVALID;
    }

    if ( key.getProfile()->isEquivalentTo(nativeProfile) )
    {
        // requested profile matches native profile, move along.
        return createImageInKeyProfile( key, progress, forceFallback, out_isFallback );
    }
    else
    {
        // find the intersection of keys.
        std::vector<TileKey> nativeKeys;
        nativeProfile->getIntersectingTiles(key.getExtent(), nativeKeys);
        
        // build a mosaic of the images from the native profile keys:
        bool foundAtLeastOneRealTile = false;

        ImageMosaic mosaic;
        for( std::vector<TileKey>::iterator k = nativeKeys.begin(); k != nativeKeys.end(); ++k )
        {
            bool isFallback = false;
            GeoImage image = createImageInKeyProfile( *k, progress, forceFallback, isFallback );
            if ( image.valid() )
            {
                mosaic.getImages().push_back( TileImage(image.getImage(), *k) );
                if ( !isFallback )
                    foundAtLeastOneRealTile = true;
            }
        }

        // bail out if we got nothing.
        if ( mosaic.getImages().size() == 0 )
            return GeoImage::INVALID;

        // if the mosaic is ALL fallback data, this tile is fallback data.
        if ( !foundAtLeastOneRealTile )
            out_isFallback = true;

        // assemble new GeoImage from the mosaic.
        double rxmin, rymin, rxmax, rymax;
        mosaic.getExtents( rxmin, rymin, rxmax, rymax );

        GeoImage result( 
            mosaic.createImage(), 
            GeoExtent( nativeProfile->getSRS(), rxmin, rymin, rxmax, rymax ) );

        // calculate a tigher extent that matches the original input key:
        GeoExtent tightExtent = nativeProfile->clampAndTransformExtent( key.getExtent() );

        // a non-exact crop is critical here to avoid resampling the data
        return result.crop( tightExtent, false );
    }
}


GeoImage
ImageLayer::createImageInKeyProfile( const TileKey& key, ProgressCallback* progress, bool forceFallback, bool& out_isFallback )
{
    GeoImage result;

    // If the layer is disabled, bail out.
    if ( !getEnabled() )
    {
        return GeoImage::INVALID;
    }

    OE_DEBUG << LC << 
        "Layer \"" << getName() << "\" create image for \"" << key.str() << "\", ext= "
        << key.getExtent().toString() << std::endl;


    // locate the cache bin for the target profile for this layer:
    CacheBin* cacheBin = getCacheBin( key.getProfile() );

    // validate that we have either a valid tile source, or we're cache-only.
    if ( ! (getTileSource() || (isCacheOnly() && cacheBin) ) )
    {
        OE_WARN << LC << "Error: layer does not have a valid TileSource, cannot create image " << std::endl;
        _runtimeOptions.enabled() = false;
        return GeoImage::INVALID;
    }

    // validate the existance of a valid layer profile (unless we're in cache-only mode, in which
    // case there is no layer profile)
    if ( !isCacheOnly() && !getProfile() )
    {
        OE_WARN << LC << "Could not establish a valid profile for Layer \"" << getName() << "\"" << std::endl;
        _runtimeOptions.enabled() = false;
        return GeoImage::INVALID;
    }

    // First, attempt to read from the cache. Since the cached data is stored in the
    // map profile, we can try this first.
    if ( cacheBin && _runtimeOptions.cachePolicy()->isCacheReadable() )
    {
        ReadResult r = cacheBin->readImage( key.str() );
        if ( r.succeeded() )
        {            
            //OE_INFO << LC << getName() << " : " << key.str() << " cache hit" << std::endl;
            ImageUtils::normalizeImage( r.getImage() );
            return GeoImage( r.releaseImage(), key.getExtent() );
        }
        else
        {
            //OE_INFO << LC << getName() << " : " << key.str() << " cache miss" << std::endl;
        }
    }
    
    // The data was not in the cache. If we are cache-only, fail sliently
    if ( isCacheOnly() )
    {
        return GeoImage::INVALID;
    }

    // Get an image from the underlying TileSource.
    bool isFallback = false;
    result = createImageFromTileSource( key, progress, forceFallback, isFallback );

    // Normalize the image if necessary
    if ( result.valid() )
    {
        ImageUtils::normalizeImage( result.getImage() );
    }

    // If we got a result, the cache is valid and we are caching in the map profile, write to the map cache.
    if (result.valid() &&
        !isFallback    &&
        cacheBin       && 
        _runtimeOptions.cachePolicy()->isCacheWriteable() )
    {
        if ( key.getExtent() != result.getExtent() )
        {
            OE_INFO << LC << "WARNING! mismatched extents." << std::endl;
        }

        cacheBin->write( key.str(), result.getImage() );
        OE_DEBUG << LC << "WRITING " << key.str() << " to the cache." << std::endl;
    }

    if ( result.valid() )
    {
        OE_DEBUG << LC << getName() << " : " << key.str() << " result OK" << std::endl;
    }
    else
    {
        OE_DEBUG << LC << getName() << " : " << key.str() << "result INVALID" << std::endl;
    }

    return result;
}



GeoImage
ImageLayer::createImageFromTileSource(const TileKey&    key,
                                      ProgressCallback* progress,
                                      bool              forceFallback,
                                      bool&             out_isFallback)
{
    // Results:
    // 
    // * return an osg::Image matching the key extent is all goes well;
    //
    // * return NULL to indicate that the key exceeds the maximum LOD of the source data,
    //   and that the engine may need to generate a "fallback" tile if necessary;
    //
    // deprecated:
    // * return an "empty image" if the LOD is valid BUT the key does not intersect the
    //   source's data extents.

    out_isFallback = false;

    TileSource* source = getTileSource();
    if ( !source )
        return GeoImage::INVALID;

    // If the profiles are different, use a compositing method to assemble the tile.
    if ( !key.getProfile()->isEquivalentTo( getProfile() ) )
    {
        return assembleImageFromTileSource( key, progress, forceFallback, out_isFallback );
    }

    // Fail is the image is blacklisted.
    // ..unless there will be a fallback attempt.
    if ( source->getBlacklist()->contains( key.getTileId() ) && !forceFallback )
    {
        OE_DEBUG << LC << "createImageFromTileSource: blacklisted(" << key.str() << ")" << std::endl;
        return GeoImage::INVALID;
    }

    // Fail if no data is available for this key.
    if ( !source->hasDataAtLOD( key.getLevelOfDetail() ) && !forceFallback )
    {
        OE_DEBUG << LC << "createImageFromTileSource: hasDataAtLOD(" << key.str() << ") == false" << std::endl;
        return GeoImage::INVALID;
    }

    if ( !source->hasDataInExtent( key.getExtent() ) )
    {
        OE_DEBUG << LC << "createImageFromTileSource: hasDataInExtent(" << key.str() << ") == false" << std::endl;
        return GeoImage::INVALID;
    }

    // Good to go, ask the tile source for an image:
    osg::ref_ptr<TileSource::ImageOperation> op = _preCacheOp;

    osg::ref_ptr<osg::Image> result;
    TileKey finalKey = key;
    bool fellBack = false;

    if ( forceFallback )
    {        
        while( !result.valid() && finalKey.valid() )
        {
            if ( !source->getBlacklist()->contains( finalKey.getTileId() ) )
            {
                result = source->createImage( finalKey, op.get(), progress );
                if ( result.valid() )
                {
                    if ( finalKey.getLevelOfDetail() != key.getLevelOfDetail() )
                    {
                        GeoImage raw( result.get(), finalKey.getExtent() );
                        GeoImage cropped = raw.crop( key.getExtent(), true );
                        result = cropped.takeImage();
                        fellBack = true;
                    }
                }
            }
            if ( !result.valid() )
            {
                finalKey = finalKey.createParentKey();
                out_isFallback = true;
            }
        }

        if ( !result.valid() )
        {
            result = ImageUtils::createEmptyImage();
            finalKey = key;
        }
    }

    else
    {
        result = source->createImage( key, op.get(), progress );
    }
    
    // If image creation failed (but was not intentionally canceled),
    // blacklist this tile for future requests.
    if ( result == 0L && (!progress || !progress->isCanceled()) )
    {
        source->getBlacklist()->add( key.getTileId() );
    }

    //return result.release();
    return GeoImage(result.get(), finalKey.getExtent());
}


GeoImage
ImageLayer::assembleImageFromTileSource(const TileKey&    key,
                                        ProgressCallback* progress,
                                        bool              forceFallback,
                                        bool&             out_isFallback)
{
    GeoImage mosaic, result;

    out_isFallback = false;

    // Scale the extent if necessary to apply an "edge buffer"
    GeoExtent ext = key.getExtent();
    if ( _runtimeOptions.edgeBufferRatio().isSet() )
    {
        double ratio = _runtimeOptions.edgeBufferRatio().get();
        ext.scale(ratio, ratio);
    }

    // Get a set of layer tiles that intersect the requested extent.
    std::vector<TileKey> intersectingTiles;
    getProfile()->getIntersectingTiles( ext, intersectingTiles );

    if ( intersectingTiles.size() > 0 )
    {
        double dst_minx, dst_miny, dst_maxx, dst_maxy;
        key.getExtent().getBounds(dst_minx, dst_miny, dst_maxx, dst_maxy);

        osg::ref_ptr<ImageMosaic> mi = new ImageMosaic();
        //std::vector<TileKey> missingTiles;

        // if we find at least one "real" tile in the mosaic, then the whole result tile is
        // "real" (i.e. not a fallback tile)
        bool foundAtLeastOneRealTile = false;

        bool retry = false;
        for (unsigned int j = 0; j < intersectingTiles.size(); ++j)
        {
            double minX, minY, maxX, maxY;
            intersectingTiles[j].getExtent().getBounds(minX, minY, maxX, maxY);

            GeoImage img;

            if ( forceFallback )
            {
                bool tileIsFallback = false;

                TileKey finalKey = intersectingTiles[j];

                while( !img.valid() && finalKey.valid() )
                {
                    bool dummy = false; // will not be populated, since forceFallback=false in the following call
                    img = createImageFromTileSource( finalKey, progress, false, dummy );

                    if ( img.valid() )
                    {
                        if ( finalKey.getLevelOfDetail() < intersectingTiles[j].getLevelOfDetail() )
                        {
                            GeoImage raw( img.getImage(), finalKey.getExtent() );
                            GeoImage cropped = raw.crop( intersectingTiles[j].getExtent() );
                            img = cropped;
                        }
                        else
                        {
                            foundAtLeastOneRealTile = true;
                        }
                    }
                    else
                    {
                        finalKey = finalKey.createParentKey();
                    }
                }
            }
            else
            {
                bool dummy = false; // will not be populated, since forceFallback=false in the following call
                img = createImageFromTileSource( intersectingTiles[j], progress, false, dummy );
                foundAtLeastOneRealTile = img.valid();
            }

            if ( img.valid() )
            {
                // make sure the image is RGBA:
                if (img.getImage()->getPixelFormat() != GL_RGBA || img.getImage()->getDataType() != GL_UNSIGNED_BYTE || img.getImage()->getInternalTextureFormat() != GL_RGBA8 )
                {
                    osg::ref_ptr<osg::Image> convertedImg = ImageUtils::convertToRGBA8(img.getImage());
                    if (convertedImg.valid())
                    {
                        img = GeoImage(convertedImg, img.getExtent());
                    }
                }
                // add it to our list of images to be mosaiced:
                mi->getImages().push_back( TileImage(img.getImage(), intersectingTiles[j]) );
            }
            else
            {
                // the tile source did not return a tile, so make a note of it.
                if (progress && (progress->isCanceled() || progress->needsRetry()))
                {
                    retry = true;
                    break;
                }
#if 0
                missingTiles.push_back( intersectingTiles[j] );
#endif
            }
        }

        if ( mi->getImages().empty() || retry )
        {
            // if we didn't get any data, fail
            OE_DEBUG << LC << "Couldn't create image for ImageMosaic " << std::endl;
            return GeoImage::INVALID;
        }

#if 0 // gw: no longer necessary since we're using GeoImage::reproject to crop/resample:
      // when confirmed by testing, remove this code

        else if ( missingTiles.size() > 0 )
        {
            // if we have missing tiles, replace them with transparent regions:
            osg::ref_ptr<const osg::Image> validImage = mi->getImages()[0].getImage();
            unsigned int tileWidth = validImage->s();
            unsigned int tileHeight = validImage->t();
            unsigned int tileDepth = validImage->r();

            for (unsigned int j = 0; j < missingTiles.size(); ++j)
            {
                // Create transparent image which size equals to the size of a valid image
                osg::ref_ptr<osg::Image> newImage = new osg::Image;
                newImage->allocateImage(tileWidth, tileHeight, tileDepth, validImage->getPixelFormat(), validImage->getDataType());
                unsigned char *data = newImage->data(0,0);
                memset(data, 0, newImage->getTotalSizeInBytes());

                mi->getImages().push_back( TileImage(newImage.get(), missingTiles[j]) );
            }
        }
#endif

        // all set. Mosaic all the images together.
        double rxmin, rymin, rxmax, rymax;
        mi->getExtents( rxmin, rymin, rxmax, rymax );

        mosaic = GeoImage(
            mi->createImage(),
            GeoExtent( getProfile()->getSRS(), rxmin, rymin, rxmax, rymax ) );

        if ( !foundAtLeastOneRealTile )
            out_isFallback = true;
    }
    else
    {
        OE_DEBUG << LC << "assembleImageFromTileSource: no intersections (" << key.str() << ")" << std::endl;
    }

    // Final step: transform the mosaic into the requesting key's extent.
    if ( mosaic.valid() )
    {
        // GeoImage::reproject() will automatically crop the image to the correct extents.
        // so there is no need to crop after reprojection. Also note that if the SRS's are the 
        // same (even though extents are different), then this operation is technically not a
        // reprojection but merely a resampling.

        result = mosaic.reproject( 
            key.getProfile()->getSRS(),
            &key.getExtent(), 
            *_runtimeOptions.reprojectedTileSize(),
            *_runtimeOptions.reprojectedTileSize() );
    }

    return result;
}
