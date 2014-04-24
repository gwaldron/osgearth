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
#include <osgEarth/ImageLayer>
#include <osgEarth/ColorFilter>
#include <osgEarth/TileSource>
#include <osgEarth/ImageMosaic>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/Progress>
#include <osgEarth/URI>
#include <osgEarth/MemCache>
#include <osg/Version>
#include <osgDB/WriteFile>
#include <memory.h>
#include <limits.h>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[ImageLayer] \"" << getName() << "\" "

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
    _minRange.init( -FLT_MAX );
    _maxRange.init( FLT_MAX );
    _lodBlending.init( false );
    _featherPixels.init( false );
    _minFilter.init( osg::Texture::LINEAR );
    _magFilter.init( osg::Texture::LINEAR );
    _texcomp.init( (osg::Texture::InternalFormatMode)~0 );
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
    conf.getIfSet( "nodata_image",   _noDataImageFilename );
    conf.getIfSet( "opacity",        _opacity );
    conf.getIfSet( "min_range",      _minRange );
    conf.getIfSet( "max_range",      _maxRange );
    conf.getIfSet( "lod_blending",   _lodBlending );
    conf.getIfSet( "shared",         _shared );
    conf.getIfSet( "feather_pixels", _featherPixels);

    if ( conf.hasValue( "transparent_color" ) )
        _transparentColor = stringToColor( conf.value( "transparent_color" ), osg::Vec4ub(0,0,0,0));

    if ( conf.hasChild("color_filters") )
    {
        _colorFilters.clear();
        ColorFilterRegistry::instance()->readChain( conf.child("color_filters"), _colorFilters );
    }

    conf.getIfSet("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.getIfSet("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.getIfSet("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.getIfSet("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.getIfSet("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.getIfSet("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.getIfSet("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.getIfSet("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.getIfSet("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.getIfSet("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.getIfSet("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.getIfSet("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    conf.getIfSet("texture_compression", "none", _texcomp, osg::Texture::USE_IMAGE_DATA_FORMAT);
    conf.getIfSet("texture_compression", "auto", _texcomp, (osg::Texture::InternalFormatMode)~0);
    //TODO add all the enums
}

Config
ImageLayerOptions::getConfig( bool isolate ) const
{
    Config conf = TerrainLayerOptions::getConfig( isolate );

    conf.updateIfSet( "nodata_image",   _noDataImageFilename );
    conf.updateIfSet( "opacity",        _opacity );
    conf.updateIfSet( "min_range",      _minRange );
    conf.updateIfSet( "max_range",      _maxRange );
    conf.updateIfSet( "lod_blending",   _lodBlending );
    conf.updateIfSet( "shared",         _shared );
    conf.updateIfSet( "feather_pixels", _featherPixels );

    if (_transparentColor.isSet())
        conf.update("transparent_color", colorToString( _transparentColor.value()));

    if ( _colorFilters.size() > 0 )
    {
        Config filtersConf("color_filters");
        if ( ColorFilterRegistry::instance()->writeChain( _colorFilters, filtersConf ) )
        {
            conf.add( filtersConf );
        }
    }

    conf.updateIfSet("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.updateIfSet("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.updateIfSet("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.updateIfSet("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.updateIfSet("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.updateIfSet("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.updateIfSet("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.updateIfSet("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.updateIfSet("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.updateIfSet("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.updateIfSet("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.updateIfSet("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    conf.updateIfSet("texture_compression", "none", _texcomp, osg::Texture::USE_IMAGE_DATA_FORMAT);
    conf.updateIfSet("texture_compression", "auto", _texcomp, (osg::Texture::InternalFormatMode)~0);
    //TODO add all the enums

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
        _noDataImage = _options.noDataImageFilename()->getImage( dbOptions );
        if ( !_noDataImage.valid() )
        {
            OE_WARN << "Failed to read nodata image from \"" << _options.noDataImageFilename()->full() << "\"" << std::endl;
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
    // target profile...because if it's not in the target profile, we will have to do
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
    // Set the tile size to 256 if it's not explicitly set.
    if (!_runtimeOptions.driver()->tileSize().isSet())
    {
        _runtimeOptions.driver()->tileSize().init( 256 );
    }

    _emptyImage = ImageUtils::createEmptyImage();
    //*((unsigned*)_emptyImage->data()) = 0x7F0000FF;
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
ImageLayer::setMinVisibleRange( float minVisibleRange )
{
    _runtimeOptions.minVisibleRange() = minVisibleRange;
    fireCallback( &ImageLayerCallback::onVisibleRangeChanged );
}

void
ImageLayer::setMaxVisibleRange( float maxVisibleRange )
{
    _runtimeOptions.maxVisibleRange() = maxVisibleRange;
    fireCallback( &ImageLayerCallback::onVisibleRangeChanged );
}

void
ImageLayer::addColorFilter( ColorFilter* filter )
{
    _runtimeOptions.colorFilters().push_back( filter );
    fireCallback( &ImageLayerCallback::onColorFiltersChanged );
}

void
ImageLayer::removeColorFilter( ColorFilter* filter )
{
    ColorFilterChain& filters = _runtimeOptions.colorFilters();
    ColorFilterChain::iterator i = std::find(filters.begin(), filters.end(), filter);
    if ( i != filters.end() )
    {
        filters.erase( i );
        fireCallback( &ImageLayerCallback::onColorFiltersChanged );
    }
}

const ColorFilterChain&
ImageLayer::getColorFilters() const
{
    return _runtimeOptions.colorFilters();
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
ImageLayer::getCacheBin( const Profile* profile)
{
    // specialize ImageLayer to only consider the horizontal signature (ignore vertical
    // datum component for images)
    std::string binId = *_runtimeOptions.cacheId() + "_" + profile->getHorizSignature();
    return TerrainLayer::getCacheBin( profile, binId );
}


GeoImage
ImageLayer::createImage(const TileKey&    key,
                        ProgressCallback* progress)
{
    return createImageInKeyProfile( key, progress );
}


GeoImage
ImageLayer::createImageInNativeProfile(const TileKey&    key,
                                       ProgressCallback* progress)
{
    GeoImage result;

    const Profile* nativeProfile = getProfile();
    if ( !nativeProfile )
    {
        OE_WARN << LC << "Could not establish the profile" << std::endl;
        return GeoImage::INVALID;
    }

    if ( key.getProfile()->isHorizEquivalentTo(nativeProfile) )
    {
        // requested profile matches native profile, move along.
        result = createImageInKeyProfile( key, progress );
    }
    else
    {
        // find the intersection of keys.
        std::vector<TileKey> nativeKeys;
        nativeProfile->getIntersectingTiles(key, nativeKeys);

        // build a mosaic of the images from the native profile keys:
        bool foundAtLeastOneRealTile = false;

        ImageMosaic mosaic;
        for( std::vector<TileKey>::iterator k = nativeKeys.begin(); k != nativeKeys.end(); ++k )
        {
            bool isFallback = false;
            GeoImage image = createImageInKeyProfile( *k, progress );
            if ( image.valid() )
            {
                mosaic.getImages().push_back( TileImage(image.getImage(), *k) );
            }
            else
            {
                // if we get EVEN ONE invalid tile, we have to abort because there will be
                // empty spots in the mosaic. (By "invalid" we mean a tile that could not
                // even be resolved through the fallback procedure.)
                return GeoImage::INVALID;
                //TODO: probably need to change this so the mosaic uses alpha.
            }
        }

        // bail out if we got nothing.
        if ( mosaic.getImages().size() > 0 )
        {
            // assemble new GeoImage from the mosaic.
            double rxmin, rymin, rxmax, rymax;
            mosaic.getExtents( rxmin, rymin, rxmax, rymax );

            result = GeoImage(
                mosaic.createImage(), 
                GeoExtent( nativeProfile->getSRS(), rxmin, rymin, rxmax, rymax ) );
        }
    }

    return result;
}


GeoImage
ImageLayer::createImageInKeyProfile(const TileKey&    key, 
                                    ProgressCallback* progress)
{
    GeoImage result;

    // If the layer is disabled, bail out.
    if ( !getEnabled() )
    {
        return GeoImage::INVALID;
    }

    // Make sure the request is in range.
    if ( !isKeyInRange(key) )
    {
        return GeoImage::INVALID;
    }

    OE_DEBUG << LC << "create image for \"" << key.str() << "\", ext= "
        << key.getExtent().toString() << std::endl;
    
    // Check the layer L2 cache first
    if ( _memCache.valid() )
    {
        CacheBin* bin = _memCache->getOrCreateBin( key.getProfile()->getFullSignature() );        
        ReadResult result = bin->readObject(key.str(), 0);
        if ( result.succeeded() )
            return GeoImage(static_cast<osg::Image*>(result.releaseObject()), key.getExtent());
        //_memCache->dumpStats(key.getProfile()->getFullSignature());
    }

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
        OE_WARN << LC << "Could not establish a valid profile" << std::endl;
        _runtimeOptions.enabled() = false;
        return GeoImage::INVALID;
    }

    // First, attempt to read from the cache. Since the cached data is stored in the
    // map profile, we can try this first.
    if ( cacheBin && getCachePolicy().isCacheReadable() )
    {
        ReadResult r = cacheBin->readImage( key.str(), getCachePolicy().getMinAcceptTime() );
        if ( r.succeeded() )
        {
            ImageUtils::normalizeImage( r.getImage() );
            return GeoImage( r.releaseImage(), key.getExtent() );
        }
    }
    
    // The data was not in the cache. If we are cache-only, fail sliently
    if ( isCacheOnly() )
    {
        return GeoImage::INVALID;
    }

    // Get an image from the underlying TileSource.
    result = createImageFromTileSource( key, progress );

    // Normalize the image if necessary
    if ( result.valid() )
    {
        ImageUtils::normalizeImage( result.getImage() );
    }

    // memory cache first:
    if ( result.valid() && _memCache.valid() )
    {
        CacheBin* bin = _memCache->getOrCreateBin( key.getProfile()->getFullSignature() ); 
        bin->write(key.str(), result.getImage());
    }

    // If we got a result, the cache is valid and we are caching in the map profile,
    // write to the map cache.
    if (result.valid()  &&
        cacheBin        && 
        getCachePolicy().isCacheWriteable() )
    {
        if ( key.getExtent() != result.getExtent() )
        {
            OE_INFO << LC << "WARNING! mismatched extents." << std::endl;
        }

        cacheBin->write( key.str(), result.getImage() );
    }

    if ( result.valid() )
    {
        OE_DEBUG << LC << key.str() << " result OK" << std::endl;
    }
    else
    {
        OE_DEBUG << LC << key.str() << "result INVALID" << std::endl;
    }

    return result;
}



GeoImage
ImageLayer::createImageFromTileSource(const TileKey&    key,
                                      ProgressCallback* progress)
{
    TileSource* source = getTileSource();
    if ( !source )
        return GeoImage::INVALID;

    // If the profiles are different, use a compositing method to assemble the tile.
    if ( !key.getProfile()->isHorizEquivalentTo( getProfile() ) )
    {
        return assembleImageFromTileSource( key, progress );
    }

    // Good to go, ask the tile source for an image:
    osg::ref_ptr<TileSource::ImageOperation> op = _preCacheOp;

    // Fail is the image is blacklisted.
    if ( source->getBlacklist()->contains( key.getTileId() ) )
    {
        OE_DEBUG << LC << "createImageFromTileSource: blacklisted(" << key.str() << ")" << std::endl;
        return GeoImage::INVALID;
    }
    
    if ( !source->hasData( key ) )
    {
        OE_DEBUG << LC << "createImageFromTileSource: hasData(" << key.str() << ") == false" << std::endl;
        return GeoImage::INVALID;
    }

    // create an image from the tile source.
    osg::ref_ptr<osg::Image> result = source->createImage( key, op.get(), progress );

    // Process images with full alpha to properly support MP blending.    
    if ( result.valid() && *_runtimeOptions.featherPixels())
    {
        ImageUtils::featherAlphaRegions( result.get() );
    }    
    
    // If image creation failed (but was not intentionally canceled),
    // blacklist this tile for future requests.
    if ( result == 0L && (!progress || !progress->isCanceled()) )
    {
        source->getBlacklist()->add( key.getTileId() );
    }

    return GeoImage(result.get(), key.getExtent());
}


GeoImage
ImageLayer::assembleImageFromTileSource(const TileKey&    key,
                                        ProgressCallback* progress)
{
    GeoImage mosaicedImage, result;

    // Scale the extent if necessary to apply an "edge buffer"
    GeoExtent ext = key.getExtent();
    if ( _runtimeOptions.edgeBufferRatio().isSet() )
    {
        double ratio = _runtimeOptions.edgeBufferRatio().get();
        ext.scale(ratio, ratio);
    }

    // Get a set of layer tiles that intersect the requested extent.
    std::vector<TileKey> intersectingKeys;
    getProfile()->getIntersectingTiles( key, intersectingKeys );

    if ( intersectingKeys.size() > 0 )
    {
        double dst_minx, dst_miny, dst_maxx, dst_maxy;
        key.getExtent().getBounds(dst_minx, dst_miny, dst_maxx, dst_maxy);

        // if we find at least one "real" tile in the mosaic, then the whole result tile is
        // "real" (i.e. not a fallback tile)
        bool retry = false;
        ImageMosaic mosaic;

        for( std::vector<TileKey>::iterator k = intersectingKeys.begin(); k != intersectingKeys.end(); ++k )
        {
            double minX, minY, maxX, maxY;
            k->getExtent().getBounds(minX, minY, maxX, maxY);

            GeoImage image = createImageFromTileSource( *k, progress );
            if ( image.valid() )
            {
                ImageUtils::normalizeImage(image.getImage());

                // Make sure all images in mosaic are based on "RGBA - unsigned byte" pixels.
                // This is not the smarter choice (in some case RGB would be sufficient) but
                // it ensure consistency between all images / layers.
                //
                // The main drawback is probably the CPU memory foot-print which would be reduced by allocating RGB instead of RGBA images.
                // On GPU side, this should not change anything because of data alignements : often RGB and RGBA textures have the same memory footprint
                //
                if (   (image.getImage()->getDataType() != GL_UNSIGNED_BYTE)
                    || (image.getImage()->getPixelFormat() != GL_RGBA) )
                {
                    osg::ref_ptr<osg::Image> convertedImg = ImageUtils::convertToRGBA8(image.getImage());
                    if (convertedImg.valid())
                    {
                        image = GeoImage(convertedImg, image.getExtent());
                    }
                }

                mosaic.getImages().push_back( TileImage(image.getImage(), *k) );
            }
            else
            {
                // the tile source did not return a tile, so make a note of it.
                if (progress && (progress->isCanceled() || progress->needsRetry()))
                {
                    retry = true;
                    break;
                }
            }
        }

        if ( mosaic.getImages().empty() || retry )
        {
            // if we didn't get any data, fail.
            OE_DEBUG << LC << "Couldn't create image for ImageMosaic " << std::endl;
            return GeoImage::INVALID;
        }

        // all set. Mosaic all the images together.
        double rxmin, rymin, rxmax, rymax;
        mosaic.getExtents( rxmin, rymin, rxmax, rymax );

        mosaicedImage = GeoImage(
            mosaic.createImage(),
            GeoExtent( getProfile()->getSRS(), rxmin, rymin, rxmax, rymax ) );
    }
    else
    {
        OE_DEBUG << LC << "assembleImageFromTileSource: no intersections (" << key.str() << ")" << std::endl;
    }

    // Final step: transform the mosaic into the requesting key's extent.
    if ( mosaicedImage.valid() )
    {
        // GeoImage::reproject() will automatically crop the image to the correct extents.
        // so there is no need to crop after reprojection. Also note that if the SRS's are the 
        // same (even though extents are different), then this operation is technically not a
        // reprojection but merely a resampling.

        result = mosaicedImage.reproject( 
            key.getProfile()->getSRS(),
            &key.getExtent(), 
            *_runtimeOptions.reprojectedTileSize(),
            *_runtimeOptions.reprojectedTileSize(),
            *_runtimeOptions.driver()->bilinearReprojection());
    }

    // Process images with full alpha to properly support MP blending.
    if ( result.valid() && *_runtimeOptions.featherPixels() )
    {
        ImageUtils::featherAlphaRegions( result.getImage() );
    }

    return result;
}


void
ImageLayer::applyTextureCompressionMode(osg::Texture* tex) const
{
    if ( tex == 0L )
        return;

    if ( _runtimeOptions.textureCompression() == (osg::Texture::InternalFormatMode)~0 )
    {
        // auto - pick a good one.
        osg::Texture::InternalFormatMode mode;
        if (ImageUtils::computeTextureCompressionMode(tex->getImage(0), mode))
        {
            tex->setInternalFormatMode(mode);
        }
    }
    else if ( _runtimeOptions.textureCompression().isSet() )
    {
        tex->setInternalFormatMode( *_runtimeOptions.textureCompression() );
    }
}
