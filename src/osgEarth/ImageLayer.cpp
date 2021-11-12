/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/ImageMosaic>
#include <osgEarth/Registry>
#include <osgEarth/Progress>
#include <osgEarth/Capabilities>
#include <osgEarth/Metrics>
#include <osgEarth/NetworkMonitor>
#include <osgEarth/TimeSeriesImage>
#include <osgEarth/Random>
#include <osgEarth/MetaTile>
#include <osg/ImageStream>
#include <cinttypes>

using namespace osgEarth;

#define LC "[ImageLayer] \"" << getName() << "\" "

// TESTING
//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

//------------------------------------------------------------------------

void
ImageLayer::Options::fromConfig(const Config& conf)
{
    _transparentColor.setDefault( osg::Vec4ub(0,0,0,0) );
    _minFilter.setDefault( osg::Texture::LINEAR_MIPMAP_LINEAR );
    _magFilter.setDefault( osg::Texture::LINEAR );
    _textureCompression.setDefault("");
    _shared.setDefault( false );
    _coverage.setDefault( false );
    _reprojectedTileSize.setDefault( 256 );

    conf.get( "nodata_image",   _noDataImageFilename );
    conf.get( "shared",         _shared );
    conf.get( "coverage",       _coverage );
    conf.get( "altitude",       _altitude );
    conf.get( "accept_draping", acceptDraping());
    conf.get( "edge_buffer_ratio", _edgeBufferRatio);
    conf.get( "reprojected_tilesize", _reprojectedTileSize);

    if ( conf.hasValue( "transparent_color" ) )
        _transparentColor = stringToColor( conf.value( "transparent_color" ), osg::Vec4ub(0,0,0,0));

    if ( conf.hasChild("color_filters") )
    {
        _colorFilters->clear();
        ColorFilterRegistry::instance()->readChain( conf.child("color_filters"), _colorFilters.mutable_value() );
    }

    conf.get("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.get("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.get("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.get("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.get("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.get("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.get("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.get("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.get("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.get("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.get("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.get("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    conf.get("texture_compression", textureCompression());

    // uniform names
    conf.get("shared_sampler", _shareTexUniformName);
    conf.get("shared_matrix",  _shareTexMatUniformName);

    // automatically set shared=true if the uniform name is set.
    if (shareTexUniformName().isSet() && !shared().isSet())
        shared() = true;
    
    conf.get("async", async());
}

Config
ImageLayer::Options::getConfig() const
{
    Config conf = TileLayer::Options::getConfig();

    conf.set( "nodata_image",   _noDataImageFilename );
    conf.set( "shared",         _shared );
    conf.set( "coverage",       _coverage );
    conf.set( "altitude",       _altitude );
    conf.set( "accept_draping", acceptDraping());
    conf.set( "edge_buffer_ratio", _edgeBufferRatio);
    conf.set( "reprojected_tilesize", _reprojectedTileSize);

    if (_transparentColor.isSet())
        conf.set("transparent_color", colorToString( _transparentColor.value()));

    if ( _colorFilters->size() > 0 )
    {
        Config filtersConf("color_filters");
        if ( ColorFilterRegistry::instance()->writeChain( _colorFilters.get(), filtersConf ) )
        {
            conf.set( filtersConf );
        }
    }

    conf.set("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.set("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.set("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.set("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.set("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.set("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.set("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.set("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.set("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.set("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.set("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.set("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    conf.set("texture_compression", textureCompression());

    // uniform names
    conf.set("shared_sampler", _shareTexUniformName);
    conf.set("shared_matrix",  _shareTexMatUniformName);

    conf.set("async", async());

    return conf;
}

//------------------------------------------------------------------------

void
ImageLayer::setShared(bool value)
{
    setOptionThatRequiresReopen(options().shared(), value);
}

bool
ImageLayer::getShared() const
{
    return options().shared().get();
}

void
ImageLayer::setCoverage(bool value)
{
    setOptionThatRequiresReopen(options().coverage(), value);
}

bool
ImageLayer::getCoverage() const
{
    return options().coverage().get();
}

void
ImageLayer::setSharedTextureUniformName(const std::string& value)
{
    if (options().shareTexUniformName() != value)
        options().shareTexUniformName() = value;
}

const std::string&
ImageLayer::getSharedTextureUniformName() const
{
    return options().shareTexUniformName().get();
}

void
ImageLayer::setSharedTextureMatrixUniformName(const std::string& value)
{
    if (options().shareTexMatUniformName() != value)
        options().shareTexMatUniformName() = value;
}

const std::string&
ImageLayer::getSharedTextureMatrixUniformName() const
{
    return options().shareTexMatUniformName().get();
}

void
ImageLayer::setAsyncLoading(bool value)
{
    options().async() = value;
}

bool
ImageLayer::getAsyncLoading() const
{
    return options().async().get();
}

ImageLayer*
ImageLayer::create(const ConfigOptions& options)
{
    osg::ref_ptr<Layer> layer = Layer::create(options);
    ImageLayer* result = dynamic_cast<ImageLayer*>(layer.get());
    if (result)
    {
        layer.release();
        return result;
    }
    return 0L;
}

Status
ImageLayer::openImplementation()
{
    Status parent = TileLayer::openImplementation();
    if (parent.isError())
        return parent;

    if (!_emptyImage.valid())
        _emptyImage = ImageUtils::createEmptyImage();

    if (!options().shareTexUniformName().isSet())
        options().shareTexUniformName().init( Stringify() << "layer_" << getUID() << "_tex" );

    if (!options().shareTexMatUniformName().isSet() )
        options().shareTexMatUniformName().init(Stringify() << options().shareTexUniformName().get() << "_matrix");

    return Status::NoError;
}

void
ImageLayer::init()
{
    TileLayer::init();

    _useCreateTexture = false;
    _sentry.setName("ImageLayer " + getName());

    // image layers render as a terrain texture.
    setRenderType(RENDERTYPE_TERRAIN_SURFACE);

    if (options().altitude().isSet())
    {
        setAltitude(options().altitude().get());
    }

    if (options().acceptDraping().isSet())
    {
        setAcceptDraping(options().acceptDraping().get());
    }
}


void
ImageLayer::setAltitude(const Distance& value)
{
    options().altitude() = value;

    if (value != 0.0)
    {
        osg::StateSet* stateSet = getOrCreateStateSet();

        stateSet->addUniform(
            new osg::Uniform("oe_terrain_altitude", (float)options().altitude()->as(Units::METERS)),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

        stateSet->setMode(GL_CULL_FACE, 0);
    }
    else
    {
        osg::StateSet* stateSet = getOrCreateStateSet();
        getOrCreateStateSet()->removeUniform("oe_terrain_altitude");
        stateSet->removeMode(GL_CULL_FACE);
    }
}

const Distance&
ImageLayer::getAltitude() const
{
    return options().altitude().get();
}

void
ImageLayer::setAcceptDraping(bool value)
{
    options().acceptDraping() = value;

    if (value == true && getStateSet() != nullptr)
        getStateSet()->removeDefine("OE_DISABLE_DRAPING");
    else
        getOrCreateStateSet()->setDefine("OE_DISABLE_DRAPING");
}

bool
ImageLayer::getAcceptDraping() const
{
    return options().acceptDraping().get();
}

void
ImageLayer::invoke_onCreate(const TileKey& key, GeoImage& data)
{
    if (_callbacks.empty() == false) // not thread-safe but that's ok
    {
        // Copy the vector to prevent thread lockup
        Callbacks temp;

        _callbacks.lock();
        temp = _callbacks;
        _callbacks.unlock();

        for(Callbacks::const_iterator i = temp.begin();
            i != temp.end();
            ++i)
        {
            i->get()->onCreate(key, data);
        }
    }
}

void
ImageLayer::setUseCreateTexture()
{
    _useCreateTexture = true;
}

void
ImageLayer::addColorFilter( ColorFilter* filter )
{
    options().colorFilters()->push_back( filter );
}

void
ImageLayer::removeColorFilter( ColorFilter* filter )
{
    ColorFilterChain& filters = options().colorFilters().mutable_value();
    ColorFilterChain::iterator i = std::find(filters.begin(), filters.end(), filter);
    if ( i != filters.end() )
    {
        filters.erase( i );
    }
}

const ColorFilterChain&
ImageLayer::getColorFilters() const
{
    return options().colorFilters().get();
}

GeoImage
ImageLayer::createImage(
    const TileKey& key)
{
    return createImage(key, nullptr);
}

GeoImage
ImageLayer::createImage(
    const TileKey& key,
    ProgressCallback* progress)
{
    OE_PROFILING_ZONE;
    OE_PROFILING_ZONE_TEXT(getName() + " " + key.str());

    if (!isOpen())
    {
        return GeoImage::INVALID;
    }

    NetworkMonitor::ScopedRequestLayer layerRequest(getName());

    GeoImage result = createImageInKeyProfile( key, progress );

    // Post-cache operations:

    for (auto& post : _postLayers)
    {
        result = post->createImage(result, key, progress);
    }

    if (result.valid())
    {
        postCreateImageImplementation(result, key, progress);
    }

    return result;
}

GeoImage
ImageLayer::createImage(
    const GeoImage& canvas,
    const TileKey& key,
    ProgressCallback* progress)
{
    return createImageImplementation(canvas, key, progress);
}

GeoImage
ImageLayer::createImageInKeyProfile(
    const TileKey& key,
    ProgressCallback* progress)
{
    // If the layer is disabled, bail out.
    if ( !isOpen() )
    {
        return GeoImage::INVALID;
    }

    // Make sure the request is in range.
    // TODO: perhaps this should be a call to mayHaveData(key) instead.
    if ( !isKeyInLegalRange(key) )
    {
        return GeoImage::INVALID;
    }

    // Tile gate prevents two threads from requesting the same key
    // at the same time, which would be unnecessary work. Only lock
    // the gate if there is an L2 cache active
    ScopedGate<TileKey> scopedGate(_sentry, key, [&]() {
        return _memCache.valid();
    });

    GeoImage result;

    OE_DEBUG << LC << "create image for \"" << key.str() << "\", ext= "
        << key.getExtent().toString() << std::endl;

    // the cache key combines the Key and the horizontal profile.
    std::string cacheKey = Cache::makeCacheKey(
        Stringify() << key.str() << "-" << std::hex << key.getProfile()->getHorizSignature(),
        "image");

    // The L2 cache key includes the layer revision of course!
    char memCacheKey[64];

    const CachePolicy& policy = getCacheSettings()->cachePolicy().get();

    // Check the layer L2 cache first
    if ( _memCache.valid() )
    {
        sprintf(memCacheKey, "%d/%s/%s", 
            getRevision(), 
            key.str().c_str(), 
            key.getProfile()->getHorizSignature().c_str());

        CacheBin* bin = _memCache->getOrCreateDefaultBin();
        ReadResult result = bin->readObject(memCacheKey, 0L);
        if (result.succeeded())
        {
            return GeoImage(static_cast<osg::Image*>(result.releaseObject()), key.getExtent());
        }
    }

    // locate the cache bin for the target profile for this layer:
    CacheBin* cacheBin = getCacheBin( key.getProfile() );

    // validate the existance of a valid layer profile (unless we're in cache-only mode, in which
    // case there is no layer profile)
    if ( !policy.isCacheOnly() && !getProfile() )
    {
        disable("Could not establish a valid profile");
        return GeoImage::INVALID;
    }

    osg::ref_ptr< osg::Image > cachedImage;

    // First, attempt to read from the cache. Since the cached data is stored in the
    // map profile, we can try this first.
    if ( cacheBin && policy.isCacheReadable() )
    {
        ReadResult r = cacheBin->readImage(cacheKey, 0L);
        if ( r.succeeded() )
        {
            cachedImage = r.releaseImage();
            bool expired = policy.isExpired(r.lastModifiedTime());
            if (!expired)
            {
                OE_DEBUG << "Got cached image for " << key.str() << std::endl;
                return GeoImage(cachedImage.get(), key.getExtent());
            }
            else
            {
                OE_DEBUG << "Expired image for " << key.str() << std::endl;
            }
        }
    }

    // The data was not in the cache. If we are cache-only, fail sliently
    if ( policy.isCacheOnly() )
    {
        // If it's cache only and we have an expired but cached image, just return it.
        if (cachedImage.valid())
        {
            return GeoImage( cachedImage.get(), key.getExtent() );
        }
        else
        {
            return GeoImage::INVALID;
        }
    }

    if (key.getProfile()->isHorizEquivalentTo(getProfile()))
    {
        TileKey best;

        if (getUpsample() == true &&
            getMaxDataLevel() > key.getLOD() &&
            (best = getBestAvailableTileKey(key)).valid() &&
            best != key)
        {
            result = createFractalUpsampledImage(key, progress);
        }
        else
        {
            result = createImageImplementation(key, progress);
        }
    }
    else
    {
        // If the profiles are different, use a compositing method to assemble the tile.
        result = assembleImage( key, progress );
    }

    // Check for cancelation before writing to a cache:
    if (progress && progress->isCanceled())
    {
        return GeoImage::INVALID;
    }

    if (result.valid())
    {
        // invoke user callbacks
        invoke_onCreate(key, result);

        if (_memCache.valid())
        {
            CacheBin* bin = _memCache->getOrCreateDefaultBin();
            bin->write(memCacheKey, result.getImage(), 0L);
        }

        // If we got a result, the cache is valid and we are caching in the map profile,
        // write to the map cache.
        if (cacheBin        &&
            policy.isCacheWriteable())
        {
            if ( key.getExtent() != result.getExtent() )
            {
                OE_INFO << LC << "WARNING! mismatched extents." << std::endl;
            }

            cacheBin->write(cacheKey, result.getImage(), 0L);
        }
    }

    else // result.valid() == false
    {
        OE_DEBUG << LC << key.str() << "result INVALID" << std::endl;
        // We couldn't get an image from the source.  So see if we have an expired cached image
        if (cachedImage.valid())
        {
            OE_DEBUG << LC << "Using cached but expired image for " << key.str() << std::endl;
            result = GeoImage( cachedImage.get(), key.getExtent());
        }
    }

    return result;
}

GeoImage
ImageLayer::assembleImage(
    const TileKey& key,
    ProgressCallback* progress)
{
    // If we got here, asset that there's a non-null layer profile.
    if (!getProfile())
    {
        setStatus(Status::Error(Status::AssertionFailure, "assembleImage with undefined profile"));
        return GeoImage::INVALID;
    }

    GeoImage mosaicedImage, result;

    // Scale the extent if necessary to apply an "edge buffer"
    GeoExtent ext = key.getExtent();
    if ( options().edgeBufferRatio().isSet() )
    {
        double ratio = options().edgeBufferRatio().get();
        ext.scale(ratio, ratio);
    }

    // Get a set of layer tiles that intersect the requested extent.
    std::vector<TileKey> intersectingKeys;
    getProfile()->getIntersectingTiles( key, intersectingKeys );

    if ( intersectingKeys.size() > 0 )
    {
#if 0
        GeoExtent ee = key.getExtent().transform(intersectingKeys.front().getProfile()->getSRS());
        OE_INFO << "Tile " << key.str() << " ... " << ee.toString() << std::endl;
        for (auto key : intersectingKeys) {
            OE_INFO << " - " << key.str() << " ... " << key.getExtent().toString() << std::endl;
        }
#endif
        double dst_minx, dst_miny, dst_maxx, dst_maxy;
        key.getExtent().getBounds(dst_minx, dst_miny, dst_maxx, dst_maxy);

        // if we find at least one "real" tile in the mosaic, then the whole result tile is
        // "real" (i.e. not a fallback tile)
        bool retry = false;
        ImageMosaic mosaic;

        // keep track of failed tiles.
        std::vector<TileKey> failedKeys;

        for(auto& k : intersectingKeys)
        {
            GeoImage image = createImageInKeyProfile(k, progress);

            if (image.valid())
            {
                if (dynamic_cast<const TimeSeriesImage*>(image.getImage()))
                {
                    OE_WARN << LC << "Cannot mosaic a TimeSeriesImage. Discarding." << std::endl;
                    return GeoImage::INVALID;
                }
                else if (dynamic_cast<const osg::ImageStream*>(image.getImage()))
                {
                    OE_WARN << LC << "Cannot mosaic an osg::ImageStream. Discarding." << std::endl;
                    return GeoImage::INVALID;
                }
                else
                {
                    mosaic.getImages().push_back(TileImage(image.getImage(), k));
                }
            }
            else
            {
                // the tile source did not return a tile, so make a note of it.
                failedKeys.push_back(k);

                if (progress && progress->isCanceled())
                {
                    retry = true;
                    break;
                }
            }
        }

        // Fail is: a) we got no data and the LOD is greater than zero; or
        // b) the operation was canceled mid-stream.
        if ( (mosaic.getImages().empty() && key.getLOD() > 0) || retry)
        {
            // if we didn't get any data at LOD>0, fail.
            OE_DEBUG << LC << "Couldn't create image for ImageMosaic " << std::endl;
            return GeoImage::INVALID;
        }

        // We got at least one good tile, OR we got nothing but since the LOD==0 we have to
        // fall back on a lower resolution.
        // So now we go through the failed keys and try to fall back on lower resolution data
        // to fill in the gaps. The entire mosaic must be populated or this qualifies as a bad tile.
        for(auto& k : failedKeys)
        {
            GeoImage image;

            for(TileKey parentKey = k.createParentKey();
                parentKey.valid() && !image.valid();
                parentKey.makeParent())
            {
                image = createImageImplementation( parentKey, progress );
                if ( image.valid() )
                {
                    GeoImage cropped;

                    if ( !isCoverage() )
                    {
                        cropped = image.crop(k.getExtent(), false, image.getImage()->s(), image.getImage()->t() );
                    }

                    else
                    {
                        // TODO: may not work.... test; tilekey extent will <> cropped extent
                        cropped = image.crop(k.getExtent(), true, image.getImage()->s(), image.getImage()->t(), false );
                    }

                    // and queue it.
                    mosaic.getImages().push_back( TileImage(cropped.getImage(), k) );

                }
            }

            if ( !image.valid() )
            {
                // a tile completely failed, even with fallback. Eject.
                OE_DEBUG << LC << "Couldn't fallback on tiles for ImageMosaic" << std::endl;
                // let it go. The empty areas will be filled with alpha by ImageMosaic.
            }
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
        OE_DEBUG << LC << "assembleImage: no intersections (" << key.str() << ")" << std::endl;
    }

    // Final step: transform the mosaic into the requesting key's extent.
    if ( mosaicedImage.valid() )
    {
        // GeoImage::reproject() will automatically crop the image to the correct extents.
        // so there is no need to crop after reprojection. Also note that if the SRS's are the
        // same (even though extents are different), then this operation is technically not a
        // reprojection but merely a resampling.

        const GeoExtent& extent = key.getExtent();

        result = mosaicedImage.reproject(
            key.getProfile()->getSRS(),
            &extent,
            getTileSize(), getTileSize(),
            true);
    }

    if (progress && progress->isCanceled())
    {
        return GeoImage::INVALID;
    }

    return result;
}

Status
ImageLayer::writeImage(const TileKey& key, const osg::Image* image, ProgressCallback* progress)
{
    if (getStatus().isError())
        return getStatus();

    return writeImageImplementation(key, image, progress);
}

Status
ImageLayer::writeImageImplementation(const TileKey& key, const osg::Image* image, ProgressCallback* progress) const
{
    return Status(Status::ServiceUnavailable);
}

const std::string
ImageLayer::getCompressionMethod() const
{
    if (isCoverage())
        return "none";

    return options().textureCompression().get();
}

void
ImageLayer::modifyTileBoundingBox(const TileKey& key, osg::BoundingBox& box) const
{
    if (options().altitude().isSet())
    {
        if (options().altitude()->as(Units::METERS) > box.zMax())
        {
            box.zMax() = options().altitude()->as(Units::METERS);
        }
    }
    TileLayer::modifyTileBoundingBox(key, box);
}

void
ImageLayer::addCallback(ImageLayer::Callback* c)
{
    _callbacks.lock();
    _callbacks.push_back(c);
    _callbacks.unlock();
}

void
ImageLayer::removeCallback(ImageLayer::Callback* c)
{
    _callbacks.lock();
    Callbacks::iterator i = std::find(_callbacks.begin(), _callbacks.end(), c);
    if (i != _callbacks.end())
        _callbacks.erase(i);
    _callbacks.unlock();
}

void
ImageLayer::addPostLayer(ImageLayer* layer)
{
    ScopedMutexLock lock(_postLayers);
    _postLayers.push_back(layer);
}

//...................................................................

#define ARENA_ASYNC_LAYER "oe.layer.async"
//#define FUTURE_IMAGE_COLOR_PLACEHOLDER

FutureTexture2D::FutureTexture2D(
    ImageLayer* layer,
    const TileKey& key) :

    osg::Texture2D(),
    FutureTexture(),
    _layer(layer),
    _key(key)
{
    // since we'll be updating it mid stream
    setDataVariance(osg::Object::DYNAMIC);

    setName(_key.str() + ":" + _layer->getName());

    // start loading the image
    dispatch();
}

void
FutureTexture2D::dispatch() const
{
    osg::observer_ptr<ImageLayer> layer_ptr(_layer);
    TileKey key(_key);

    Job job(JobArena::get(ARENA_ASYNC_LAYER));
    job.setName(Stringify() << key.str() << " " << _layer->getName());

    // prioritize higher LOD tiles.
    job.setPriority(key.getLOD());

    _result = job.dispatch<GeoImage>(
        [layer_ptr, key](Cancelable* progress) mutable
        {
            GeoImage result;
            osg::ref_ptr<ImageLayer> safe(layer_ptr);
            if (safe.valid())
            {
                osg::ref_ptr<ProgressCallback> p = new ProgressCallback(progress);
                result = safe->createImage(key, p.get());
            }
            return result;
        });
}

bool
FutureTexture2D::requiresUpdateCall() const
{
    // careful - if we return false here, it may never get called again.

    if (_resolved)
    {
        return false;
    }

    if (_result.isCanceled())
    {
        dispatch();
    }

    return true;
}


void
FutureTexture2D::update(osg::NodeVisitor* nv)
{
    if (_resolved == false && _result.isAvailable() == true)
    {
        OE_DEBUG<< LC << "Async result available for " << getName() << std::endl;

        // fetch the result
        GeoImage geoImage = _result.get();

        if (geoImage.getStatus().isError())
        {
            OE_WARN << LC << "Error: " << geoImage.getStatus().message() << std::endl;
            _failed = true;
        }
        else
        {
            osg::ref_ptr<osg::Image> image = geoImage.takeImage();

            if (image.valid())
            {
                this->setImage(image);
                this->dirtyTextureObject();
            }

            else
            {
                _failed = true;
                this->dirtyTextureObject();
            }
        }

        // reset the future so update won't be called again
        _result.abandon();

        _resolved = true;
    }
}

GeoImage
ImageLayer::createFractalUpsampledImage(
    const TileKey& key,
    ProgressCallback* progress)
{
    OE_PROFILING_ZONE;

    // Input metatile grid. Always use the immediate parent for
    // fractal enhancement.
    MetaTile<GeoImage> input;
    input.setCreateTileFunction(
        [&](const TileKey& key, ProgressCallback* p) -> GeoImage
        {
            if (this->isKeyInLegalRange(key))
                return this->createImage(key, p);
            else
                return GeoImage::INVALID;
        }
    );
    TileKey parentKey = key.createParentKey();
    osg::Matrix scale_bias;
    key.getExtent().createScaleBias(parentKey.getExtent(), scale_bias);
    input.setCenterTileKey(parentKey, scale_bias);

    // validate that we have a good metatile.
    if (input.valid() == false)
        return GeoImage::INVALID;

    // set up a workspace for creating the new image.
    int ws_width = getTileSize() + 3;
    int ws_height = getTileSize() + 3;

    osg::ref_ptr<osg::Image> workspace = new osg::Image();
    workspace->allocateImage(
        ws_width, ws_height, 1,
        input.getCenterTile().getImage()->getPixelFormat(),
        input.getCenterTile().getImage()->getDataType(),
        input.getCenterTile().getImage()->getPacking());

    ImageUtils::PixelWriter writeToWorkspace(workspace.get());
    ImageUtils::PixelReader readFromWorkspace(workspace.get());

    // output image:
    osg::ref_ptr<osg::Image> output = new osg::Image();
    output->allocateImage(
        getTileSize(), getTileSize(), 1,
        input.getCenterTile().getImage()->getPixelFormat(),
        input.getCenterTile().getImage()->getDataType(),
        input.getCenterTile().getImage()->getPacking());

    // Random number generator for fractal algorithm:
    Util::Random prng(key.hash());

    // temporaries:
    GeoImage::pixel_type pixel, p0, p1, p2, p3;
    float k0, k1, k2, k3;
    unsigned r;
    int s, t;

    // First pass: loop over the grid and populate even-numbered
    // pixels with values from the ancestors.
    for (t = 0; t < ws_height; t += 2)
    {
        for (s = 0; s < ws_width; s += 2)
        {
            input.read(pixel, s - 2, t - 2);
            writeToWorkspace(pixel, s, t);

            if (progress && progress->isCanceled())
                return GeoImage::INVALID;
        }

        if (progress && progress->isCanceled())
            return GeoImage::INVALID;
    }

    // Second pass: diamond
    for (t = 1; t < workspace->t() - 1; t += 2)
    {
        for (s = 1; s < workspace->s() - 1; s += 2)
        {
            r = prng.next(4u);

            // Diamond: pick one of the four diagonals to copy into the
            // center pixel, attempting to preserve curves. When there is
            // no clear choice, go random.
            readFromWorkspace(p0, s - 1, t - 1); k0 = p0.r();
            readFromWorkspace(p1, s + 1, t - 1); k1 = p1.r();
            readFromWorkspace(p2, s + 1, t + 1); k2 = p2.r();
            readFromWorkspace(p3, s - 1, t + 1); k3 = p3.r();

            // three the same
            if (k0 == k1 && k1 == k2 && k2 != k3) pixel = p0;
            else if (k1 == k2 && k2 == k3 && k3 != k0) pixel = p1;
            else if (k2 == k3 && k3 == k0 && k0 != k1) pixel = p2;
            else if (k3 == k0 && k0 == k1 && k1 != k2) pixel = p3;

            // continuations
            else if (k0 == k2 && k0 != k1 && k0 != k3) pixel = p0;
            else if (k1 == k3 && k1 != k2 && k1 != k0) pixel = p1;

            // all else, random.
            else pixel = (r == 0) ? p0 : (r == 1) ? p1 : (r == 2) ? p2 : p3;

            writeToWorkspace(pixel, s, t);
        }
    }

    // Third pass: square
    for (t = 2; t < workspace->t() - 1; ++t)
    {
        for (s = 2; s < workspace->s() - 1; ++s)
        {
            if (((s & 1) == 1 && (t & 1) == 0) || ((s & 1) == 0 && (t & 1) == 1))
            {
                r = prng.next(4u);

                // Square: pick one of the four adjacents to copy into the
                // center pixel, attempting to preserve curves. When there is
                // no clear choice, go random.
                readFromWorkspace(p0, s - 1, t); k0 = p0.r();
                readFromWorkspace(p1, s, t - 1); k1 = p1.r();
                readFromWorkspace(p2, s + 1, t); k2 = p2.r();
                readFromWorkspace(p3, s, t + 1); k3 = p3.r();

                // three the same
                if (k0 == k1 && k1 == k2 && k2 != k3) pixel = p0;
                else if (k1 == k2 && k2 == k3 && k3 != k0) pixel = p1;
                else if (k2 == k3 && k3 == k0 && k0 != k1) pixel = p2;
                else if (k3 == k0 && k0 == k1 && k1 != k2) pixel = p3;

                // continuations
                else if (k0 == k2 && k0 != k1 && k0 != k3) pixel = p0;
                else if (k1 == k3 && k1 != k2 && k1 != k0) pixel = p1;

                // all else, random.
                else pixel = (r == 0) ? p0 : (r == 1) ? p1 : (r == 2) ? p2 : p3;

                writeToWorkspace(pixel, s, t);
            }
        }
    }

    // finally blit from workspace interior to the output image.
    ImageUtils::PixelWriter writeToOutput(output.get());
    for (t = 0; t < output->t(); ++t)
    {
        for (s = 0; s < output->s(); ++s)
        {
            readFromWorkspace(pixel, s + 2, t + 2);
            writeToOutput(pixel, s, t);
        }
    }

    if (progress && progress->isCanceled())
    {
        return GeoImage::INVALID;
    }

    return GeoImage(output.get(), key.getExtent());
}
