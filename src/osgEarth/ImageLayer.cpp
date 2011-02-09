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
#include <osgEarth/ImageLayer>
#include <osgEarth/TileSource>
#include <osgEarth/ImageMosaic>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osg/Version>
#include <memory.h>
#include <limits.h>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[ImageLayer] "

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
    _gamma.init( 1.0f );
    _transparentColor.init( osg::Vec4ub(0,0,0,0) );
    _minRange.init( -FLT_MAX );
    _maxRange.init( FLT_MAX );
    _compressTextures.init( false );
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
    conf.getIfSet( "gamma", _gamma );
    conf.getIfSet( "min_range", _minRange );
    conf.getIfSet( "max_range", _maxRange );
    conf.getIfSet( "compress_textures", _compressTextures);

    if ( conf.hasValue( "transparent_color" ) )
        _transparentColor = stringToColor( conf.value( "transparent_color" ), osg::Vec4ub(0,0,0,0));

	//Load the filter settings
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
}

Config
ImageLayerOptions::getConfig() const
{
    Config conf = TerrainLayerOptions::getConfig();
    conf.updateIfSet( "nodata_image", _noDataImageFilename );
    conf.updateIfSet( "opacity", _opacity );
    conf.updateIfSet( "gamma", _gamma );
    conf.updateIfSet( "min_range", _minRange );
    conf.updateIfSet( "max_range", _maxRange );
    conf.updateIfSet( "compress_textures", _compressTextures);

	if (_transparentColor.isSet())
        conf.update("transparent_color", colorToString( _transparentColor.value()));

    //Save the filter settings
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
}

//------------------------------------------------------------------------

ImageLayerTileProcessor::ImageLayerTileProcessor( const ImageLayerOptions& options )
{
    init( options, false );
}

void
ImageLayerTileProcessor::init( const ImageLayerOptions& options, bool layerInTargetProfile )
{
    _options = options;
    _layerInTargetProfile = layerInTargetProfile;

    if ( _layerInTargetProfile )
        OE_DEBUG << LC << "Good, the layer and map have the same profile." << std::endl;

    const osg::Vec4ub& ck= *_options.transparentColor();
    _chromaKey.set( ck.r() / 255.0f, ck.g() / 255.0f, ck.b() / 255.0f, 1.0 );

    if ( _options.noDataImageFilename().isSet() && !_options.noDataImageFilename()->empty() )
    {
        //OE_INFO << "Setting nodata image to \"" << _options.noDataImageFilename().value() << "\"" << std::endl;
        _noDataImage = osgDB::readImageFile( _options.noDataImageFilename().value() );
        if ( !_noDataImage.valid() )
        {
            OE_WARN << "Warning: Could not read nodata image from \"" << _options.noDataImageFilename().value() << "\"" << std::endl;
        }
    }
}

struct ApplyChromaKey
{
    osg::Vec4f _chromaKey;
    bool operator()( osg::Vec4f& pixel ) {
        bool equiv = ImageUtils::areRGBEquivalent( pixel, _chromaKey );
        if ( equiv ) pixel.a() = 0.0f;
        return equiv;
    }
};

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
TerrainLayer(),
_options( options )
{
    init();
}

ImageLayer::ImageLayer( const std::string& name, const TileSourceOptions& driverOptions ) :
TerrainLayer(),
_options( ImageLayerOptions(name, driverOptions) )
{
    init();
}

ImageLayer::ImageLayer( const ImageLayerOptions& options, TileSource* tileSource ) :
TerrainLayer( tileSource ),
_options( options )
{
    init();
}

void
ImageLayer::init()
{
//    _prevGamma = 1.0f;

    // intialize the runtime actuals from the initialization options:
    _actualOpacity = _options.opacity().value();
    //_actualGamma   = _options.gamma().value();

    //TODO: probably should graduate this to the superclass.
    _actualEnabled = _options.enabled().value();
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
    _actualOpacity = osg::clampBetween( value, 0.0f, 1.0f );
    fireCallback( &ImageLayerCallback::onOpacityChanged );
}

void
ImageLayer::setGamma( float value )
{
    _actualGamma = value;
    fireCallback( &ImageLayerCallback::onGammaChanged );
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
        getProfile() &&
        _targetProfileHint->isEquivalentTo( getProfile() );

    ImageLayerPreCacheOperation* op = new ImageLayerPreCacheOperation();    
    op->_processor.init( _options, layerInTargetProfile );

    _preCacheOp = op;
}

GeoImage
ImageLayer::createImage( const TileKey& key, ProgressCallback* progress)
{
    GeoImage result;

    const Profile* layerProfile = getProfile();
    const Profile* mapProfile = key.getProfile();

	if ( !layerProfile )
	{
		OE_WARN << LC << "Could not get a valid profile for Layer \"" << getName() << "\"" << std::endl;
        return GeoImage::INVALID;
	}

	//OE_NOTICE << "[osgEarth::MapLayer::createImage] " << key.str() << std::endl;
	if ( !_actualCacheOnly && !getTileSource()  )
	{
		OE_WARN << LC << "Error:  MapLayer does not have a valid TileSource, cannot create image " << std::endl;
		return GeoImage::INVALID;
	}

	//Determine whether we should cache in the Map profile or the Layer profile.

	bool cacheInMapProfile = true;
	if (mapProfile->isEquivalentTo( layerProfile ))
	{
		OE_DEBUG << LC << "Layer \"" << getName() << "\": Map and Layer profiles are equivalent " << std::endl;
	}
	//If the map profile and layer profile are in the same SRS but with different tiling scemes and exact cropping is not required, cache in the layer profile.
    else if (mapProfile->getSRS()->isEquivalentTo( layerProfile->getSRS()) && _options.exactCropping() == false )
	{
		OE_DEBUG << LC << "Layer \"" << getName() << "\": Map and Layer profiles are in the same SRS and non-exact cropping is allowed, caching in layer profile." << std::endl;
		cacheInMapProfile = false;
	}

	bool cacheInLayerProfile = !cacheInMapProfile;

    //Write the cache TMS file if it hasn't been written yet.
    if (!_cacheProfile.valid() && _cache.valid() && _options.cacheEnabled() == true && _tileSource.valid())
    {
        _cacheProfile = cacheInMapProfile ? mapProfile : _profile.get();
        _cache->storeProperties( _cacheSpec, _cacheProfile, _tileSource->getPixelsPerTile() );
    }

	if (cacheInMapProfile)
	{
		OE_DEBUG << LC << "Layer \"" << getName() << "\" caching in Map profile " << std::endl;
	}

	//If we are caching in the map profile, try to get the image immediately.
    if (cacheInMapProfile && _cache.valid() && _options.cacheEnabled() == true )
	{
        osg::ref_ptr<const osg::Image> cachedImage;
        if ( _cache->getImage( key, _cacheSpec, cachedImage ) )
		{
			OE_DEBUG << LC << "Layer \"" << getName()<< "\" got tile " << key.str() << " from map cache " << std::endl;

            result = GeoImage( ImageUtils::cloneImage(cachedImage.get()), key.getExtent() );
            ImageUtils::normalizeImage( result.getImage() );
            if (*_options.compressTextures())
            {
                result.compress();
            }
            return result;
		}
	}

	//If the key profile and the source profile exactly match, simply request the image from the source
    if ( mapProfile->isEquivalentTo( layerProfile ) )
    {
		OE_DEBUG << LC << "Key and source profiles are equivalent, requesting single tile" << std::endl;
        osg::ref_ptr<const osg::Image> image;
        osg::Image* im = createImageWrapper( key, cacheInLayerProfile, progress );
        if ( im )
        {
            result = GeoImage( im, key.getExtent() );
        }
    }

    // Otherwise, we need to process the tiles.
    else
    {
		OE_DEBUG << LC << "Key and source profiles are different, creating mosaic" << std::endl;
		GeoImage mosaic;

		// Determine the intersecting keys and create and extract an appropriate image from the tiles
		std::vector<TileKey> intersectingTiles;

        //Scale the extent if necessary
        GeoExtent ext = key.getExtent();
        if ( _options.edgeBufferRatio().isSet() )
        {
            double ratio = _options.edgeBufferRatio().get();
            ext.scale(ratio, ratio);
        }

        layerProfile->getIntersectingTiles(ext, intersectingTiles);

		if (intersectingTiles.size() > 0)
		{
			double dst_minx, dst_miny, dst_maxx, dst_maxy;
			key.getExtent().getBounds(dst_minx, dst_miny, dst_maxx, dst_maxy);

			osg::ref_ptr<ImageMosaic> mi = new ImageMosaic;
			std::vector<TileKey> missingTiles;

			for (unsigned int j = 0; j < intersectingTiles.size(); ++j)
			{
				double minX, minY, maxX, maxY;
				intersectingTiles[j].getExtent().getBounds(minX, minY, maxX, maxY);

				OE_DEBUG << LC << "\t Intersecting Tile " << j << ": " << minX << ", " << minY << ", " << maxX << ", " << maxY << std::endl;

				osg::ref_ptr<osg::Image> img;
                img = createImageWrapper( intersectingTiles[j], cacheInLayerProfile, progress );

                if ( img.valid() )
                {
                    if (img->getPixelFormat() != GL_RGBA || img->getDataType() != GL_UNSIGNED_BYTE || img->getInternalTextureFormat() != GL_RGBA8 )
					{
                        osg::ref_ptr<osg::Image> convertedImg = ImageUtils::convertToRGBA8(img.get());
                        if (convertedImg.valid())
                        {
                            img = convertedImg;
                        }
					}
					mi->getImages().push_back(TileImage(img.get(), intersectingTiles[j]));
				}
				else
				{
					missingTiles.push_back(intersectingTiles[j]);
				}
			}

			if (mi->getImages().empty())
			{
				OE_DEBUG << LC << "Couldn't create image for ImageMosaic " << std::endl;
                return GeoImage::INVALID;
			}
			else if (missingTiles.size() > 0)
			{
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

                    mi->getImages().push_back(TileImage(newImage.get(), missingTiles[j]));
                }
			}

			double rxmin, rymin, rxmax, rymax;
			mi->getExtents( rxmin, rymin, rxmax, rymax );

			mosaic = GeoImage(
				mi->createImage(),
				GeoExtent( layerProfile->getSRS(), rxmin, rymin, rxmax, rymax ) );
		}

		if ( mosaic.valid() )
        {
            // the imagery must be reprojected iff:
            //  * the SRS of the image is different from the SRS of the key;
            //  * UNLESS they are both geographic SRS's (in which case we can skip reprojection)
            bool needsReprojection =
                !mosaic.getSRS()->isEquivalentTo( key.getProfile()->getSRS()) &&
                !(mosaic.getSRS()->isGeographic() && key.getProfile()->getSRS()->isGeographic());

            bool needsLeftBorder = false;
            bool needsRightBorder = false;
            bool needsTopBorder = false;
            bool needsBottomBorder = false;

            // If we don't need to reproject the data, we had to mosaic the data, so check to see if we need to add
            // an extra, transparent pixel on the sides because the data doesn't encompass the entire map.
            if (!needsReprojection)
            {
                GeoExtent keyExtent = key.getExtent();
                // If the key is geographic and the mosaic is mercator, we need to get the mercator
                // extents to determine if we need to add the border or not
                // (TODO: this might be OBE due to the elimination of the Mercator fast-path -gw)
                if (key.getExtent().getSRS()->isGeographic() && mosaic.getSRS()->isMercator())
                {
                    keyExtent = osgEarth::Registry::instance()->getGlobalMercatorProfile()->clampAndTransformExtent( 
                        key.getExtent( ));
                }


                //Use an epsilon to only add the border if it is significant enough.
                double eps = 1e-6;
                
                double leftDiff = mosaic.getExtent().xMin() - keyExtent.xMin();
                if (leftDiff > eps)
                {
                    needsLeftBorder = true;
                }

                double rightDiff = keyExtent.xMax() - mosaic.getExtent().xMax();
                if (rightDiff > eps)
                {
                    needsRightBorder = true;
                }

                double bottomDiff = mosaic.getExtent().yMin() - keyExtent.yMin();
                if (bottomDiff > eps)
                {
                    needsBottomBorder = true;
                }

                double topDiff = keyExtent.yMax() - mosaic.getExtent().yMax();
                if (topDiff > eps)
                {
                    needsTopBorder = true;
                }
            }

            if ( needsReprojection )
            {
				OE_DEBUG << LC << "  Reprojecting image" << std::endl;

                // We actually need to reproject the image.  Note: GeoImage::reproject() will automatically
                // crop the image to the correct extents, so there is no need to crop after reprojection.
                result = mosaic.reproject( 
                    key.getProfile()->getSRS(),
                    &key.getExtent(), 
                    _options.reprojectedTileSize().value(), _options.reprojectedTileSize().value() );
            }
            else
            {
				OE_DEBUG << LC << "  Cropping image" << std::endl;
                // crop to fit the map key extents
                GeoExtent clampedMapExt = layerProfile->clampAndTransformExtent( key.getExtent() );
                if ( clampedMapExt.isValid() )
				{
                    int size = _options.exactCropping() == true ? _options.reprojectedTileSize().value() : 0;
                    result = mosaic.crop(clampedMapExt, _options.exactCropping().value(), size, size);
				}
                else
                    result = GeoImage::INVALID;
            }

            //Add the transparent pixel AFTER the crop so that it doesn't get cropped out
            if (result.valid() && (needsLeftBorder || needsRightBorder || needsBottomBorder || needsTopBorder))
            {
                result = result.addTransparentBorder(needsLeftBorder, needsRightBorder, needsBottomBorder, needsTopBorder);
            }
        }
    }

    // Normalize the image if necessary
    if ( result.valid() )
    {
        ImageUtils::normalizeImage( result.getImage() );
    }

	//If we got a result, the cache is valid and we are caching in the map profile, write to the map cache.
    if (result.valid() && _cache.valid() && _options.cacheEnabled() == true && cacheInMapProfile)
	{
		OE_DEBUG << LC << "Layer \"" << getName() << "\" writing tile " << key.str() << " to cache " << std::endl;
		_cache->setImage( key, _cacheSpec, result.getImage());
	}
    if (*_options.compressTextures())
    {
        result.compress();
    }
    return result;
}

osg::Image*
ImageLayer::createImageWrapper(const TileKey& key,
                               bool cacheInLayerProfile,
                               ProgressCallback* progress )
{
    osg::Image* result = 0L;

    if (_cache.valid() && cacheInLayerProfile && _options.cacheEnabled() == true )
    {
        osg::ref_ptr<const osg::Image> cachedImage;
		if ( _cache->getImage( key, _cacheSpec, cachedImage ) )
	    {
            OE_DEBUG << LC << " Layer \"" << getName() << "\" got " << key.str() << " from cache " << std::endl;
            return ImageUtils::cloneImage(cachedImage.get());
    	}
    }

	if ( !_actualCacheOnly )
	{
        TileSource* source = getTileSource();
        if ( !source )
            return false;

        //Only try to get the image if it's not in the blacklist
        if (!source->getBlacklist()->contains( key.getTileId() ))
        {
            //Only try to get data if the source actually has data
            if (source->hasData( key ) )
            {
                result = source->createImage( key, _preCacheOp.get(), progress );

                //If the image is not valid and the progress was not cancelled, blacklist
                if ( result == 0L && (!progress || !progress->isCanceled()))
                {
                    //Add the tile to the blacklist
                    OE_DEBUG << LC << "Adding tile " << key.str() << " to the blacklist" << std::endl;
                    source->getBlacklist()->add(key.getTileId());
                }
            }
            else
            {
                OE_DEBUG << LC << "Source has no data at " << key.str() << std::endl;
            }
        }
        else
        {
            OE_DEBUG << LC << "Tile " << key.str() << " is blacklisted, not checking" << std::endl;
        }

        // Cache is necessary:
        if ( result && _cache.valid() && cacheInLayerProfile && _options.cacheEnabled() == true )
		{
			_cache->setImage( key, _cacheSpec, result );
		}
	}

    return result;
}
