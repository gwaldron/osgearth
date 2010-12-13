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
    _prevGamma = 1.0f;

    // intialize the runtime actuals from the initialization options:
    _actualOpacity = _options.opacity().value();
    _actualGamma   = _options.gamma().value();

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
ImageLayer::initTileSource()
{
    // call superclass first.
    TerrainLayer::initTileSource();

    if ( _tileSource.valid() && _tileSource->isOK() )
    {
        if ( _options.noDataImageFilename().isSet() && !_options.noDataImageFilename()->empty() )
        {
            OE_INFO << "Setting nodata image to \"" << _options.noDataImageFilename().value() << "\"" << std::endl;
            _noDataImage = osgDB::readImageFile( _options.noDataImageFilename().value() );
            if ( !_noDataImage.valid() )
            {
                OE_WARN << "Warning: Could not read nodata image from \"" << _options.noDataImageFilename().value() << "\"" << std::endl;
            }
        }
    }
}

void
ImageLayer::postProcess( GeoImage& input )
{
    //TODO: dump this and move it to the TextureCompositor (ideally to a shader).
    //TODO: this is not thread-safe; more than one thread might initialize the gamma LUT at the
    //      same time

#if 0
    if ( input.valid() && _actualGamma != 1.0f )
    {
        double g = _actualGamma;
        if ( g != _prevGamma )
        {
            double gc = g > 0.0 ? 1.0/g : 1.0/5.0;
            OE_DEBUG << LC << "Layer \"" << getName() << "\", building gamma LUT, gamma = " << g << std::endl;
            for(unsigned int i = 0; i < 256; i++)
            {
                _gammaLUT[i] = (unsigned char)(pow(double(i) / 255.0, gc) * 255.0);
            }            
            _prevGamma = g;
        }

        osg::Image* im = input.getImage();
        unsigned int p = input.getImage()->getTotalSizeInBytes();
        unsigned char* d = input.getImage()->data();
        if ( im->getPixelFormat() == GL_RGBA )
        {
            for(unsigned int i=0; i<p; i+=4, d+=4)
            {
                *d = _gammaLUT[*d];
                *(d+1) = _gammaLUT[*(d+1)];
                *(d+2) = _gammaLUT[*(d+2)];
            }
        }
        else if ( im->getPixelFormat() == GL_RGB )
        {
            for(unsigned int i=0; i<p; ++i, ++d)
            {
                *d = _gammaLUT[*d];
            }
        }
        else
        {
            OE_DEBUG << LC << "Gamma not applied (image not RGB or RGBA)" << std::endl;
        }
    }
#endif
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
        //_cache->storeLayerProperties( getName(), _cacheProfile, _actualCacheFormat, _tileSource->getPixelsPerTile() );
    }

	if (cacheInMapProfile)
	{
		OE_DEBUG << LC << "Layer \"" << getName() << "\" caching in Map profile " << std::endl;
	}

	//If we are caching in the map profile, try to get the image immediately.
    if (cacheInMapProfile && _cache.valid() && _options.cacheEnabled() == true )
	{
        osg::ref_ptr<osg::Image> image;
        if ( _cache->getImage( key, _cacheSpec, image ) )
		{
			OE_DEBUG << LC << "Layer \"" << getName()<< "\" got tile " << key.str() << " from map cache " << std::endl;
            result = GeoImage( image.get(), key.getExtent() );
            postProcess( result );
            return result;
		}
	}

	//If the key profile and the source profile exactly match, simply request the image from the source
    if ( mapProfile->isEquivalentTo( layerProfile ) )
    {
		OE_DEBUG << "Key and source profiles are equivalent, requesting single tile" << std::endl;
        osg::ref_ptr<osg::Image> image;
        if ( createImageWrapper( key, cacheInLayerProfile, image, progress ) )
        {
            result = GeoImage( image.get(), key.getExtent() );
        }
    }
    // Otherwise, we need to process the tiles.
    else
    {
		OE_DEBUG << "Key and source profiles are different, creating mosaic" << std::endl;
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
                if ( createImageWrapper(intersectingTiles[j], cacheInLayerProfile, img, progress) )
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
                osg::ref_ptr<osg::Image> validImage = mi->getImages()[0].getImage();
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

	//If we got a result, the cache is valid and we are caching in the map profile, write to the map cache.
    if (result.valid() && _cache.valid() && _options.cacheEnabled() == true && cacheInMapProfile)
	{
		OE_DEBUG << LC << "Layer \"" << getName() << "\" writing tile " << key.str() << " to cache " << std::endl;
		_cache->setImage( key, _cacheSpec, result.getImage());
	}

    postProcess( result );
    return result;
}

bool
ImageLayer::createImageWrapper(const TileKey& key,
                               bool cacheInLayerProfile,
                               osg::ref_ptr<osg::Image>& out_image,
                               ProgressCallback* progress )
{
	osg::ref_ptr<osg::Image> image;

    if (_cache.valid() && cacheInLayerProfile && _options.cacheEnabled() == true )
    {
		if ( _cache->getImage( key, _cacheSpec, out_image ) )
	    {
            OE_DEBUG << LC << " Layer \"" << getName() << "\" got " << key.str() << " from cache " << std::endl;
            return true;
    	}
    }
    
    //TileSource* source = 0L;

    //if ( _actualCacheOnly == false )
    //    source = getTileSource();

	if ( !_actualCacheOnly )
	{
        TileSource* source = getTileSource();
        if ( !source )
            return false;
        
        // just in case the output image had data in it...unlikely
        if ( out_image.valid() )
            out_image = 0L;

        //Only try to get the image if it's not in the blacklist
        if (!source->getBlacklist()->contains( key.getTileId() ))
        {
            //Only try to get data if the source actually has data
            if (source->hasData( key ) )
            {
                source->getImage( key, image, progress );

                //If the image is not valid and the progress was not cancelled, blacklist
                if (!image.valid() && (!progress || !progress->isCanceled()))
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

		//Check to see if the image is the nodata image
		if (image.valid() && _noDataImage.valid() )
		{
			if (ImageUtils::areEquivalent(image.get(), _noDataImage.get()))
			{
				OE_DEBUG << LC << "Found nodata for " << key.str() << std::endl;
				image = 0L;
			}
		}

		//Apply a transparent color mask if one is specified
        if (image.valid() && _options.transparentColor().isSet() )
		{
            const osg::Vec4ub& chroma = _options.transparentColor().get();

			for (unsigned int row = 0; row < image->t(); ++row)
			{
				for (unsigned int col = 0; col < image->s(); ++col)
				{
					unsigned char r = image->data(col, row)[0];
					unsigned char g = image->data(col, row)[1];
					unsigned char b = image->data(col, row)[2];

					if (r == chroma.r() &&
						g == chroma.g() &&
						b == chroma.b())
					{
						image->data(col,row)[3] = 0;
					}
				}
			}
		}

        if (image.valid() && _cache.valid() && cacheInLayerProfile && _options.cacheEnabled() == true )
		{
			_cache->setImage( key, _cacheSpec, image.get() );
		}
	}

    out_image = image.get();
    return out_image.valid();
}
