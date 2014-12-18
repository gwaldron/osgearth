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
#include <osgEarth/TerrainTileModelFactory>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/ImageToHeightFieldConverter>

#define LC "[TerrainTileModelFactory] "

using namespace osgEarth;

//.........................................................................

namespace
{
    // Scale and bias matrices, one for each TileKey quadrant.
    const osg::Matrixf quadrantScaleBias[4] =
    {
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.0f,0.0f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.0f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.0f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.5f,0,1.0f)
    };

#if 0
    void createScaleBiasMatrix(unsigned quadrant, osg::Matrix& out)
    {
        float scale = 0.5f;
        float xbias = quadrant == 0 || quadrant == 3 ? 0.0f : 0.5f;
        float ybias = quadrant <= 1 ? 0.0f : 0.5f;
        out.preMultScale(osg::Vec3f(scale,scale,scale));
        out.preMultTranslate(osg::Vec3f(xbias,ybias,0.0f));
    }
#endif
}

//.........................................................................

TerrainTileModelFactory::TerrainTileModelFactory(const TerrainOptions& options) :
_options( options )
{
    // NOP
}

TerrainTileModel*
TerrainTileModelFactory::createTileModel(const MapFrame&              frame,
                                         const TileKey&               key,
                                         const TerrainTileModelStore* modelStore,
                                         ProgressCallback*            progress)
{
    // Make a new model:
    osg::ref_ptr<TerrainTileModel> model = new TerrainTileModel(
        key,
        frame.getRevision() );

    // assemble all the components:
    addImageLayers( model.get(), frame, key, modelStore, progress );
    addElevation  ( model.get(), frame, key, modelStore, progress );
    addNormalMap  ( model.get(), frame, key, modelStore, progress );

    // done.
    return model.release();
}

void
TerrainTileModelFactory::addImageLayers(TerrainTileModel*            model,
                                        const MapFrame&              frame,
                                        const TileKey&               key,
                                        const TerrainTileModelStore* modelStore,
                                        ProgressCallback*            progress)
{
    OE_START_TIMER(fetch_image_layers);

    for(ImageLayerVector::const_iterator i = frame.imageLayers().begin();
        i != frame.imageLayers().end();
        ++i )
    {
        ImageLayer* layer = i->get();

        if ( layer->getEnabled() && layer->isKeyInRange(key) )
        {
            // This will only go true if we are requesting a ROOT TILE but we have to
            // fall back on lower resolution data to create it.
            bool isFallback = false;

            GeoImage geoImage;

            const Profile* layerProfile = layer->getProfile();

            // The "fast path" preserves mercator tiles without reprojection.
            bool useMercatorFastPath =
                _options.enableMercatorFastPath() != false &&
                frame.getMapInfo().isGeocentric()          &&
                layerProfile                               &&
                layerProfile->getSRS()->isSphericalMercator();

            // If this is a ROOT tile, we will try to fall back on lower-resolution
            // data if we can't find something at the optimal LOD.
            bool isRootKey =
                (key.getLOD() == 0) || // should never be
                (key.getLOD()-1 == _options.firstLOD().get() );

            TileSource* tileSource = layer->getTileSource();

            // Only try to get data from the source if it actually intersects the key extent
            bool hasDataInExtent = true;
            if ( tileSource && layerProfile )
            {
                GeoExtent ext = key.getExtent();
                if (!layerProfile->getSRS()->isEquivalentTo( ext.getSRS() ))
                {
                    ext = layerProfile->clampAndTransformExtent( ext );
                }
                hasDataInExtent = tileSource->hasDataInExtent( ext );
            }
            
            // fetch the image from the layer if it's available:
            if ( hasDataInExtent && layer->isKeyInRange(key) )
            {
                // Ask the layer to produce an image tile.
                if ( useMercatorFastPath )
                    geoImage = layer->createImageInNativeProfile( key, progress );
                else
                    geoImage = layer->createImage( key, progress );

                // If the request failed, and this is a root tile, try to find
                // lower-resolution data to fulfill the request.
                if ( !geoImage.valid() && isRootKey )
                {
                    for(TileKey fallbackKey = key.createParentKey();
                        fallbackKey.valid() && !geoImage.valid();
                        fallbackKey = fallbackKey.createParentKey())
                    {
                        if ( useMercatorFastPath )
                            geoImage = layer->createImageInNativeProfile( fallbackKey, progress );
                        else
                            geoImage = layer->createImage( fallbackKey, progress );

                        if ( geoImage.valid() )
                        {
                            OE_DEBUG << LC << "Fell back from (" 
                                << key.str() << ") to ("
                                << fallbackKey.str() << ") for a root tile request."
                                << std::endl;
                        }
                    }
                }
            }
            
            TerrainTileLayerModel* layerModel = new TerrainTileLayerModel(
                TerrainTileModel::LAYER_IMAGE);

            if ( geoImage.valid() )
            {
                // made an image. Store as a texture with an identity matrix.
                osg::Texture* texture = createImageTexture(geoImage.getImage(), layer);
                layerModel->setTexture( texture );
                layerModel->setMatrix( 0L );
            }
            else if ( modelStore )
            {
                // no image; use the parent texture with a scale/bias matrix.
                osg::ref_ptr<const TerrainTileModel> parentModel;
                TileKey parentKey = key.createParentKey();
                if ( modelStore->get(parentKey, parentModel) )
                {
                    const TerrainTileLayerModel* parentLayerModel = parentModel->getLayer(layer->getUID());
                    if ( parentLayerModel )
                    {
                        layerModel->setTexture( parentLayerModel->getTexture() );
                        osg::RefMatrixf* parentMatrix = parentLayerModel->getMatrix();
                        unsigned quadrant = key.getQuadrant();
                        layerModel->setMatrix( new osg::RefMatrixf(
                            parentMatrix?
                                (*parentMatrix) * quadrantScaleBias[quadrant] :
                                quadrantScaleBias[quadrant] ) );
                    }
                }
            }

#if 0 // TODO: figure this out in the engine
            if ( geoImage.valid() )
            {
                // TODO: the engine derive the Locator; it shouldn't be in the model.
                if ( useMercatorFastPath )
                    layerModel->setLocator( new MercatorLocator(geoImage.getExtent()) );
                else
                    layerModel->setLocator( GeoLocator::createForExtent(geoImage.getExtent(), frame.getMapInfo()) );
            }
#endif

            model->layers().push_back( layerModel );
        }
    }

    if (progress)
        progress->stats()["fetch_imagery_time"] += OE_STOP_TIMER(fetch_image_layers);
}


void
TerrainTileModelFactory::addElevation(TerrainTileModel*            model,
                                      const MapFrame&              frame,
                                      const TileKey&               key,
                                      const TerrainTileModelStore* modelStore,
                                      ProgressCallback*            progress)
{    
    // make an elevation layer.
    OE_START_TIMER(fetch_elevation);

    const MapInfo& mapInfo = frame.getMapInfo();

    const osgEarth::ElevationInterpolation& interp =
        frame.getMapOptions().elevationInterpolation().get();

    // Request a heightfield from the map.
    osg::ref_ptr<osg::HeightField> mainHF;

    TerrainTileLayerModel* layerModel = new TerrainTileLayerModel(
        TerrainTileModel::LAYER_ELEVATION);

    // Make a new heightfield. Use the cache as necessary.
    osg::Image* image = 0L;
    if (getOrCreateHeightField(frame, key, SAMPLE_FIRST_VALID, interp, mainHF, progress))
    {
        model->heightFields().setNeighbor(0, 0, mainHF.get());

        // convert the heightfield to a 1-channel 32-bit fp image:
        ImageToHeightFieldConverter conv;
        image = conv.convert( mainHF.get(), 32 ); // 32 = GL_FLOAT
        if ( image )
        {
            // if we have access to the model store, use the parent's data to
            // fill in any missing values:
            if ( modelStore )
            {
                osg::ref_ptr<const TerrainTileModel> parentModel;
                TileKey parentKey = key.createParentKey();
                if ( modelStore->get(parentKey, parentModel) )
                {
                    const TerrainTileLayerModel* parentElevLayer = parentModel->getLayer(
                        TerrainTileModel::LAYER_ELEVATION );

                    if (parentElevLayer && parentElevLayer->getTexture())
                    {
                        ImageUtils::replaceNoDataValues(
                            image,
                            key.getExtent().bounds(),
                            parentElevLayer->getTexture()->getImage(0),
                            parentKey.getExtent().bounds() );
                    }
                }
            }
        }
    }

    if ( image )
    {
        // Made an image, so store this as a texture with no matrix.
        osg::Texture* texture = createElevationTexture( image );
        layerModel->setTexture( texture );
        layerModel->setMatrix( 0L );
    }
    else if ( modelStore )
    {
        // no image; use the parent texture with a scale/bias matrix.
        osg::ref_ptr<const TerrainTileModel> parentModel;
        TileKey parentKey = key.createParentKey();
        if ( modelStore->get(parentKey, parentModel) )
        {
            const TerrainTileLayerModel* parentLayerModel = parentModel->getLayer(
                TerrainTileModel::LAYER_ELEVATION );

            if ( parentLayerModel )
            {
                layerModel->setTexture( parentLayerModel->getTexture() );
                osg::RefMatrixf* parentMatrix = parentLayerModel->getMatrix();
                unsigned quadrant = key.getQuadrant();
                layerModel->setMatrix( new osg::RefMatrixf(
                    parentMatrix?
                        (*parentMatrix) * quadrantScaleBias[quadrant] :
                        quadrantScaleBias[quadrant] ) );
            }
        }
    }

    model->layers().push_back( layerModel );

    if (progress)
        progress->stats()["fetch_elevation_time"] += OE_STOP_TIMER(fetch_elevation);
}

void
TerrainTileModelFactory::addNormalMap(TerrainTileModel*            model,
                                      const MapFrame&              frame,
                                      const TileKey&               key,
                                      const TerrainTileModelStore* modelStore,
                                      ProgressCallback*            progress)
{
    OE_START_TIMER(fetch_normalmap);

    const osgEarth::ElevationInterpolation& interp =
        frame.getMapOptions().elevationInterpolation().get();

    TerrainTileLayerModel* layerModel = new TerrainTileLayerModel(
        TerrainTileModel::LAYER_NORMAL);

    // Can only generate the normal map if the center heightfield was built:
    osg::Image* image = 0L;

    if ( model->heightFields().getNeighbor(0, 0) != 0L )
    {
        // assemble the neighboring heightfields so we can get the edges correct:
        for(int x=-1; x<=1; x+=2)
        {
            for(int y=-1; y<=1; y+=2)
            {
                osg::ref_ptr<osg::HeightField> neighborHF;
                bool isFallback = false;
                TileKey neighborKey = key.createNeighborKey(x, y);
                if (getOrCreateHeightField(frame, neighborKey, SAMPLE_FIRST_VALID, interp, neighborHF, progress))
                {
                    model->heightFields().setNeighbor(x, y, neighborHF.get());
                }
            }
        }

        // Create an image encoding normals and curvature:
        image = HeightFieldUtils::convertToNormalMap(
            model->heightFields(),
            key.getProfile()->getSRS() );
    }

    if ( image )
    {
        // Made an image, so store this as a texture with no matrix.
        osg::Texture* texture = createNormalTexture( image );
        layerModel->setTexture( texture );
        layerModel->setMatrix( 0L );
    }
    else if ( modelStore )
    {
        // no image; use the parent texture with a scale/bias matrix.
        osg::ref_ptr<const TerrainTileModel> parentModel;
        TileKey parentKey = key.createParentKey();
        if ( modelStore->get(parentKey, parentModel) )
        {
            const TerrainTileLayerModel* parentLayerModel = parentModel->getLayer(
                TerrainTileModel::LAYER_NORMAL );

            if ( parentLayerModel )
            {
                layerModel->setTexture( parentLayerModel->getTexture() );
                osg::RefMatrixf* parentMatrix = parentLayerModel->getMatrix();
                unsigned quadrant = key.getQuadrant();
                layerModel->setMatrix( new osg::RefMatrixf(
                    parentMatrix?
                        (*parentMatrix) * quadrantScaleBias[quadrant] :
                        quadrantScaleBias[quadrant] ) );
            }
        }
    }

    model->layers().push_back( layerModel );


    if (progress)
        progress->stats()["fetch_normalmap_time"] += OE_STOP_TIMER(fetch_normalmap);
}

bool
TerrainTileModelFactory::getOrCreateHeightField(const MapFrame&                 frame,
                                                const TileKey&                  key,
                                                ElevationSamplePolicy           samplePolicy,
                                                ElevationInterpolation          interpolation,
                                                osg::ref_ptr<osg::HeightField>& out_hf,
                                                ProgressCallback*               progress)
{
    // check the quick cache.
    HFCacheKey cachekey;
    cachekey._key          = key;
    cachekey._revision     = frame.getRevision();
    cachekey._samplePolicy = samplePolicy;

    if (progress)
        progress->stats()["hfcache_try_count"] += 1;

    bool hit = false;
    HFCache::Record rec;
    if ( _heightFieldCache.get(cachekey, rec) )
    {
        out_hf = rec.value().get();

        if (progress)
        {
            progress->stats()["hfcache_hit_count"] += 1;
            progress->stats()["hfcache_hit_rate"] = progress->stats()["hfcache_hit_count"]/progress->stats()["hfcache_try_count"];
        }

        return true;
    }

    bool populated = frame.populateHeightField(
        out_hf,
        key,
        true, // convertToHAE
        samplePolicy,
        progress );

    // Treat Plate Carre specially by scaling the height values. (There is no need
    // to do this with an empty heightfield)
    const MapInfo& mapInfo = frame.getMapInfo();
    if ( mapInfo.isPlateCarre() )
    {
        HeightFieldUtils::scaleHeightFieldToDegrees( out_hf.get() );
    }

    // cache it.
    _heightFieldCache.insert( cachekey, out_hf.get() );
    return true;
}

osg::Texture*
TerrainTileModelFactory::createImageTexture(osg::Image*       image,
                                            const ImageLayer* layer) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );

    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint(false);

    osg::Texture::FilterMode magFilter = 
        layer ? layer->getImageLayerOptions().minFilter().get() : osg::Texture::LINEAR;
    osg::Texture::FilterMode minFilter =
        layer ? layer->getImageLayerOptions().minFilter().get() : osg::Texture::LINEAR;

    tex->setFilter( osg::Texture::MAG_FILTER, magFilter );
    tex->setFilter( osg::Texture::MIN_FILTER, minFilter );
    tex->setMaxAnisotropy( 4.0f );

    // Disable mip mapping for npot tiles
    if (!ImageUtils::isPowerOfTwo( image ) || (!image->isMipmap() && ImageUtils::isCompressed(image)))
    {
        tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    }    

    const optional<bool>& unRefPolicy = Registry::instance()->unRefImageDataAfterApply();
    if ( unRefPolicy.isSet() )
        tex->setUnRefImageDataAfterApply( unRefPolicy.get() );

    return tex;
}

osg::Texture*
TerrainTileModelFactory::createElevationTexture(osg::Image* image) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );
    tex->setInternalFormat(GL_LUMINANCE32F_ARB);
    tex->setSourceFormat(GL_LUMINANCE);
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    tex->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint( false );
    tex->setMaxAnisotropy( 1.0f );
    return tex;
}

osg::Texture*
TerrainTileModelFactory::createNormalTexture(osg::Image* image) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );
    tex->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    tex->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint( false );
    tex->setMaxAnisotropy( 1.0f );
    return tex;
}
