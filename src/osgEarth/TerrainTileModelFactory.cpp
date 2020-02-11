/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Map>
#include <osgEarth/Registry>
#include <osgEarth/LandCoverLayer>
#include <osgEarth/Metrics>

#include <osg/Texture2D>

#define LC "[TerrainTileModelFactory] "

using namespace osgEarth;

//.........................................................................

TerrainTileModelFactory::TerrainTileModelFactory(const TerrainOptions& options) :
_options         ( options ),
_heightFieldCache( true, 128 )
{
    _heightFieldCacheEnabled = (::getenv("OSGEARTH_MEMORY_PROFILE") == 0L);

    // Create an empty texture that we can use as a placeholder
    _emptyTexture = new osg::Texture2D(ImageUtils::createEmptyImage());
    _emptyTexture->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
}

TerrainTileModel*
TerrainTileModelFactory::createTileModel(const Map*                       map,
                                         const TileKey&                   key,
                                         const CreateTileModelFilter&     filter,
                                         const TerrainEngineRequirements* requirements,
                                         ProgressCallback*                progress)
{
    OE_PROFILING_ZONE;
    // Make a new model:
    osg::ref_ptr<TerrainTileModel> model = new TerrainTileModel(
        key,
        map->getDataModelRevision() );

    // assemble all the components:
    addColorLayers(model.get(), map, requirements, key, filter, progress, false);

    if ( requirements == 0L || requirements->elevationTexturesRequired() )
    {
        unsigned border = (requirements && requirements->elevationBorderRequired()) ? 1u : 0u;

        addElevation( model.get(), map, key, filter, border, progress );
    }

    addLandCover(model.get(), map, key, filter, progress);

    //addPatchLayers(model.get(), map, key, filter, progress, false);

    // done.
    return model.release();
    return model.release();
}

TerrainTileModel*
TerrainTileModelFactory::createStandaloneTileModel(const Map*                       map,
                                                   const TileKey&                   key,
                                                   const CreateTileModelFilter&     filter,
                                                   const TerrainEngineRequirements* requirements,
                                                   ProgressCallback*                progress)
{
    OE_PROFILING_ZONE;
    // Make a new model:
    osg::ref_ptr<TerrainTileModel> model = new TerrainTileModel(
        key,
        map->getDataModelRevision());

    // assemble all the components:
    addColorLayers(model.get(), map, requirements, key, filter, progress, true);

    if (requirements == 0L || requirements->elevationTexturesRequired())
    {
        unsigned border = (requirements && requirements->elevationBorderRequired()) ? 1u : 0u;

        addStandaloneElevation(model.get(), map, key, filter, border, progress);
    }

    addStandaloneLandCover(model.get(), map, key, filter, progress);

    //addPatchLayers(model.get(), map, key, filter, progress, true);

    // done.
    return model.release();
}

TerrainTileImageLayerModel*
TerrainTileModelFactory::addImageLayer(TerrainTileModel* model,
                                       ImageLayer* imageLayer,
                                       const TileKey& key,
                                       const TerrainEngineRequirements* reqs,
                                       ProgressCallback* progress)
{
    TerrainTileImageLayerModel* layerModel = NULL;
    osg::Texture* tex = 0L;
    TextureWindow window;
    osg::Matrix scaleBiasMatrix;
        
    if (imageLayer->isKeyInLegalRange(key) && imageLayer->mayHaveData(key))
    {
        if (imageLayer->useCreateTexture())
        {
            window = imageLayer->createTexture(key, progress);
            tex = window.getTexture();
            scaleBiasMatrix = window.getMatrix();
        }

        else
        {
            GeoImage geoImage = imageLayer->createImage(key, progress);

            if (geoImage.valid())
            {
                if (imageLayer->isCoverage())
                    tex = createCoverageTexture(geoImage.getImage());
                else
                    tex = createImageTexture(geoImage.getImage(), imageLayer);
            }
        }
    }

    // if this is the first LOD, and the engine requires that the first LOD
    // be populated, make an empty texture if we didn't get one.
    if (tex == 0L &&
        _options.firstLOD() == key.getLOD() &&
        reqs && reqs->fullDataAtFirstLodRequired())
    {
        tex = _emptyTexture.get();
    }

    if (tex)
    {
        tex->setName(model->getKey().str());

        layerModel = new TerrainTileImageLayerModel();

        layerModel->setImageLayer(imageLayer);

        layerModel->setTexture(tex);
        layerModel->setMatrix(new osg::RefMatrixf(scaleBiasMatrix));

        model->colorLayers().push_back(layerModel);

        if (imageLayer->isShared())
        {
            model->sharedLayers().push_back(layerModel);
        }

        if (imageLayer->isDynamic())
        {
            model->setRequiresUpdateTraverse(true);
        }
    }

    return layerModel;
}

void
TerrainTileModelFactory::addStandaloneImageLayer(
    TerrainTileModel* model,
    ImageLayer* imageLayer,
    const TileKey& key,
    const TerrainEngineRequirements* reqs,
    ProgressCallback* progress)
{
    TerrainTileImageLayerModel* layerModel = NULL;
    TileKey keyToUse = key;
    osg::Matrixf scaleBiasMatrix;
    while (keyToUse.valid() && !layerModel)
    {
        layerModel = addImageLayer(model, imageLayer, keyToUse, reqs, progress);
        if (!layerModel)
        {
            TileKey parentKey = keyToUse.createParentKey();
            if (parentKey.valid())
            {
                osg::Matrixf sb;
                keyToUse.getExtent().createScaleBias(parentKey.getExtent(), sb);
                scaleBiasMatrix.preMult(sb);
            }
            keyToUse = parentKey;
        }
    }
    if (layerModel)
    {
        layerModel->setMatrix(new osg::RefMatrixf(scaleBiasMatrix));
    }
}

void
TerrainTileModelFactory::addColorLayers(TerrainTileModel* model,
                                        const Map* map,
                                        const TerrainEngineRequirements* reqs,
                                        const TileKey&    key,
                                        const CreateTileModelFilter& filter,
                                        ProgressCallback* progress,
                                        bool standalone)
{
    OE_PROFILING_ZONE;
    OE_START_TIMER(fetch_image_layers);

    int order = 0;

    LayerVector layers;
    map->getLayers(layers);

    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        Layer* layer = i->get();

        if (layer->getRenderType() != layer->RENDERTYPE_TERRAIN_SURFACE)
            continue;

        if (!layer->getEnabled())
            continue;

        if (!filter.accept(layer))
            continue;

        ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer);
        if (imageLayer)
        {
            if (standalone)
            {
                addStandaloneImageLayer(model, imageLayer, key, reqs, progress);
            }
            else
            {
                addImageLayer(model, imageLayer, key, reqs, progress);
            }
        }
        else // non-image kind of TILE layer:
        {
            TerrainTileColorLayerModel* colorModel = new TerrainTileColorLayerModel();
            colorModel->setLayer(layer);
            model->colorLayers().push_back(colorModel);
        }
    }

    if (progress)
        progress->stats()["fetch_imagery_time"] += OE_STOP_TIMER(fetch_image_layers);
}


void
TerrainTileModelFactory::addPatchLayers(TerrainTileModel* model,
                                        const Map* map,
                                        const TileKey&    key,
                                        const CreateTileModelFilter& filter,
                                        ProgressCallback* progress,
                                        bool fallback)
{
    OE_START_TIMER(fetch_patch_layers);

    PatchLayerVector patchLayers;
    map->getLayers(patchLayers);

    for(PatchLayerVector::const_iterator i = patchLayers.begin();
        i != patchLayers.end();
        ++i )
    {
        PatchLayer* layer = i->get();

        if (!filter.accept(layer))
            continue;

        if (!layer->getEnabled())
            continue;

        if (layer->getAcceptCallback() == 0L || layer->getAcceptCallback()->acceptKey(key))
        {
            GeoNode node = layer->createNode(key, progress);
            if (node.valid())
            {
                TerrainTilePatchLayerModel* patchModel = new TerrainTilePatchLayerModel();
                patchModel->setPatchLayer(layer);
                patchModel->setNode(node.getNode());
            }
        }
    }

    if (progress)
        progress->stats()["fetch_patches_time"] += OE_STOP_TIMER(fetch_patch_layers);
}


void
TerrainTileModelFactory::addElevation(TerrainTileModel*            model,
                                      const Map*                   map,
                                      const TileKey&               key,
                                      const CreateTileModelFilter& filter,
                                      unsigned                     border,
                                      ProgressCallback*            progress )
{
    // make an elevation layer.
    OE_START_TIMER(fetch_elevation);

    bool accepted = true;
    ElevationLayerVector layers;
    map->getLayers(layers);

    if (!filter.empty())
    {
        accepted = false;
        for(ElevationLayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
        {
            if (filter.accept(i->get()))
            {
                accepted = true;
                break;
            }
        }
    }
    if (!accepted)
        return;

    const osgEarth::RasterInterpolation& interp = map->getElevationInterpolation();

    // Request a heightfield from the map.
    osg::ref_ptr<osg::HeightField> mainHF;
    osg::ref_ptr<NormalMap> normalMap;

    bool hfOK = getOrCreateHeightField(map, layers, key, SAMPLE_FIRST_VALID, interp, border, mainHF, normalMap, progress) && mainHF.valid();

    if (hfOK == false && key.getLOD() == _options.firstLOD().get())
    {
        OE_DEBUG << LC << "No HF at key " << key.str() << ", making placeholder" << std::endl;
        mainHF = new osg::HeightField();
        mainHF->allocate(1, 1);
        mainHF->setHeight(0, 0, 0.0f);
        hfOK = true;
    }

    if (hfOK && mainHF.valid())
    {
        osg::ref_ptr<TerrainTileElevationModel> layerModel = new TerrainTileElevationModel();
        layerModel->setHeightField( mainHF.get() );

        // pre-calculate the min/max heights:
        for( unsigned col = 0; col < mainHF->getNumColumns(); ++col )
        {
            for( unsigned row = 0; row < mainHF->getNumRows(); ++row )
            {
                float h = mainHF->getHeight(col, row);
                if ( h > layerModel->getMaxHeight() )
                    layerModel->setMaxHeight( h );
                if ( h < layerModel->getMinHeight() )
                    layerModel->setMinHeight( h );
            }
        }

        // needed for normal map generation
        model->heightFields().setNeighbor(0, 0, mainHF.get());

        // convert the heightfield to a 1-channel 32-bit fp image:
        ImageToHeightFieldConverter conv;
        osg::Image* hfImage = conv.convertToR32F(mainHF.get());

        if ( hfImage )
        {
            // Made an image, so store this as a texture with no matrix.
            osg::Texture* texture = createElevationTexture( hfImage );
            layerModel->setTexture( texture );
            model->elevationModel() = layerModel.get();
        }

        if (normalMap.valid())
        {
            TerrainTileImageLayerModel* layerModel = new TerrainTileImageLayerModel();
            layerModel->setName( "oe_normal_map" );

            // Made an image, so store this as a texture with no matrix.
            osg::Texture* texture = createNormalTexture(normalMap.get(), *_options.compressNormalMaps());
            layerModel->setTexture( texture );
            model->normalModel() = layerModel;
        }
    }

    if (progress)
        progress->stats()["fetch_elevation_time"] += OE_STOP_TIMER(fetch_elevation);
}

bool
TerrainTileModelFactory::getOrCreateHeightField(const Map*                      map,
                                                const ElevationLayerVector&     layers,
                                                const TileKey&                  key,
                                                ElevationSamplePolicy           samplePolicy,
                                                RasterInterpolation             interpolation,
                                                unsigned                        border,
                                                osg::ref_ptr<osg::HeightField>& out_hf,
                                                osg::ref_ptr<NormalMap>&        out_normalMap,
                                                ProgressCallback*               progress)
{
    OE_PROFILING_ZONE;

    // gather the combined revision (additive is fine)
    int combinedLayerRevision = 0;
    for(ElevationLayerVector::const_iterator i = layers.begin();
        i != layers.end();
        ++i)
    {
        // need layer UID too? gw
        combinedLayerRevision += i->get()->getRevision();
    }
    
    // check the quick cache.
    HFCacheKey cachekey;
    cachekey._key          = key;
    cachekey._revision     = (int)map->getDataModelRevision() + combinedLayerRevision;
    cachekey._samplePolicy = samplePolicy;

    if (progress)
        progress->stats()["hfcache_try_count"] += 1;

    bool hit = false;
    HFCache::Record rec;
    if ( _heightFieldCacheEnabled && _heightFieldCache.get(cachekey, rec) )
    {
        out_hf = rec.value()._hf.get();
        out_normalMap = rec.value()._normalMap.get();

        if (progress)
        {
            progress->stats()["hfcache_hit_count"] += 1;
            progress->stats()["hfcache_hit_rate"] = progress->stats()["hfcache_hit_count"]/progress->stats()["hfcache_try_count"];
        }

        return true;
    }

    if ( !out_hf.valid() )
    {
        out_hf = HeightFieldUtils::createReferenceHeightField(
            key.getExtent(),
            257, 257,           // base tile size for elevation data
            border,             // 1 sample border around the data makes it 259x259
            true);              // initialize to HAE (0.0) heights
    }

    if (!out_normalMap.valid())
    {
        //OE_INFO << "TODO: check terrain reqs\n";
        out_normalMap = new NormalMap(257, 257); // ImageUtils::createEmptyImage(257, 257);
    }

    bool populated = layers.populateHeightFieldAndNormalMap(
        out_hf.get(),
        out_normalMap.get(),
        key,
        map->getProfileNoVDatum(), // convertToHAE,
        interpolation,
        progress );

#ifdef TREAT_ALL_ZEROS_AS_MISSING_TILE
    // check for a real tile with all zeros and treat it the same as non-existant data.
    if ( populated )
    {
        bool isEmpty = true;
        for(osg::FloatArray::const_iterator f = out_hf->getFloatArray()->begin(); f != out_hf->getFloatArray()->end(); ++f)
        {
            if ( (*f) != 0.0f )
            {
                isEmpty = false;
                break;
            }
        }
        if ( isEmpty )
        {
            populated = false;
        }
    }
#endif

    if ( populated )
    {
        // cache it.
        if (_heightFieldCacheEnabled )
        {
            HFCacheValue newValue;
            newValue._hf = out_hf.get();
            newValue._normalMap = out_normalMap.get();

            _heightFieldCache.insert( cachekey, newValue );
        }
    }

    return populated;
}

void
TerrainTileModelFactory::addStandaloneElevation(
    TerrainTileModel*            model,
    const Map*                   map,
    const TileKey&               key,
    const CreateTileModelFilter& filter,
    unsigned                     border,
    ProgressCallback*            progress)
{
    TileKey keyToUse = key;
    while (keyToUse.valid() && model->elevationModel().valid() == false)
    {
        addElevation(model, map, keyToUse, filter, border, progress);
        if (model->elevationModel() == NULL)
        {
            keyToUse = keyToUse.createParentKey();
        }
    }
    if (model->elevationModel().valid())
    {
        osg::Matrixf scaleBiasMatrix;
        key.getExtent().createScaleBias(keyToUse.getExtent(), scaleBiasMatrix);
        model->elevationModel()->setMatrix(new osg::RefMatrixf(scaleBiasMatrix));
    }
}

TerrainTileLandCoverModel*
TerrainTileModelFactory::addLandCover(TerrainTileModel*            model,
                                      const Map*                   map,
                                      const TileKey&               key,
                                      const CreateTileModelFilter& filter,
                                      ProgressCallback*            progress)
{
    TerrainTileLandCoverModel* landCoverModel = NULL;

    LandCoverLayerVector layers;
    map->getLayers(layers);

    osg::ref_ptr<osg::Image> coverageImage;

    osg::ref_ptr<osg::Texture> tex;

    if (layers.populateLandCoverImage(coverageImage, key, progress))
    {
        tex = createCoverageTexture(coverageImage.get());
    }

    if (tex)
    {
        tex->setName(model->getKey().str());

        landCoverModel = new TerrainTileLandCoverModel();

        landCoverModel->setTexture(tex.get());

        model->landCoverModel() = landCoverModel;
    }

    return landCoverModel;
}

void
TerrainTileModelFactory::addStandaloneLandCover(
    TerrainTileModel*            model,
    const Map*                   map,
    const TileKey&               key,
    const CreateTileModelFilter& filter,
    ProgressCallback*            progress)
{
    TerrainTileLandCoverModel* layerModel = NULL;
    TileKey keyToUse = key;
    osg::Matrixf scaleBiasMatrix;
    while (keyToUse.valid() && !layerModel)
    {
        layerModel = addLandCover(model, map, keyToUse, filter, progress);
        if (!layerModel)
        {
            TileKey parentKey = keyToUse.createParentKey();
            if (parentKey.valid())
            {
                osg::Matrixf sb;
                keyToUse.getExtent().createScaleBias(parentKey.getExtent(), sb);
                scaleBiasMatrix.preMult(sb);
            }
            keyToUse = parentKey;
        }
    }
    if (layerModel)
    {
        layerModel->setMatrix(new osg::RefMatrixf(scaleBiasMatrix));
    }
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
        layer ? layer->options().magFilter().get() : osg::Texture::LINEAR;
    osg::Texture::FilterMode minFilter =
        layer ? layer->options().minFilter().get() : osg::Texture::LINEAR;

    tex->setFilter( osg::Texture::MAG_FILTER, magFilter );
    tex->setFilter( osg::Texture::MIN_FILTER, minFilter );
    tex->setMaxAnisotropy( 4.0f );

    // Disable mip mapping for npot tiles
    if (!ImageUtils::isPowerOfTwo( image ) || (!image->isMipmap() && ImageUtils::isCompressed(image)))
    {
        tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    }

    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

    layer->applyTextureCompressionMode(tex);

    ImageUtils::activateMipMaps(tex);
    
    return tex;
}

osg::Texture*
TerrainTileModelFactory::createCoverageTexture(osg::Image* image) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );

    tex->setInternalFormat(GL_R16F);

    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint(false);

    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );

    tex->setMaxAnisotropy( 1.0f );

    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

    return tex;
}

osg::Texture*
TerrainTileModelFactory::createElevationTexture(osg::Image* image) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );
    tex->setInternalFormat(GL_R32F);
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
    tex->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint( false );
    tex->setMaxAnisotropy( 1.0f );
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    return tex;
}

osg::Texture*
TerrainTileModelFactory::createNormalTexture(osg::Image* image, bool compress) const
{
    if (compress)
    {            
        // Only compress the image if it's not already compressed.
        if (image->getPixelFormat() != GL_COMPRESSED_RED_GREEN_RGTC2_EXT)
        {
            // See if we have a CPU compressor generator:
            osgDB::ImageProcessor* ip = osgDB::Registry::instance()->getImageProcessor();
            if (ip)
            {
                ip->compress(*image, osg::Texture::USE_RGTC2_COMPRESSION, true, true, osgDB::ImageProcessor::USE_CPU, osgDB::ImageProcessor::NORMAL);
            }
            else
            {
                OE_NOTICE << LC << "Failed to get image processor, cannot compress normal map" << std::endl;
            }
        }
    }    

    osg::Texture2D* tex = new osg::Texture2D(image);
    tex->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setMaxAnisotropy(1.0f);
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    return tex;
}
