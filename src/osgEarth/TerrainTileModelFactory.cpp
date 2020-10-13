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
#include <osgEarth/TerrainConstraintLayer>
#include <osgEarth/Metrics>

#include <osg/Texture2D>
#include <osg/Texture2DArray>

#define LC "[TerrainTileModelFactory] "

using namespace osgEarth;

class FutureImage : public osg::Image
{
public:

    FutureImage(ImageLayer* layer, const TileKey& key) : osg::Image()
    {
        _layer = layer;
        _key = key;

        osg::observer_ptr<ImageLayer> layer_ptr(_layer);

        Job<const osg::Image> job([layer_ptr, key](Cancelable* progress) mutable {
            osg::ref_ptr<ImageLayer> safe(layer_ptr);
            if (safe.valid()) {
                GeoImage result = safe->createImage(key, nullptr); // progress TODO
                return result.takeImage();
            }
            else return static_cast<const osg::Image*>(nullptr);
        });

        _result = job.schedule("ASYNC_LAYER");
    }

    virtual bool requiresUpdateCall() const override
    {
        // tricky, because if we return false here, it will
        // never get called again.
        return _result.isAvailable() || !_result.isAbandoned();
    }

    virtual void update(osg::NodeVisitor* nv) override
    {
        if (_result.isAvailable())
        {
            // no refptr here because we are going to steal the data.
            osg::ref_ptr<osg::Image> i = const_cast<osg::Image*>(_result.release());

            if (i.valid())
            {
                this->setImage(
                    i->s(), i->t(), i->r(),
                    i->getInternalTextureFormat(), i->getPixelFormat(), i->getDataType(),
                    i->data(), i->getAllocationMode(),
                    i->getPacking(),
                    i->getRowLength());

                // since we stole the data, make sure we don't double-delete it
                i->setAllocationMode(osg::Image::NO_DELETE);

                // trigger texture(s) that own this image to reapply
                this->dirty();
            }
        }
    }

    osg::ref_ptr<ImageLayer> _layer;
    TileKey _key;
    Job<const osg::Image>::Result _result;
};

//.........................................................................

CreateTileManifest::CreateTileManifest()
{
    _includesElevation = false;
    _includesConstraints = false;
    _includesLandCover = false;
}

void CreateTileManifest::insert(const Layer* layer)
{
    if (layer)
    {
        _layers[layer->getUID()] = layer->getRevision();

        if (dynamic_cast<const ElevationLayer*>(layer))
        {
            _includesElevation = true;
        }

        else if (dynamic_cast<const TerrainConstraintLayer*>(layer))
        {
            _includesConstraints = true;
        }

        else if (dynamic_cast<const LandCoverLayer*>(layer))
        {
            _includesLandCover = true;
        }
    }
}

bool CreateTileManifest::excludes(const Layer* layer) const
{
    return !empty() && _layers.find(layer->getUID()) == _layers.end();
}

bool CreateTileManifest::empty() const
{
    return _layers.empty();
}

bool CreateTileManifest::inSyncWith(const Map* map) const
{
    for(LayerTable::const_iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        const Layer* layer = map->getLayerByUID(i->first);

        // note: if the layer is NULL, it was removed, so let it pass.

        if (layer != NULL && layer->getRevision() != i->second)
        {
            return false;
        }
    }
    return true;
}

void CreateTileManifest::updateRevisions(const Map* map)
{
    for(LayerTable::iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        const Layer* layer = map->getLayerByUID(i->first);
        if (layer)
        {
            i->second = layer->getRevision();
        }
    }
}

bool CreateTileManifest::includes(const Layer* layer) const
{
    return includes(layer->getUID());
}

bool CreateTileManifest::includes(UID uid) const
{
    return empty() || _layers.find(uid) != _layers.end();
}

bool CreateTileManifest::includesElevation() const
{
    return empty() || _includesElevation;
}

bool CreateTileManifest::includesConstraints() const
{
    return _includesConstraints;
}

bool CreateTileManifest::includesLandCover() const
{
    return empty() || _includesLandCover;
}

//.........................................................................

TerrainTileModelFactory::TerrainTileModelFactory(const TerrainOptions& options) :
_options( options )
{
    // Create an empty texture that we can use as a placeholder
    _emptyColorTexture = new osg::Texture2D(ImageUtils::createEmptyImage());
    _emptyColorTexture->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

    osg::Image* landCoverImage = LandCover::createImage(1u);
    ImageUtils::PixelWriter writeLC(landCoverImage);
    writeLC(osg::Vec4(0,0,0,0), 0, 0);
    _emptyLandCoverTexture = new osg::Texture2D(landCoverImage);
    _emptyLandCoverTexture->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
}

TerrainTileModel*
TerrainTileModelFactory::createTileModel(
    const Map*                       map,
    const TileKey&                   key,
    const CreateTileManifest&        manifest,
    const TerrainEngineRequirements* requirements,
    ProgressCallback*                progress)
{
    OE_PROFILING_ZONE;
    // Make a new model:
    osg::ref_ptr<TerrainTileModel> model = new TerrainTileModel(
        key,
        map->getDataModelRevision() );

    // assemble all the components:
    addColorLayers(model.get(), map, requirements, key, manifest, progress, false);

    if ( requirements == 0L || requirements->elevationTexturesRequired() )
    {
        unsigned border = (requirements && requirements->elevationBorderRequired()) ? 1u : 0u;

        addElevation( model.get(), map, key, manifest, border, progress );
    }

    addLandCover(model.get(), map, key, requirements, manifest, progress);

    // done.
    return model.release();
}

TerrainTileModel*
TerrainTileModelFactory::createStandaloneTileModel(
    const Map*                       map,
    const TileKey&                   key,
    const CreateTileManifest&        manifest,
    const TerrainEngineRequirements* requirements,
    ProgressCallback*                progress)
{
    OE_PROFILING_ZONE;
    // Make a new model:
    osg::ref_ptr<TerrainTileModel> model = new TerrainTileModel(
        key,
        map->getDataModelRevision());

    // assemble all the components:
    addColorLayers(model.get(), map, requirements, key, manifest, progress, true);

    if (requirements == 0L || requirements->elevationTexturesRequired())
    {
        unsigned border = (requirements && requirements->elevationBorderRequired()) ? 1u : 0u;

        addStandaloneElevation(model.get(), map, key, manifest, border, progress);
    }

    addStandaloneLandCover(model.get(), map, key, requirements, manifest, progress);

    //addPatchLayers(model.get(), map, key, filter, progress, true);

    // done.
    return model.release();
}

TerrainTileImageLayerModel*
TerrainTileModelFactory::addImageLayer(
    TerrainTileModel* model,
    ImageLayer* imageLayer,
    const TileKey& key,
    const TerrainEngineRequirements* reqs,
    ProgressCallback* progress)
{
    OE_PROFILING_ZONE;
    OE_PROFILING_ZONE_TEXT(imageLayer->getName());

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

        else if (imageLayer->getAsyncLoading() == true)
        {
            osg::Image* image = new FutureImage(imageLayer, key);

            tex = new osg::Texture2D(image);
            tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            tex->setResizeNonPowerOfTwoHint(false);
            osg::Texture::FilterMode magFilter = imageLayer->options().magFilter().get();
            osg::Texture::FilterMode minFilter = imageLayer->options().minFilter().get();
            tex->setFilter(osg::Texture::MAG_FILTER, magFilter);
            tex->setFilter(osg::Texture::MIN_FILTER, minFilter);
            tex->setMaxAnisotropy(4.0f);
            tex->setUnRefImageDataAfterApply(false);
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
        tex = _emptyColorTexture.get();
    }

    if (tex)
    {
        tex->setName(model->getKey().str());

        layerModel = new TerrainTileImageLayerModel();

        layerModel->setImageLayer(imageLayer);

        layerModel->setTexture(tex);
        layerModel->setMatrix(new osg::RefMatrixf(scaleBiasMatrix));
        layerModel->setRevision(imageLayer->getRevision());

        model->colorLayers().push_back(layerModel);

        if (imageLayer->isShared())
        {
            model->sharedLayers().push_back(layerModel);
        }

        if (imageLayer->isDynamic() || imageLayer->getAsyncLoading())
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
                scaleBiasMatrix.postMult(sb);
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
TerrainTileModelFactory::addColorLayers(
    TerrainTileModel* model,
    const Map* map,
    const TerrainEngineRequirements* reqs,
    const TileKey& key,
    const CreateTileManifest& manifest,
    ProgressCallback* progress,
    bool standalone)
{
    OE_PROFILING_ZONE;

    int order = 0;

    LayerVector layers;
    map->getLayers(layers);

    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        Layer* layer = i->get();

        if (!layer->isOpen())
            continue;

        if (layer->getRenderType() != layer->RENDERTYPE_TERRAIN_SURFACE)
            continue;

        if (manifest.excludes(layer))
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
            colorModel->setRevision(layer->getRevision());
            model->colorLayers().push_back(colorModel);
        }
    }
}

#if 0
void
TerrainTileModelFactory::addPatchLayers(
    TerrainTileModel* model,
    const Map* map,
    const TileKey&    key,
    const CreateTileManifest& manifest,
    ProgressCallback* progress,
    bool fallback)
{
    PatchLayerVector patchLayers;
    map->getLayers(patchLayers);

    for(PatchLayerVector::const_iterator i = patchLayers.begin();
        i != patchLayers.end();
        ++i )
    {
        PatchLayer* layer = i->get();

        if (!layer->isOpen())
            continue;

        if (manifest.excludes(layer))
            continue;

        if (layer->getAcceptCallback() == 0L || layer->getAcceptCallback()->acceptKey(key))
        {
            GeoNode node = layer->createNode(key, progress);
            if (node.valid())
            {
                TerrainTilePatchLayerModel* patchModel = new TerrainTilePatchLayerModel();
                patchModel->setPatchLayer(layer);
                patchModel->setRevision(layer->getRevision());
                patchModel->setNode(node.getNode());
            }
        }
    }
}
#endif

void
TerrainTileModelFactory::addElevation(
    TerrainTileModel*            model,
    const Map*                   map,
    const TileKey&               key,
    const CreateTileManifest&    manifest,
    unsigned                     border,
    ProgressCallback*            progress)
{
    OE_PROFILING_ZONE;
    OE_PROFILING_ZONE_TEXT("Elevation");

    bool needElevation = manifest.includesElevation();
    ElevationLayerVector layers;
    map->getLayers(layers);

    int combinedRevision = map->getDataModelRevision();
    if (!manifest.empty())
    {
        for (const auto& layer : layers)
        {
            if (needElevation == false && !manifest.excludes(layer.get()))
            {
                needElevation = true;
            }
            combinedRevision += layer->getRevision();
        }
    }
    if (!needElevation)
        return;

#if 0
    LayerVector terrain_layers;

    map->getLayers(
        terrain_layers,
        [] (const Layer* layer) {
            return
                dynamic_cast<const ElevationLayer*>(layer) != nullptr ||
                dynamic_cast<const TerrainConstraintLayer*>(layer) != nullptr;
        });

    int combinedRevision = map->getDataModelRevision();

    if (!manifest.empty())
    {
        for(const auto& layer : terrain_layers)
        {
            if (needElevation == false && !manifest.excludes(layer.get()))
            {
                needElevation = true;
            }
            combinedRevision += layer->getRevision();
        }
    }
    if (!needElevation)
        return;
#endif

    osg::ref_ptr<ElevationTexture> elevTex;

    bool getNormalMap = (_options.normalMaps() == true);

    const bool acceptLowerRes = false;

    if (map->getElevationPool()->getTile(key, acceptLowerRes, elevTex, &_workingSet, progress))
    {
        osg::ref_ptr<TerrainTileElevationModel> layerModel = new TerrainTileElevationModel();

        layerModel->setRevision(combinedRevision);

        if ( elevTex.valid() )
        {
            // Make a normal map if it doesn't already exist
            elevTex->generateNormalMap(map, &_workingSet, progress);

            // Made an image, so store this as a texture with no matrix.
            layerModel->setTexture( elevTex.get() );

            // Keep the heightfield pointer around for legacy 3rd party usage (VRF)
            layerModel->setHeightField(elevTex->getHeightField());

            model->elevationModel() = layerModel.get();
        }
    }
}

void
TerrainTileModelFactory::addStandaloneElevation(
    TerrainTileModel*            model,
    const Map*                   map,
    const TileKey&               key,
    const CreateTileManifest&    manifest,
    unsigned                     border,
    ProgressCallback*            progress)
{
    TileKey keyToUse = key;
    while (keyToUse.valid() && model->elevationModel().valid() == false)
    {
        addElevation(model, map, keyToUse, manifest, border, progress);
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
TerrainTileModelFactory::addLandCover(
    TerrainTileModel*            model,
    const Map*                   map,
    const TileKey&               key,
    const TerrainEngineRequirements* reqs,
    const CreateTileManifest&    manifest,
    ProgressCallback*            progress)
{
    OE_PROFILING_ZONE;
    OE_PROFILING_ZONE_TEXT("LandCover");

    TerrainTileLandCoverModel* landCoverModel = NULL;

    // Note. We only support one land cover layer...
    LandCoverLayerVector layers;
    map->getLayers(layers);
    int combinedRevision = map->getDataModelRevision();

    // any land cover layer means using them all:
    bool needLandCover = manifest.includesLandCover();

    if (!manifest.empty())
    {
        for(LandCoverLayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
        {
            const LandCoverLayer* layer = i->get();
            if (layer->isOpen())
            {
                if (needLandCover == false && !manifest.excludes(layer))
                {
                    needLandCover = true;
                }

                combinedRevision += layer->getRevision();
            }
        }
    }

    if (!needLandCover)
    {
        return NULL;
    }

    osg::ref_ptr<osg::Image> coverageImage;

    osg::ref_ptr<osg::Texture> tex;

    if (layers.populateLandCoverImage(coverageImage, key, progress))
    {
        tex = createCoverageTexture(coverageImage.get());
    }

    // if this is the first LOD, and the engine requires that the first LOD
    // be populated, make an empty texture if we didn't get one.
    if (tex == 0L &&
        _options.firstLOD() == key.getLOD() &&
        reqs && reqs->fullDataAtFirstLodRequired())
    {
        tex = _emptyLandCoverTexture.get();
    }

    if (tex)
    {
        tex->setName(model->getKey().str());

        landCoverModel = new TerrainTileLandCoverModel();
        landCoverModel->setRevision(combinedRevision);

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
    const TerrainEngineRequirements* reqs,
    const CreateTileManifest&    manifest,
    ProgressCallback*            progress)
{
    TerrainTileLandCoverModel* layerModel = NULL;
    TileKey keyToUse = key;
    osg::Matrixf scaleBiasMatrix;
    while (keyToUse.valid() && !layerModel)
    {
        layerModel = addLandCover(model, map, keyToUse, reqs, manifest, progress);
        if (!layerModel)
        {
            TileKey parentKey = keyToUse.createParentKey();
            if (parentKey.valid())
            {
                osg::Matrixf sb;
                keyToUse.getExtent().createScaleBias(parentKey.getExtent(), sb);
                scaleBiasMatrix.postMult(sb);
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
TerrainTileModelFactory::createImageTexture(const osg::Image* image,
                                            const ImageLayer* layer) const
{
    osg::Texture* tex = nullptr;
    bool hasMipMaps = false;
    bool isCompressed = false;

    // figure out the texture compression method to use (if any)
    std::string compressionMethod = layer->getCompressionMethod();
    if (compressionMethod.empty())
        compressionMethod = _options.textureCompression().get();

    GLenum pixelFormat = image->getPixelFormat();
    GLenum internalFormat = image->getInternalTextureFormat();

    // Fix incorrect internal format if necessary
    if (internalFormat == pixelFormat)
    {
        if (pixelFormat == GL_RGB) internalFormat = GL_RGB8;
        else if (pixelFormat == GL_RGBA) internalFormat = GL_RGBA8;
        else if (pixelFormat == GL_RG) internalFormat = GL_RG8;
        else if (pixelFormat == GL_RED) internalFormat = GL_R8;
    }

    if (image->r() == 1)
    {
        osg::ref_ptr<const osg::Image> compressed = ImageUtils::compressImage(image, compressionMethod);
        const osg::Image* mipmapped = ImageUtils::mipmapImage(compressed.get());
        tex = new osg::Texture2D(const_cast<osg::Image*>(mipmapped));
        hasMipMaps = mipmapped->isMipmap();
        isCompressed = mipmapped->isCompressed();

        if (layer->getCompressionMethod() == "gpu" && !mipmapped->isCompressed())
            tex->setInternalFormatMode(tex->USE_S3TC_DXT5_COMPRESSION);
    }

    else // if (image->r() > 1)
    {
        std::vector< osg::ref_ptr<osg::Image> > images;
        ImageUtils::flattenImage(image, images);

        // Make sure we are using a proper sized internal format
        for(int i=0; i<images.size(); ++i)
        {
            images[i]->setInternalTextureFormat(internalFormat);
        }
        
        osg::ref_ptr<const osg::Image> compressed;
        for(auto& ref : images)
        {
            compressed = ImageUtils::compressImage(ref.get(), compressionMethod);
            ref = const_cast<osg::Image*>(ImageUtils::mipmapImage(compressed.get()));

            if (layer->getCompressionMethod() == "gpu" && !compressed->isCompressed())
                tex->setInternalFormatMode(tex->USE_S3TC_DXT5_COMPRESSION);

            hasMipMaps = compressed->isMipmap();
            isCompressed = compressed->isCompressed();
        }

        osg::Texture2DArray* tex2dArray = new osg::Texture2DArray();

        tex2dArray->setTextureSize(image[0].s(), image[0].t(), images.size());
        tex2dArray->setInternalFormat(images[0]->getInternalTextureFormat());
        tex2dArray->setSourceFormat(images[0]->getPixelFormat());
        for (int i = 0; i < (int)images.size(); ++i)
            tex2dArray->setImage(i, const_cast<osg::Image*>(images[i].get()));

        tex = tex2dArray;
    }

    tex->setDataVariance(osg::Object::STATIC);
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

    // Disable mip mapping if we don't have it
    if (!hasMipMaps)
    {
        tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    }

    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

    // For GL_RED, swizzle the RGBA all to RED in order to match old GL_LUMINANCE behavior
    for(unsigned i=0; i<tex->getNumImages(); ++i)
    {
        if (tex->getImage(i) && tex->getImage(i)->getPixelFormat() == GL_RED)
        {
            tex->setSwizzle(osg::Vec4i(GL_RED, GL_RED, GL_RED, GL_RED));
            break;
        }
    }
    
    return tex;
}

osg::Texture*
TerrainTileModelFactory::createCoverageTexture(const osg::Image* image) const
{
    osg::Texture2D* tex = new osg::Texture2D(const_cast<osg::Image*>(image));
    tex->setDataVariance(osg::Object::STATIC);

    tex->setInternalFormat(LandCover::getTextureFormat());

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
TerrainTileModelFactory::createElevationTexture(const osg::Image* image) const
{
    osg::Texture2D* tex = new osg::Texture2D(const_cast<osg::Image*>(image));
    tex->setDataVariance(osg::Object::STATIC);
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
