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
#include "TerrainTileModelFactory"
#include "ImageToHeightFieldConverter"
#include "Map"
#include "Registry"
#include "LandCoverLayer"
#include "TerrainConstraintLayer"
#include "Metrics"

#include <osg/Texture2D>
#include <osg/Texture2DArray>

#define LC "[TerrainTileModelFactory] "

using namespace osgEarth;

#define LABEL_IMAGERY "REX textures"
#define LABEL_NORMALMAP "REX textures"
#define LABEL_ELEVATION "REX textures"
#define LABEL_COVERAGE "REX textures"

//.........................................................................

CreateTileManifest::CreateTileManifest()
{
    _includesElevation = false;
    _includesConstraints = false;
    _includesLandCover = false;
    _progressive.setDefault(false);
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

void CreateTileManifest::setProgressive(bool value)
{
    _progressive = value;
}

//.........................................................................

TerrainTileModelFactory::TerrainTileModelFactory(
    const TerrainOptions& options) :
    _options(options)
{
    //nop
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

    if ( requirements == nullptr || requirements->elevationTexturesRequired() )
    {
        unsigned border = (requirements && requirements->elevationBorderRequired()) ? 1u : 0u;

        addElevation( model.get(), map, key, manifest, border, progress );
    }

    if (requirements == nullptr || requirements->landCoverTexturesRequired())
    {
        addLandCover(model.get(), map, key, requirements, manifest, progress);
    }

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

    // done.
    return model.release();
}

bool
TerrainTileModelFactory::addImageLayer(
    TerrainTileModel* model,
    ImageLayer* imageLayer,
    const TileKey& key,
    const TerrainEngineRequirements* reqs,
    ProgressCallback* progress)
{
    OE_PROFILING_ZONE;
    OE_PROFILING_ZONE_TEXT(imageLayer->getName());

    if (!imageLayer->isOpen())
        return false;

    TextureWindow window;
    osg::Matrix scaleBiasMatrix;
    Texture::Ptr tex;

    if (imageLayer->isKeyInLegalRange(key) && imageLayer->mayHaveData(key))
    {
        if (imageLayer->useCreateTexture())
        {
            window = imageLayer->createTexture(key, progress);
            if (window.getTexture())
            {
                tex = Texture::create(window.getTexture());
                scaleBiasMatrix = window.getMatrix();
            }
        }

        else if (imageLayer->getAsyncLoading() == true)
        {
            osg::Texture* t = new FutureTexture2D(imageLayer, key);
            t->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            t->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            t->setResizeNonPowerOfTwoHint(false);
            osg::Texture::FilterMode magFilter = imageLayer->options().magFilter().get();
            osg::Texture::FilterMode minFilter = imageLayer->options().minFilter().get();
            t->setFilter(osg::Texture::MAG_FILTER, magFilter);
            t->setFilter(osg::Texture::MIN_FILTER, minFilter);
            t->setMaxAnisotropy(4.0f);
            t->setUnRefImageDataAfterApply(false);

            tex = Texture::create(t);
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

                // Propagate the tracking token to the texture if there is one:
                if (tex && geoImage.getTrackingToken())
                {
                    tex->osgTexture()->getOrCreateUserDataContainer()->addUserObject(
                        geoImage.getTrackingToken());
                }
            }

        }
    }

    // if this is the first LOD, and the engine requires that the first LOD
    // be populated, make an empty texture if we didn't get one.
    if (tex == nullptr &&
        _options.firstLOD() == key.getLOD() &&
        reqs && reqs->fullDataAtFirstLodRequired())
    {
        tex = Texture::create(ImageUtils::createEmptyImage());
    }

    if (tex)
    {
        tex->label() = LABEL_IMAGERY;

        tex->name() =
            model->key().str() + ":" +
            (imageLayer->getName().empty() ? "(unnamed image layer)" : imageLayer->getName());

        TerrainTileModel::ColorLayer layerModel;
        layerModel.layer() = imageLayer;
        layerModel.texture() = tex;
        layerModel.matrix() = scaleBiasMatrix;
        layerModel.revision() = imageLayer->getRevision();

        if (imageLayer->isShared())
        {
            model->sharedLayerIndices().push_back(
                model->colorLayers().size());
        }

        if (imageLayer->isDynamic() || imageLayer->getAsyncLoading())
        {
            model->requiresUpdateTraversal() = true;
        }

        model->colorLayers().push_back(std::move(layerModel));
        return true;
    }

    return false;

    return &model->colorLayers().back();
}

void
TerrainTileModelFactory::addStandaloneImageLayer(
    TerrainTileModel* model,
    ImageLayer* imageLayer,
    const TileKey& key,
    const TerrainEngineRequirements* reqs,
    ProgressCallback* progress)
{
    //TerrainTileImageLayerModel* layerModel = NULL;
    TileKey keyToUse = key;
    osg::Matrixf scaleBiasMatrix;
    bool added = false;
    while (keyToUse.valid() && !added)
    {
        bool added = addImageLayer(model, imageLayer, keyToUse, reqs, progress);
        if (!added)
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

    if (added)
    {
        model->colorLayers().back().matrix() = scaleBiasMatrix;
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

        // skip layers that are closed
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
        else // non-image kind of TILE layer (e.g., splatting)
        {
            TerrainTileModel::ColorLayer colorModel;
            colorModel.layer() = layer;
            colorModel.revision() = layer->getRevision();
            model->colorLayers().push_back(std::move(colorModel));
        }
    }
}

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


    osg::ref_ptr<ElevationTexture> elevTex;

    const bool acceptLowerRes = false;

    if (map->getElevationPool()->getTile(key, acceptLowerRes, elevTex, &_workingSet, progress))
    {
        if (elevTex.valid())
        {
            model->elevation().revision() = combinedRevision;
            model->elevation().texture() = Texture::create(elevTex.get());
            model->elevation().texture()->label() = LABEL_ELEVATION;

            if (_options.useNormalMaps() == true)
            {
                // Make a normal map if it doesn't already exist
                elevTex->generateNormalMap(map, &_workingSet, progress);

                if (elevTex->getNormalMapTexture())
                {
                    elevTex->getNormalMapTexture()->setName(key.str() + ":normalmap");
                    model->normalMap().texture() = Texture::create(elevTex->getNormalMapTexture());
                    model->normalMap().texture()->label() = LABEL_NORMALMAP;
                }
            }

            // Keep the heightfield pointer around for legacy 3rd party usage (VRF)
            model->elevation().heightField() = elevTex->getHeightField();
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
    while (keyToUse.valid() && model->elevation().texture() == nullptr)
    {
        addElevation(model, map, keyToUse, manifest, border, progress);

        if (model->elevation().texture() == nullptr)
        {
            keyToUse = keyToUse.createParentKey();
        }
    }
    if (model->elevation().texture() != nullptr)
    {
        osg::Matrixf scaleBiasMatrix;
        key.getExtent().createScaleBias(keyToUse.getExtent(), scaleBiasMatrix);
        model->elevation().matrix() = scaleBiasMatrix;
    }
}

bool
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

    if (layers.populateLandCoverImage(coverageImage, key, progress))
    {
        model->landCover().texture() = createCoverageTexture(coverageImage.get());
    }

    // if this is the first LOD, and the engine requires that the first LOD
    // be populated, make an empty texture if we didn't get one.
    if (model->landCover().texture() == nullptr &&
        _options.firstLOD() == key.getLOD() &&
        reqs && reqs->fullDataAtFirstLodRequired())
    {
        osg::Image* landCoverImage = LandCover::createImage(1u);
        ImageUtils::PixelWriter writeLC(landCoverImage);
        writeLC(osg::Vec4(0, 0, 0, 0), 0, 0);
        model->landCover().texture() = Texture::create(landCoverImage);
    }

    if (model->landCover().texture())
    {
        model->landCover().texture()->label() = LABEL_COVERAGE;
        model->landCover().texture()->name() = model->key().str() + ":landcover";
    }

    return model->landCover().texture() != nullptr;
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
    TileKey keyToUse = key;
    osg::Matrixf scaleBiasMatrix;
    while (keyToUse.valid() && !model->landCover().texture())
    {
        addLandCover(model, map, keyToUse, reqs, manifest, progress);
        if (!model->landCover().texture())
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
    if (model->landCover().texture())
    {
        model->landCover().matrix() = scaleBiasMatrix;
    }
}

namespace
{
    //#define DEBUG_TEXTURES

    struct DebugTexture2D : public osg::Texture2D
    {
        DebugTexture2D(osg::Image* image) : osg::Texture2D(image) { }
        virtual ~DebugTexture2D() {
            OE_INFO << "Deleted texture " << getName() << std::endl;
        }
        void releaseGLObjects(osg::State* state) const {
            osg::Texture2D::releaseGLObjects(state);
            OE_INFO << "Released texture " << getName() << std::endl;
        }
    };

    osg::Texture2D* createTexture2D(const osg::Image* image)
    {
#ifdef DEBUG_TEXTURES
        return new DebugTexture2D(const_cast<osg::Image*>(image));
#else
        return new osg::Texture2D(const_cast<osg::Image*>(image));
#endif
    }
}

Texture::Ptr
TerrainTileModelFactory::createImageTexture(
    const osg::Image* image,
    const ImageLayer* layer) const
{
    if (image == nullptr || layer == nullptr)
        return nullptr;

    osg::Texture* tex = nullptr;
    bool hasMipMaps = false;
    bool isCompressed = false;

    if (image->requiresUpdateCall())
    {
        // image sequences and other data that updates itself
        // shall not be mipmapped/compressed here
        tex = createTexture2D(image);
    }
    else
    {
        // figure out the texture compression method to use (if any)
        std::string compressionMethod = layer->getCompressionMethod();
        if (compressionMethod.empty())
            compressionMethod = _options.textureCompression().get();

        GLenum pixelFormat = image->getPixelFormat();
        GLenum internalFormat = image->getInternalTextureFormat();
        GLenum dataType = image->getDataType();

        // Fix incorrect internal format if necessary
        if (internalFormat == pixelFormat)
        {
            int bits = dataType == GL_UNSIGNED_BYTE ? 8 : 16;
            if (pixelFormat == GL_RGB) internalFormat = bits == 8 ? GL_RGB8 : GL_RGB16;
            else if (pixelFormat == GL_RGBA) internalFormat = bits == 8 ? GL_RGBA8 : GL_RGBA16;
            else if (pixelFormat == GL_RG) internalFormat = bits == 8 ? GL_RG8 : GL_RG16;
            else if (pixelFormat == GL_RED) internalFormat = bits == 8 ? GL_R8 : GL_R16;
        }

        if (image->r() == 1)
        {
            osg::ref_ptr<const osg::Image> compressed = ImageUtils::compressImage(image, compressionMethod);
            const osg::Image* mipmapped = ImageUtils::mipmapImage(compressed.get());

            tex = createTexture2D(mipmapped);

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
            for (int i = 0; i < images.size(); ++i)
            {
                images[i]->setInternalTextureFormat(internalFormat);
            }

            osg::ref_ptr<const osg::Image> compressed;
            for (auto& ref : images)
            {
                compressed = ImageUtils::compressImage(ref.get(), compressionMethod);
                ref = const_cast<osg::Image*>(ImageUtils::mipmapImage(compressed.get()));

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

            if (layer->getCompressionMethod() == "gpu" && !isCompressed)
                tex->setInternalFormatMode(tex->USE_S3TC_DXT5_COMPRESSION);
        }

        tex->setDataVariance(osg::Object::STATIC);
    }

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
    for(unsigned i=0; i< tex->getNumImages(); ++i)
    {
        if (tex->getImage(i) && tex->getImage(i)->getPixelFormat() == GL_RED)
        {
            tex->setSwizzle(osg::Vec4i(GL_RED, GL_RED, GL_RED, GL_RED));
            break;
        }
    }

    return Texture::create(tex);
}

Texture::Ptr
TerrainTileModelFactory::createCoverageTexture(const osg::Image* image) const
{
    osg::Texture2D* tex = createTexture2D(image);
    tex->setDataVariance(osg::Object::STATIC);

    tex->setInternalFormat(LandCover::getTextureFormat());

    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint(false);

    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );

    tex->setMaxAnisotropy( 1.0f );

    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

    return Texture::create(tex);
}
