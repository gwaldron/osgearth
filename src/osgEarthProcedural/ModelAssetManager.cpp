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
#include "BiomeManager"

#include <osgEarth/TextureArena>
#include <osgEarth/Elevation>
#include <osgEarth/GLUtils>
#include <osgEarth/Metrics>
#include <osgEarth/MaterialLoader>
#include <osgEarth/Utils>
#include <osgEarth/MeshConsolidator>
#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osgUtil/SmoothingVisitor>
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#undef LC
#define LC "[BiomeManager] "

#define OE_TEST OE_NULL

#define NORMAL_MAP_TEX_UNIT 1
#define PBR_TEX_UNIT 2


ModelAsset::ModelAsset(const Config& conf)
{
    scale().setDefault(1.0f);
    stiffness().setDefault(0.5f);
    minLush().setDefault(0.0f);
    maxLush().setDefault(1.0f);
    sizeVariation().setDefault(0.0f);
    width().setDefault(0.0f);
    height().setDefault(0.0f);
    topBillboardHeight().setDefault(0.0f);
    traitsRequired().setDefault(false);

    conf.get("url", modelURI());
    conf.get("name", name());
    conf.get("side_url", sideBillboardURI());
    conf.get("top_url", topBillboardURI());
    conf.get("width", width());
    conf.get("height", height());
    conf.get("scale", scale());
    conf.get("size_variation", sizeVariation());
    conf.get("stiffness", stiffness());
    conf.get("min_lush", minLush());
    conf.get("max_lush", maxLush());
    conf.get("top_height", topBillboardHeight());
    conf.get("traits", traits());
    conf.get("traits_required", traitsRequired());

    // save the original so the user can extract user-defined values
    _sourceConfig = conf;
}

Config
ModelAsset::getConfig() const
{
    Config conf("model");
    conf.set("name", name());
    conf.set("url", modelURI());
    conf.set("side_url", sideBillboardURI());
    conf.set("top_url", topBillboardURI());
    conf.set("width", width());
    conf.set("height", height());
    conf.set("scale", scale());
    conf.set("size_variation", sizeVariation());
    conf.set("stiffness", stiffness());
    conf.set("min_lush", minLush());
    conf.set("max_lush", maxLush());
    conf.set("top_height", topBillboardHeight());
    conf.set("traits", traits());
    conf.set("traits_required", traitsRequired());
    return conf;
}

//...................................................................

MaterialAsset::MaterialAsset(const Config& conf)
{
    conf.get("name", name());
    conf.get("url", uri());
    conf.get("size", size());
}

Config
MaterialAsset::getConfig() const
{
    Config conf("asset");
    conf.set("name", name());
    conf.set("url", uri());
    conf.set("size", size());
    return conf;
}

//...................................................................

AssetCatalog::AssetCatalog(const Config& conf)
{
    // load all model asset defs
    ConfigSet modelassetgroups = conf.child("models").children("group");
    if (modelassetgroups.empty()) modelassetgroups = conf.child("modelassets").children("group");
    for (const auto& c : modelassetgroups)
    {
        std::string group = c.value("name");
        ConfigSet modelassets = c.children("asset");
        for (const auto& m : modelassets)
        {
            ModelAsset asset(m);
            asset.group() = group;
            _models[asset.name()] = asset;
        }
    }
    // load in all materials first into a temporary table
    std::map<std::string, MaterialAsset> temp_materials; // use map since we're iterating
    ConfigSet materialassets = conf.child("materials").children("asset");
    for (const auto& c : materialassets)
    {
        MaterialAsset a(c);
        if (a.name().isSet())
            temp_materials[a.name().get()] = std::move(a);
    }

    // make a list of materials, with the lifemap matrix materials first
    // so their indices are 'well known' from here on out.
    std::unordered_set<std::string> added;

    // add substrate textures:
    ConfigSet substrates = conf.child("lifemapmatrix").child("substrate").children("asset");
    for (const auto& c : substrates)
    {
        auto iter = temp_materials.find(c.value("name"));
        if (iter != temp_materials.end())
        {
            if (added.find(iter->first) == added.end())
            {
                _materials.push_back(iter->second);
                added.emplace(iter->first);
            }
            else
            {
                OE_WARN << LC << "LifeMapMatrix materials must all be unique!" << std::endl;
            }
        }
        else
        {
            OE_WARN << LC << "Unrecognized material asset \"" << iter->second.name().get() << "\" referenced in the lifemap matrix"
                << std::endl;
        }
    }

    // append overlay textures:
    ConfigSet overlays = conf.child("lifemapmatrix").child("overlay").children("asset");
    for (const auto& c : overlays)
    {
        auto iter = temp_materials.find(c.value("name"));
        if (iter != temp_materials.end())
        {
            if (added.find(iter->first) == added.end())
            {
                _materials.push_back(iter->second);
                added.emplace(iter->first);
            }
            else
            {
                OE_WARN << LC << "LifeMapMatrix materials must all be unique!" << std::endl;
            }
        }
        else
        {
            OE_WARN << LC << "Unrecognized material asset \"" << iter->second.name().get() << "\" referenced in the lifemap matrix" << std::endl;
        }
    }

    _lifemapMatrixWidth = _materials.size() / 2;

    // load all the other textures at the end, avoiding dupes
    for (auto& iter : temp_materials)
    {
        if (added.find(iter.first) == added.end())
        {
            _materials.push_back(iter.second);
            added.emplace(iter.first);
        }
    }
}

Config
AssetCatalog::getConfig() const
{
    Config conf("AssetCatalog");

    Config models("Models");
    for (auto& model : _models)
        models.add(model.second.getConfig());
    if (!models.empty())
        conf.add(models);

    return conf;
}

unsigned
AssetCatalog::getLifeMapMatrixWidth() const
{
    return _lifemapMatrixWidth;
}

const ModelAsset*
AssetCatalog::getModel(const std::string& name) const
{
    auto i = _models.find(name);
    return i != _models.end() ? &i->second : nullptr;
}

const MaterialAsset*
AssetCatalog::getMaterial(const std::string& name) const
{
    for (auto& material : _materials)
    {
        if (material.name() == name)
            return &material;
    }
    return nullptr;
}

bool
AssetCatalog::empty() const
{
    return _models.empty() && _materials.empty();
}

//...................................................................

ResidentModelAsset::Ptr
ResidentModelAsset::create()
{
    return Ptr(new ResidentModelAsset());
}

ResidentModelAsset::ResidentModelAsset() :
    _assetDef(nullptr)
{
    //nop
}

//...................................................................

namespace
{
    osg::Image* convertNormalMapFromRGBToRG(const osg::Image* in)
    {
        OE_SOFT_ASSERT_AND_RETURN(in != nullptr, nullptr);

        osg::Image* out = new osg::Image();
        out->allocateImage(in->s(), in->t(), 1, GL_RG, GL_UNSIGNED_BYTE);
        out->setInternalTextureFormat(GL_RG8);
        osg::Vec4 v;
        osg::Vec4 packed;
        ImageUtils::PixelReader read(in);
        ImageUtils::PixelWriter write(out);
        read.forEachPixel([&](auto& i)
            {
                read(v, i);
                osg::Vec3 normal(v.r()*2.0f - 1.0f, v.g()*2.0f - 1.0f, v.b()*2.0f - 1.0f);
                NormalMapGenerator::pack(normal, packed);
                write(packed, i);
            }
        );

        return out;
    }

    // scan the image for the row with the most alpha.
    float computeTopBillboardPosition(const osg::Image* in)
    {
        if (!ImageUtils::PixelReader::supports(in))
            return -1.0f;

        int max_offset = 0;
        int best_t = 0;

        ImageUtils::PixelReader read(in);
        read.setBilinear(false);
        osg::Vec4 pixel;

        for (int t = 0; t < in->t(); ++t)
        {
            int row_max_offset = 0;
            for (int s = 0; s < in->s(); ++s)
            {
                read(pixel, s, t);
                if (pixel.a() > 0.15f)
                {
                    int offset = abs((int)s - (int)in->s() / 2);
                    if (offset > row_max_offset)
                        row_max_offset = offset;
                }
            }
            if (row_max_offset > max_offset)
            {
                max_offset = row_max_offset;
                best_t = t;
            }
        }

        return (float)best_t / (float)(in->t() - 1);
    }
}

//...................................................................

ModelAssetMaterializer::ModelAssetMaterializer(ModelAssetManager& manager) :
    _manager(manager),
    _factory(manager.getTextures())
{
    _factory.setGetOrCreateFunction(ChonkFactory::getWeakTextureCacheFunction(
        manager._texturesCache, manager._texturesCacheMutex));

    _getNormalMapFileName = MaterialUtils::getDefaultNormalMapNameMangler();
    _materialLoader.setMangler(NORMAL_MAP_TEX_UNIT, _getNormalMapFileName);

    _getPBRMapFileName = MaterialUtils::getDefaultPBRMapNameMangler();
    _materialLoader.setMangler(PBR_TEX_UNIT, _getPBRMapFileName);
}

void
ModelAssetMaterializer::materialize(
    const ModelAssetRefs& assetRefs,
    ResidentModelAssetInstances& outputInstances,
    const osgDB::Options* readOptions)
{
    // The group points to multiple assets, which we will analyze and load.
    for (auto& asset_ptr : assetRefs)
    {
        const ModelAsset* assetDef = asset_ptr->asset();

        OE_SOFT_ASSERT(assetDef != nullptr);
        if (assetDef == nullptr)
            continue;

        // Look up this model asset. If it's already in the resident set,
        // do nothing; otherwise make it resident by loading all the data.
        ResidentModelAsset::Ptr& residentAsset = _manager._residentModelAssets[assetDef->name()];

        // First reference to this instance? Populate it:
        if (residentAsset == nullptr)
        {
            OE_TEST << LC << "  Loading asset " << assetDef->name() << std::endl;

            residentAsset = ResidentModelAsset::create();

            residentAsset->assetDef() = assetDef;

            if (assetDef->modelURI().isSet())
            {
                const URI& uri = assetDef->modelURI().get();

                auto cache_iter = _modelcache.find(uri);
                if (cache_iter != _modelcache.end())
                {
                    residentAsset->model() = cache_iter->second._node.get();
                    residentAsset->boundingBox() = cache_iter->second._modelAABB;
                }
                else
                {
                    residentAsset->model() = uri.getNode(readOptions);

                    if (residentAsset->model().valid())
                    {
                        // apply a static scale:
                        if (assetDef->scale().isSet())
                        {
                            osg::MatrixTransform* mt = new osg::MatrixTransform();
                            mt->setMatrix(osg::Matrix::scale(assetDef->scale().get(), assetDef->scale().get(), assetDef->scale().get()));
                            mt->addChild(residentAsset->model().get());
                            residentAsset->model() = mt;
                        }

                        // find materials:
                        _materialLoader.setReferrer(uri.full());
                        residentAsset->model()->accept(_materialLoader);

                        // add flexors:
                        bool isUndergrowth = assetDef->group() == "undergrowth";
                        _manager.addFlexors(residentAsset->model(), assetDef->stiffness().get(), isUndergrowth);

                        _modelcache[uri]._node = residentAsset->model().get();

                        osg::ComputeBoundsVisitor cbv;
                        residentAsset->model()->accept(cbv);
                        residentAsset->boundingBox() = cbv.getBoundingBox();
                        _modelcache[uri]._modelAABB = residentAsset->boundingBox();

                        OE_TEST << LC << "Loaded model: " << uri.base() <<
                            " with bbox " << residentAsset->boundingBox().xMin() << " "
                            << residentAsset->boundingBox().yMin() << " "
                            << residentAsset->boundingBox().xMax() << " "
                            << residentAsset->boundingBox().yMax() <<
                            std::endl;
                    }
                    else
                    {
                        OE_WARN << LC << "Failed to load model " << uri.full() << std::endl;
                    }
                }
            }

            // If the width is expressly set (height is optional) we will use it to override any
            // bounding box computed by the model.
            if (assetDef->width().isSet() || !residentAsset->boundingBox().valid())
            {
                double width = assetDef->width().get();

                double height =
                    assetDef->height().isSet() ? assetDef->height().get() :
                    residentAsset->boundingBox().valid() ? residentAsset->boundingBox().zMax() :
                    assetDef->height().get();

                residentAsset->boundingBox().set(-width, -width, 0.0, width, width, height);
            }

            URI sideBB;
            if (assetDef->sideBillboardURI().isSet())
                sideBB = assetDef->sideBillboardURI().get();
            else if (assetDef->modelURI().isSet())
                sideBB = URI(assetDef->modelURI()->full() + ".side.png", assetDef->modelURI()->context());

            float impostorFarLODScale = 1.0f;

            if (!sideBB.empty())
            {
                const URI& uri = sideBB;

                auto ic = _texcache.find(uri);
                if (ic != _texcache.end())
                {
                    residentAsset->sideBillboardTex() = ic->second->sideBillboardTex();
                    residentAsset->sideBillboardNormalMap() = ic->second->sideBillboardNormalMap();
                    residentAsset->sideBillboardPBRMap() = ic->second->sideBillboardPBRMap();
                }
                else
                {
                    osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
                    if (image.valid())
                    {
                        residentAsset->sideBillboardTex() = new osg::Texture2D(image.get());

                        OE_TEST << LC << "Loaded side BB: " << uri.base() << std::endl;
                        _texcache[uri] = residentAsset;

                        // normal map:
                        URI normalMapURI(_getNormalMapFileName(uri.full()));
                        osg::ref_ptr<osg::Image> normalMap = normalMapURI.getImage(readOptions);
                        if (normalMap.valid())
                        {
                            OE_TEST << LC << "Loaded NML: " << normalMapURI.base() << std::endl;
                            residentAsset->sideBillboardNormalMap() = new osg::Texture2D(normalMap.get());
                        }
                        else
                        {
                            OE_INFO << LC << "Failed to load: " << normalMapURI.base() << std::endl;
                        }

                        URI pbrMapURI(_getPBRMapFileName(uri.full()));
                        osg::ref_ptr<osg::Image> pbrMap = pbrMapURI.getImage(readOptions);
                        if (pbrMap.valid())
                        {
                            OE_TEST << LC << "Loaded PBR: " << pbrMapURI.base() << std::endl;
                            residentAsset->sideBillboardPBRMap() = new osg::Texture2D(pbrMap);
                        }
                        else
                        {
                            OE_INFO << LC << "Failed to load: " << pbrMapURI.base() << std::endl;
                        }
                    }
                    else
                    {
                        OE_WARN << LC << "Failed to load side billboard " << uri.full() << std::endl;
                    }
                }

                URI topBB;
                if (assetDef->topBillboardURI().isSet())
                    topBB = assetDef->topBillboardURI().get();
                else if (assetDef->modelURI().isSet())
                    topBB = URI(assetDef->modelURI()->full() + ".top.png", assetDef->modelURI()->context());

                if (!topBB.empty())
                {
                    const URI& uri = topBB;

                    auto ic = _texcache.find(uri);
                    if (ic != _texcache.end())
                    {
                        residentAsset->topBillboardTex() = ic->second->topBillboardTex();
                        residentAsset->topBillboardNormalMap() = ic->second->topBillboardNormalMap();
                        residentAsset->topBillboardPBRMap() = ic->second->topBillboardPBRMap();
                    }
                    else
                    {
                        osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
                        if (image.valid())
                        {
                            residentAsset->topBillboardTex() = new osg::Texture2D(image.get());

                            OE_TEST << LC << "Loaded top BB: " << uri.base() << std::endl;
                            _texcache[uri] = residentAsset;

                            // normal map:
                            URI normalMapURI(_getNormalMapFileName(uri.full()));
                            osg::ref_ptr<osg::Image> normalMap = normalMapURI.getImage(readOptions);
                            if (normalMap.valid())
                            {
                                OE_TEST << LC << "Loaded NML: " << normalMapURI.base() << std::endl;
                                residentAsset->topBillboardNormalMap() = new osg::Texture2D(normalMap.get());
                            }
                            else
                            {
                                OE_INFO << LC << "Failed to load: " << normalMapURI.base() << std::endl;
                            }

                            // PBR map:
                            URI pbrMapURI(_getPBRMapFileName(uri.full()));
                            osg::ref_ptr<osg::Image> pbrMap = pbrMapURI.getImage(readOptions);
                            if (pbrMap.valid())
                            {
                                OE_TEST << LC << "Loaded PBR: " << pbrMapURI.base() << std::endl;
                                residentAsset->topBillboardPBRMap() = new osg::Texture2D(pbrMap);
                            }
                            else
                            {
                                OE_INFO << LC << "Failed to load: " << pbrMapURI.base() << std::endl;
                            }
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to load top billboard " << uri.full() << std::endl;
                        }
                    }
                }

                std::vector<osg::Texture*> textures(6);
                textures[0] = residentAsset->sideBillboardTex().get();
                textures[1] = residentAsset->sideBillboardNormalMap().get();
                textures[2] = residentAsset->sideBillboardPBRMap().get();
                textures[3] = residentAsset->topBillboardTex().get();
                textures[4] = residentAsset->topBillboardNormalMap().get();
                textures[5] = residentAsset->topBillboardPBRMap().get();

                // if this group has an impostor creation function, call it
                auto iter = _manager._createImpostorFunctions.find(assetDef->group());
                if (iter != _manager._createImpostorFunctions.end())
                {
                    auto& createImpostor = iter->second;

                    if (createImpostor != nullptr)
                    {
                        auto& bbox = residentAsset->boundingBox();

                        float top_z = 0.33f * (bbox.zMax() - bbox.zMin());

                        if (residentAsset->assetDef()->topBillboardHeight().isSet())
                        {
                            top_z = residentAsset->assetDef()->topBillboardHeight().value();
                        }
                        else if (residentAsset->sideBillboardTex().valid())
                        {
                            float v = computeTopBillboardPosition(residentAsset->sideBillboardTex()->getImage(0));
                            if (v >= 0.0f)
                            {
                                top_z = v * (bbox.zMax() - bbox.zMin());
                            }
                        }

                        auto imp = createImpostor(bbox, top_z, textures);

                        // post-process if there's a post-URI callback.
                        // This is so the Impostor can have extended materials injected.
                        auto* post = URIPostReadCallback::from(readOptions);
                        if (post)
                        {
                            ReadResult result(imp._node);
                            (*post)(result);
                        }

                        residentAsset->impostor() = imp._node;
                        impostorFarLODScale = imp._farLODScale;

                        // if no MAIN model exists, just re-use the impostor as the 
                        // main model!
                        if (!residentAsset->model().valid())
                        {
                            residentAsset->model() = residentAsset->impostor().get();
                        }
                    }
                }
            }

            // Finally, chonkify.
            if (residentAsset->model().valid())
            {
                // Replace impostor with 3D model "N" times closer than the SSE:
                float far_pixel_scale = _manager.getLODTransitionPixelScale();

                // Real model can get as close as it wants:
                float near_pixel_scale = FLT_MAX;

                if (residentAsset->chonk() == nullptr)
                {
                    residentAsset->chonk() = Chonk::create();
                    residentAsset->chonk()->name() = residentAsset->assetDef()->name();
                }

#if 0
                // FOR DEBUGGING - ADD A CHONK THAT VISUALIZES THE NORMALS
                osg::ref_ptr<osg::Group> debuggroup = new osg::Group();
                debuggroup->addChild(residentAsset->model());
                debuggroup->addChild(makeDebugModel(residentAsset->model().get()));

                if (residentAsset->chonk() == nullptr)
                    residentAsset->chonk() = Chonk::create();

                residentAsset->chonk()->add(
                    debuggroup,
                    getLODTransitionPixelScale(),
                    FLT_MAX,
                    factory);

#else

                residentAsset->chonk()->add(
                    residentAsset->model().get(),
                    far_pixel_scale,
                    near_pixel_scale,
                    _factory);
#endif
            }

            if (residentAsset->impostor().valid())
            {
                // Fade out the impostor at this pixel scale (i.e., multiple of SSE)
                float far_pixel_scale = impostorFarLODScale;

                // Fade the impostor to the 3D model at this pixel scale:
                float near_pixel_scale = residentAsset->model().valid() ?
                    _manager.getLODTransitionPixelScale() :
                    FLT_MAX;

                if (residentAsset->chonk() == nullptr)
                {
                    residentAsset->chonk() = Chonk::create();
                    residentAsset->chonk()->name() = residentAsset->assetDef()->name();
                }

                residentAsset->chonk()->add(
                    residentAsset->impostor().get(),
                    far_pixel_scale,
                    near_pixel_scale,
                    _factory);
            }
        }

        // If this data successfully materialized, add it to the
        // biome's instance collection.
        if (residentAsset->sideBillboardTex().valid() || residentAsset->model().valid())
        {
            ResidentModelAssetInstance instance;
            instance.residentAsset() = residentAsset;
            instance.weight() = asset_ptr->weight();
            instance.coverage() = asset_ptr->coverage();
            outputInstances.emplace_back(std::move(instance));
        }
    }
}

//...................................................................

ModelAssetManager::ModelAssetManager()
{
    // this arena will hold all the textures for loaded assets.
    _textures = new TextureArena();
    _textures->setAutoRelease(true);
    _textures->setName("ModelAssets");
    _textures->setBindingPoint(1);
}

void
ModelAssetManager::setLODTransitionPixelScale(float value)
{
    _lodTransitionPixelScale = value;

    // We need to rebuild the assets, so clear everything out,
    // recalculate the biome set, and bump the revision.
    std::lock_guard<std::mutex> lock(_residentData_mutex);

    _residentModelAssets.clear();
    ++revision;
}

float
ModelAssetManager::getLODTransitionPixelScale() const
{
    return _lodTransitionPixelScale;
}

void
ModelAssetManager::recalculateResidentAssets()
{
    std::lock_guard<std::mutex> lock(_residentData_mutex);

    std::list<std::string> to_delete;

    for (auto& entry : _residentModelAssets)
    {
        if (entry.second.use_count() == 1)
        {
            to_delete.push_back(entry.first);
        }
    }

    for (auto& asset_name : to_delete)
    {
        _residentModelAssets.erase(asset_name);
        OE_TEST << LC << "Unloaded asset " << asset_name << std::endl;
    }
}

std::vector<const ModelAsset*>
ModelAssetManager::getResidentAssets() const
{
    std::vector<const ModelAsset*> result;

    std::lock_guard<std::mutex> lock(_residentData_mutex);

    result.reserve(_residentModelAssets.size());

    for (auto& entry : _residentModelAssets)
    {
        result.push_back(entry.second->assetDef());
    }

    return result;
}

std::vector<const ModelAsset*>
ModelAssetManager::getResidentAssetsIfNotLocked() const
{
    std::vector<const ModelAsset*> result;
    if (_residentData_mutex.try_lock())
    {
        result.reserve(_residentModelAssets.size());
        for (auto& entry : _residentModelAssets)
        {
            result.push_back(entry.second->assetDef());
        }
        _residentData_mutex.unlock();
    }
    return result;
}

void
ModelAssetManager::setCreateImpostorFunction(
    const std::string& group,
    CreateImpostorFunction func)
{
    std::lock_guard<std::mutex> lock(_residentData_mutex);
    _createImpostorFunctions[group] = func;
}

//...................................................................


namespace
{
    osg::Geometry* makeDebugModel(osg::Node* node)
    {
        auto geom = new osg::Geometry();

        float radius = node->getBound().radius();

        auto out_colors = new osg::Vec4Array(osg::Array::BIND_OVERALL);
        out_colors->push_back(osg::Vec4(1, 0, 0, 1));
        geom->setColorArray(out_colors);

        auto out_verts = new osg::Vec3Array();
        geom->setVertexArray(out_verts);

        auto out_indices = new osg::DrawElementsUShort(GL_TRIANGLES);
        geom->addPrimitiveSet(out_indices);

        auto geom_functor = [&out_verts, &out_indices, radius](
            osg::Geometry& geom,
            const osg::Matrix& local2model)
        {
            auto verts = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
            OE_SOFT_ASSERT_AND_RETURN(verts, void(), "NO VERTS");

            auto normals = dynamic_cast<osg::Vec3Array*>(geom.getNormalArray());
            OE_SOFT_ASSERT_AND_RETURN(normals, void(), "NO NORMALS");

            const float x = radius * 0.005f;
            const float z = x * 7.5f;

            for (int i = 0; i < verts->size(); ++i)
            {
                osg::Vec3 vert = (*verts)[i] * local2model;
                osg::Vec3 normal = osg::Matrix::transform3x3((*normals)[i], local2model);

                osg::Quat zupToNormal;
                zupToNormal.makeRotate(osg::Vec3(0, 0, 1), normal);

                int start = out_verts->size();

                out_verts->push_back(vert + zupToNormal * (osg::Vec3(-x, -x, 0)));
                out_verts->push_back(vert + zupToNormal * (osg::Vec3(+x, +x, 0)));
                out_verts->push_back(vert + zupToNormal * (osg::Vec3(-x, +x, 0)));
                out_verts->push_back(vert + zupToNormal * (osg::Vec3(+x, -x, 0)));
                out_verts->push_back(vert + zupToNormal * (osg::Vec3(0, 0, z)));

                out_indices->push_back(start + 0);
                out_indices->push_back(start + 1);
                out_indices->push_back(start + 4);
                out_indices->push_back(start + 2);
                out_indices->push_back(start + 3);
                out_indices->push_back(start + 4);
            }
        };

        TypedNodeVisitor<osg::Geometry> visitor(geom_functor);
        node->accept(visitor);
        return geom;
    }
}

BiomeManager::ResidentBiomesById
BiomeManager::getResidentBiomes(const osgDB::Options* readOptions)
{
    OE_PROFILING_ZONE;

    // First refresh the resident biome collection based on current refcounts
    recalculateResidentBiomes();

    // Next go through and load any assets that are not yet loaded
    materializeNewAssets(readOptions);

    // Make a copy:
    ResidentBiomesById result = _residentBiomes;

    return std::move(result);
}

void
BiomeManager::materializeNewAssets(const osgDB::Options* readOptions)
{
    OE_PROFILING_ZONE;

    // exclusive access to the resident dataset
    std::lock_guard<std::mutex> lock(_mutex);

    ModelAssetMaterializer materializer(*_assetManager);

    // Go through the residency list and materialize any model assets
    // that are not already loaded (and present in _residentModelAssets);
    // Along the way, build the instances for each biome.
    for (auto& iter : _residentBiomes)
    {
        auto& residentBiome = iter.second;

        // clear the list and start fresh:
        residentBiome.instances.clear();
        
        // load the assets referened by this biome:
        materializer.materialize(
            residentBiome.biome->getModelAssets(),
            residentBiome.instances,
            readOptions);
    }
}

void
ModelAssetManager::addFlexors(osg::ref_ptr<osg::Node>& node, float stiffness, bool isUndergrowth)
{
    OE_SOFT_ASSERT_AND_RETURN(node.valid(), void(), "Invalid node");
    // note: stiffness is [0..1].

    osg::ComputeBoundsVisitor cb;
    node->accept(cb);
    auto& bbox = cb.getBoundingBox();

    TypedNodeVisitor<osg::Geometry>::Function func;

    if (isUndergrowth)
    {
        // for undergrowth, create flexors that point skyward. 
        func = [&bbox](osg::Geometry& geom, const osg::Matrix& local2model)
        {
            auto verts = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
            OE_SOFT_ASSERT_AND_RETURN(verts, void());

            // make a flexor array on demand
            osg::Vec3Array* flexors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, verts->size());
            geom.setTexCoordArray(3, flexors);

            const osg::Vec3 zaxis(0, 0, 1);
            osg::Matrix model2local;
            model2local.invert(local2model);

            for (int i = 0; i < verts->size(); ++i)
            {
                osg::Vec3 vert_model = (*verts)[i] * local2model;

                // back to local space
                osg::Vec3 flexor = osg::Matrix::transform3x3(zaxis, model2local);
                (*flexors)[i] = flexor * accel(vert_model.z() / bbox.zMax());
            }

            // apply the "ZAXIS" shading technique
            auto tech = new osg::UByteArray(1, { Chonk::NORMAL_TECHNIQUE_ZAXIS });
            tech->setBinding(osg::Array::BIND_OVERALL);
            geom.setVertexAttribArray(6, tech);
        };
    }

    else // not undergrorwth (normal tree, bush models)
    {
        func = [&bbox, stiffness](osg::Geometry& geom, const osg::Matrix& l2w)
        {
            osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
            if (!verts) return;

            osg::Vec3Array* flexors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
            flexors->reserve(verts->size());
            geom.setTexCoordArray(3, flexors);

            osg::Vec3f base(bbox.center().x(), bbox.center().y(), bbox.zMin());
            float xy_radius = std::max(bbox.xMax() - bbox.xMin(), bbox.yMax() - bbox.yMin());

            osg::Matrix w2l;
            w2l.invert(l2w);

            for (auto& local_vert : *verts)
            {
                auto vert = local_vert * l2w;
                osg::Vec3f anchor(0.0f, 0.0f, vert.z());
                float height_ratio = harden((vert.z() - bbox.zMin()) / (bbox.zMax() - bbox.zMin()));
                auto lateral_vec = (vert - anchor);
                float lateral_ratio = harden(lateral_vec.length() / xy_radius);
                auto flex =
                    mix(normalize(lateral_vec), normalize(vert), height_ratio) *
                    (lateral_ratio * 3.0f) *
                    (height_ratio * 1.5f) *
                    (1.0 - stiffness);

                flex = osg::Matrix::transform3x3(flex, w2l);
                flexors->push_back(flex);
            }
        };
    }

    TypedNodeVisitor<osg::Geometry> visitor(func);
    node->accept(visitor);
}
