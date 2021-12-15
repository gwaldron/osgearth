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
#include <osgEarth/MaterialLoader>
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#undef LC
#define LC "[BiomeManager] "

#define NORMAL_MAP_TEX_UNIT 1

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
        osg::Image* out = new osg::Image();
        out->allocateImage(in->s(), in->t(), 1, GL_RG, GL_UNSIGNED_BYTE);
        out->setInternalTextureFormat(GL_RG8);
        osg::Vec4 v;
        osg::Vec4 packed;
        ImageUtils::PixelReader read(in);
        ImageUtils::PixelWriter write(out);

        ImageUtils::ImageIterator i(read);
        i.forEachPixel([&]()
            {
                read(v, i.s(), i.t());
                osg::Vec3 normal(v.r()*2.0f - 1.0f, v.g()*2.0f - 1.0f, v.b()*2.0f - 1.0f);
                NormalMapGenerator::pack(normal, packed);
                write(packed, i.s(), i.t());
            }
        );

        return out;
    }
}

//...................................................................

BiomeManager::BiomeManager() :
    _revision(0),
    _refsAndRevision_mutex("BiomeManager.refsAndRevision(OE)"),
    _residentData_mutex("BiomeManager.residentData(OE)")
{
    //nop
}

void
BiomeManager::ref(const Biome* biome)
{
    ScopedMutexLock lock(_refsAndRevision_mutex);

    auto item = _refs.emplace(biome, 0);
    ++item.first->second;
    if (item.first->second == 1) // ref count of 1 means it's new
    {
        ++_revision;
        OE_INFO << LC << "Hello, " << biome->name().get() << std::endl;
    }
}

void
BiomeManager::unref(const Biome* biome)
{
    ScopedMutexLock lock(_refsAndRevision_mutex);

    auto iter = _refs.find(biome);
    
    // silent assertion
    if (iter == _refs.end() || iter->second == 0)
        return;

    --iter->second;
    if (iter->second == 0)
    {
        ++_revision;
        OE_INFO << LC << "Goodbye, " << biome->name().get() << std::endl;
    }
}

int
BiomeManager::getRevision() const
{
    return _revision;
}

void
BiomeManager::reset()
{
    // reset the reference counts, and bump the revision so the
    // next call to update will remove any resident data
    {
        ScopedMutexLock lock(_refsAndRevision_mutex);

        for (auto& iter : _refs)
        {
            const Biome* biome = iter.first;
            OE_INFO << LC << "Goodbye, " << biome->name().get() << std::endl;
            iter.second = 0;
        }

        ++_revision;
    }

    // Resolve the references and unload any resident assets from memory.
    refresh();
}

void
BiomeManager::refresh()
{
    std::vector<const Biome*> biomes_to_add;
    std::vector<const Biome*> biomes_to_remove;

    // Figure out which biomes we need to load and which we can discard.
    {
        ScopedMutexLock lock(_refsAndRevision_mutex);
        
        for (auto& ref : _refs)
        {
            const Biome* biome = ref.first;
            int refcount = ref.second;

            if (refcount > 0)
            {
                biomes_to_add.push_back(biome);
            }
            else
            {
                biomes_to_remove.push_back(biome);
            }
        }
    }

    // Update the resident biome data structure:
    {
        ScopedMutexLock lock(_residentData_mutex);

        // add biomes that might need adding
        for (auto biome : biomes_to_add)
        {
            auto& dummy = _residentBiomeData[biome];
        }

        // get rid of biomes we no longer need.
        for (auto biome : biomes_to_remove)
        {
            _residentBiomeData.erase(biome);
        }


        // finally, update the collection of resident assets to
        // reflect the reference counts.
        std::list<const ModelAsset*> to_delete;

        for (auto& entry : _residentModelAssets)
        {
            if (entry.second.use_count() == 1)
            {
                to_delete.push_back(entry.first);
            }
        }

        for (auto& asset : to_delete)
        {
            _residentModelAssets.erase(asset);
            OE_DEBUG << LC << "Unloaded asset " << asset->name().get() << std::endl;
        }
    }
}

std::vector<const Biome*>
BiomeManager::getActiveBiomes() const
{
    ScopedMutexLock lock(_refsAndRevision_mutex);

    std::vector<const Biome*> result;

    for (auto& ref : _refs)
    {
        if (ref.second > 0)
            result.push_back(ref.first);
    }
    return std::move(result);
}

std::vector<const ModelAsset*>
BiomeManager::getResidentAssets() const
{
    ScopedMutexLock lock(_residentData_mutex);

    std::vector<const ModelAsset*> result;
    result.reserve(_residentModelAssets.size());
    for (auto& entry : _residentModelAssets)
        result.push_back(entry.first);
    return std::move(result);
}

namespace
{
    struct ModelCacheEntry
    {
        osg::ref_ptr<osg::Node> _node;
        osg::BoundingBox _modelAABB;
    };
}

GeometryCloudCollection
BiomeManager::updateResidency(
    CreateImposterFunction createImposter,
    const osgDB::Options* readOptions)
{
    // First refresh the resident biome collection based on current refcounts.
    refresh();

    // exclusive access to the resident dataset
    ScopedMutexLock lock(_residentData_mutex);

    std::set<AssetGroup::Type> asset_groups;
    
    // Any billboard that doesn't have its own normal map will use this one.
    osg::ref_ptr<osg::Texture> defaultNormalMap = osgEarth::createEmptyNormalMapTexture();

    // Some caches to avoid duplicating data
    using TextureShareCache = std::map<URI, ResidentModelAsset::Ptr>;
    TextureShareCache texcache;
    using ModelCache = std::map<URI, ModelCacheEntry>;
    ModelCache modelcache;

    // Clear out each biome's instances so we can start fresh.
    // This is a low-cost operation since anything we can re-use
    // will already by in the _residentModelAssetData collection.
    OE_DEBUG << LC << "Found " << _residentBiomeData.size() << " resident biomes..." << std::endl;
    for (auto& iter : _residentBiomeData)
    {
        auto& groups = iter.second;
        for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
            groups[i].clear();
    }

    // This loader will find material textures and install them on
    // secondary texture image units.. in this case, normal maps.
    // We can expand this later to include other types of material maps.
    Util::MaterialLoader materialLoader;

    materialLoader.setMangler(NORMAL_MAP_TEX_UNIT,
        [](const std::string& filename)
        {
            return osgDB::getNameLessExtension(filename) + "_NML.png";
        }
    );

    materialLoader.setTextureFactory(NORMAL_MAP_TEX_UNIT,
        [](osg::Image* image)
        {
            // compresses the incoming texture if necessary
            if (image->getPixelFormat() != GL_RG)
                return new osg::Texture2D(convertNormalMapFromRGBToRG(image));
            else
                return new osg::Texture2D(image);
        }
    );

    // Go through the residency list and materialize any model assets
    // that are not already loaded (and present in _residentModelAssets);
    // Along the way, build the instances for each biome.
    for (auto& e : _residentBiomeData)
    {
        const Biome* biome = e.first;

        for (int group = 0; group < NUM_ASSET_GROUPS; ++group)
        {
            const Biome::ModelAssetsToUse& assetsToUse = biome->getModelAssetsToUse(group);

            // this is the collection of instances we're going to populate
            ResidentModelAssetInstances& assetInstances = e.second[group];

            // The group points to multiple assets, which we will analyze and load.
            for (auto& asset_ptr : assetsToUse)
            {
                const ModelAsset* assetDef = asset_ptr.asset;

                OE_SOFT_ASSERT(assetDef != nullptr);
                if (assetDef == nullptr)
                    continue;

                // Look up this model asset. If it's already in the resident set,
                // do nothing; otherwise make it resident by loading all the data.
                ResidentModelAsset::Ptr& residentAsset = _residentModelAssets[assetDef];

                // First reference to this instance? Populate it:
                if (residentAsset == nullptr)
                {
                    OE_INFO << LC << "  Loading asset " << assetDef->name().get() << std::endl;

                    residentAsset = ResidentModelAsset::create();

                    residentAsset->_assetDef = assetDef;

                    if (assetDef->modelURI().isSet())
                    {
                        const URI& uri = assetDef->modelURI().get();
                        ModelCache::iterator ic = modelcache.find(uri);
                        if (ic != modelcache.end())
                        {
                            residentAsset->_model = ic->second._node.get();
                            residentAsset->_modelAABB = ic->second._modelAABB;
                        }
                        else
                        {
                            residentAsset->_model = uri.getNode(readOptions);
                            if (residentAsset->_model.valid())
                            {
                                // find materials:
                                residentAsset->_model->accept(materialLoader);

                                OE_DEBUG << LC << "Loaded model: " << uri.base() << std::endl;
                                modelcache[uri]._node = residentAsset->_model.get();

                                osg::ComputeBoundsVisitor cbv;
                                residentAsset->_model->accept(cbv);
                                residentAsset->_modelAABB = cbv.getBoundingBox();
                                modelcache[uri]._modelAABB = residentAsset->_modelAABB;
                            }
                            else
                            {
                                OE_WARN << LC << "Failed to load model " << uri.full() << std::endl;
                            }
                        }
                    }

                    if (assetDef->sideBillboardURI().isSet())
                    {
                        const URI& uri = assetDef->sideBillboardURI().get();

                        auto ic = texcache.find(uri);
                        if (ic != texcache.end())
                        {
                            residentAsset->_sideBillboardTex = ic->second->_sideBillboardTex;
                            residentAsset->_sideBillboardNormalMap = ic->second->_sideBillboardNormalMap;
                        }
                        else
                        {
                            osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
                            if (image.valid())
                            {
                                residentAsset->_sideBillboardTex = new osg::Texture2D(image.get());

                                OE_DEBUG << LC << "Loaded BB: " << uri.base() << std::endl;
                                texcache[uri] = residentAsset;

                                // normal map is the same file name but with _NML inserted before the extension
                                URI normalMapURI(
                                    osgDB::getNameLessExtension(uri.full()) +
                                    "_NML." +
                                    osgDB::getFileExtension(uri.full()));

                                // silenty fail if no normal map found.
                                osg::ref_ptr<osg::Image> normalMap = normalMapURI.getImage(readOptions);
                                if (normalMap.valid())
                                {
                                    residentAsset->_sideBillboardNormalMap = new osg::Texture2D(normalMap.get());
                                }
                                else
                                {
                                    residentAsset->_sideBillboardNormalMap = defaultNormalMap;
                                }
                            }
                            else
                            {
                                OE_WARN << LC << "Failed to load side billboard " << uri.full() << std::endl;
                            }
                        }


                        if (assetDef->topBillboardURI().isSet())
                        {
                            const URI& uri = assetDef->topBillboardURI().get();

                            auto ic = texcache.find(uri);
                            if (ic != texcache.end())
                            {
                                residentAsset->_topBillboardTex = ic->second->_topBillboardTex;
                                residentAsset->_topBillboardNormalMap = ic->second->_topBillboardNormalMap;
                            }
                            else
                            {
                                osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
                                if (image.valid())
                                {
                                    residentAsset->_topBillboardTex = new osg::Texture2D(image.get());

                                    OE_DEBUG << LC << "Loaded BB: " << uri.base() << std::endl;
                                    texcache[uri] = residentAsset;

                                    // normal map is the same file name but with _NML inserted before the extension
                                    URI normalMapURI(
                                        osgDB::getNameLessExtension(uri.full()) +
                                        "_NML." +
                                        osgDB::getFileExtension(uri.full()));

                                    // silenty fail if no normal map found.
                                    osg::ref_ptr<osg::Image> normalMap = normalMapURI.getImage(readOptions);
                                    if (normalMap.valid())
                                    {
                                        residentAsset->_topBillboardNormalMap = new osg::Texture2D(normalMap.get());
                                    }
                                    else
                                    {
                                        residentAsset->_topBillboardNormalMap = defaultNormalMap;
                                    }
                                }
                                else
                                {
                                    OE_WARN << LC << "Failed to load top billboard " << uri.full() << std::endl;
                                }
                            }
                        }

                        std::vector<osg::Texture*> textures(4);
                        textures[0] = residentAsset->_sideBillboardTex.get();
                        textures[1] = residentAsset->_sideBillboardNormalMap.get();
                        textures[2] = residentAsset->_topBillboardTex.get();
                        textures[3] = residentAsset->_topBillboardNormalMap.get();

                        residentAsset->_billboard = createImposter((AssetGroup::Type)group, textures);
                    }
                }

                // If this data successfully materialized, add it to the
                // biome's instance collection.
                if (residentAsset->_sideBillboardTex.valid() || residentAsset->_model.valid())
                {
                    ResidentModelAssetInstance instance;
                    instance._residentAsset = residentAsset;
                    instance._weight = asset_ptr.weight;
                    instance._fill = asset_ptr.fill;
                    assetInstances.push_back(std::move(instance));
                }
            }

            if (!assetInstances.empty())
            {
                asset_groups.insert((AssetGroup::Type)group);
            }
        }
    }

    // Now assemble the new geometry clouds based on the updated 
    // resident biome information.
    GeometryCloudCollection clouds;

    for (auto group : asset_groups)
    {
        osg::ref_ptr<GeometryCloud> cloud = createGeometryCloud(
            group,
            _residentBiomeData);

        if (cloud.valid())
        {
            clouds[group] = cloud;
        }
    }

    return std::move(clouds);
}

namespace
{
    unsigned getNumVertices(osg::Node* node)
    {
        unsigned count = 0u;
        osg::Geometry* geom = node->asGeometry();
        if (geom) {
            for (unsigned i = 0; i < geom->getNumPrimitiveSets(); ++i)
                count += geom->getVertexArray()->getNumElements();
        }
        else {
            osg::Group* group = node->asGroup();
            if (group) {
                for (unsigned i = 0; i < group->getNumChildren(); ++i)
                    count += getNumVertices(group->getChild(i));
            }
        }
        return count;
    }
}

GeometryCloud*
BiomeManager::createGeometryCloud(
    AssetGroup::Type group,
    const ResidentBiomes& residentBiomes) const
{
    TextureArena* arena = new TextureArena();
    arena->setName(AssetGroup::name(group));

    GeometryCloud* cloud = new GeometryCloud(arena);

    // Keep track so we don't add the same model twice
    std::unordered_set<void*> visited;

    CommandIndexMap modelCommands;
    CommandIndexMap imposterCommands;

    // For each resident biome, locate all asset instances that belong
    // to the specified category. Add each to the geometry cloud.
    for (auto& b_iter : residentBiomes)
    {
        // Find the instance list for the requested group,
        // and if it's not empty, carry on:
        auto& instances = b_iter.second[group];

        // for each instance, add it to the cloud as appropriate:
        for (auto& instance : instances)
        {
            auto residentAsset = instance._residentAsset;

            // initialize the commands (insert will fail silently if it already exists)
            modelCommands.insert(std::make_pair(residentAsset, -1));
            imposterCommands.insert(std::make_pair(residentAsset, -1));

            auto model = residentAsset->_model.get();
            if (model && visited.count(model) == 0)
            {
                auto result = cloud->add(
                    model,
                    0U,
                    NORMAL_MAP_TEX_UNIT);

                modelCommands[residentAsset] = result._commandIndex;
                //instance._textures = result._textures;

                visited.insert(model);
            }

            auto imposter = residentAsset->_billboard.get();
            if (imposter && visited.count(imposter) == 0)
            {
                auto result = cloud->add(
                    imposter,
                    getNumVertices(imposter),   // apply alignment so shader can use gl_VertexID
                    NORMAL_MAP_TEX_UNIT);       // normal maps in texture image unit 1

                imposterCommands[residentAsset] = result._commandIndex;
                //instance._textures = result._textures;

                visited.insert(imposter);
            }
        }
    }

    osg::StateSet* stateSet = cloud->getGeometry()->getOrCreateStateSet();

    // attach the texture arena:
    stateSet->setAttribute(
        arena,
        osg::StateAttribute::ON);

    // attach the GPU lookup tables for this cloud:
    osg::StateAttribute* luts = createGPULookupTables(
        group,
        modelCommands,
        imposterCommands,
        residentBiomes);

    stateSet->setAttribute(
        luts, 
        osg::StateAttribute::ON);

    return cloud;
}

osg::StateAttribute*
BiomeManager::createGPULookupTables(
    AssetGroup::Type group,
    const CommandIndexMap& modelCommands,
    const CommandIndexMap& imposterCommands,
    const ResidentBiomes& residentBiomes) const
{
    // Our SSBOs to be reflected on the GPU:
    auto luts = new BiomeManager::BiomeGPUData();

    // SSBO bindings corresponding to the "layout" directives in the shader
    // Consider making these configuration if necessary. These need to match
    // the buffer bindings in GroundCover.Types.glsl.
    luts->biomeLUT().setBindingIndex(6);
    luts->assetLUT().setBindingIndex(7);

    // working cache
    std::unordered_map<
        const ResidentModelAssetInstance*, 
        BiomeManager::AssetLUT_Data> temp;

    // First, caculate the weighting numbers by looping through all
    // biomes and adding up the weights.
    float weightTotal = 0.0f;
    float smallestWeight = FLT_MAX;

    for (const auto& b : residentBiomes)
    {
        const Biome* biome = b.first;
        const ResidentModelAssetInstances& instances = b.second[group];
        for (const auto& instance : instances)
        {
            weightTotal += instance._weight;
            smallestWeight = std::min(smallestWeight, instance._weight);

            auto residentAsset = instance._residentAsset;
            auto assetDef = residentAsset->_assetDef;

            // record an asset:
            float width = assetDef->width().get();
            if (assetDef->width().isSet() == false &&
                residentAsset->_modelAABB.valid())
            {
                width = std::max(
                    residentAsset->_modelAABB.xMax() - residentAsset->_modelAABB.xMin(),
                    residentAsset->_modelAABB.yMax() - residentAsset->_modelAABB.yMin());
            }

            float height = assetDef->height().get();
            if (assetDef->height().isSet() == false &&
                residentAsset->_modelAABB.valid())
            {
                height = residentAsset->_modelAABB.zMax() - residentAsset->_modelAABB.zMin();
            }

            float sizeVariation = assetDef->sizeVariation().get();

            // store in our temporary LUT
            auto& r = temp[&instance];
            r.modelCommand = modelCommands.at(residentAsset);
            r.imposterCommand = imposterCommands.at(residentAsset);
            r.width = width;
            r.height = height;
            r.fill = instance._fill;
            r.sizeVariation = sizeVariation;

            OE_SOFT_ASSERT(r.modelCommand >= 0 || r.imposterCommand >= 0);
        }
    }

    // calculate the weight multiplier
    float weightMultiplier = 1.0f / smallestWeight;
    weightTotal = weightMultiplier * weightTotal;

    // Calcluate the total number of possible slots and allocate them:
    int numElements = 0;
    for (const auto& b : residentBiomes)
    {
        const Biome* biome = b.first;
        const ResidentModelAssetInstances& instances = b.second[group];
        for(const auto& instance : instances)
        {
            numElements += instance._weight*weightMultiplier;
        }
    }
    luts->assetLUT().setNumElements(numElements);

    // Next, build the asset table using weightings.
    // Slot zero (0) will not be used because biome index starts at 1.
    // Reserve it for later use.

    std::map<unsigned, BiomeLUT_Data> biomeLUTData;

    int offset = 0;
    for (const auto& b : residentBiomes)
    {
        const Biome* biome = b.first;

        if (biome->index() <= 0)
        {
            OE_SOFT_ASSERT(biome->index() > 0, "Found a biome index <= 0, skipping");
            continue;
        }

        const ResidentModelAssetInstances& instances = b.second[group];
        int ptr = offset;

        for (const auto& instance : instances)
        {
            int num = (int)(instance._weight*weightMultiplier);

            // copy the asset record into all slots corresponding to
            // this instance. It wastes a bit of memory but no one cares
            for (int w = 0; w < num; ++w)
            {
                auto& r = luts->assetLUT()[ptr++];
                r = temp[&instance];
            }
        }

        // populate the biome LUT:
        BiomeLUT_Data entry;
        entry.offset = offset;
        entry.count = (ptr - offset);
        biomeLUTData[biome->index()] = std::move(entry);

        offset += (ptr - offset);
    }

    // allocate enough space for the highest-numbered biome index
    luts->biomeLUT().setNumElements(
        biomeLUTData.empty() ? 1 : biomeLUTData.rbegin()->first+1);

    for (auto& iter : biomeLUTData)
    {
        luts->biomeLUT()[iter.first] = std::move(iter.second);
    }

    luts->assetLUT().dirty();
    luts->biomeLUT().dirty();

    return luts;
}
