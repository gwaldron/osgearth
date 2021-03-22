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

#include <osgEarth/Elevation>
#include <osgEarth/GLUtils>
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#undef LC
#define LC "[BiomeManager] "

//...................................................................

std::shared_ptr<ModelAssetData>
ModelAssetData::create()
{
    return std::make_shared<ModelAssetData>();
}

ModelAssetData::ModelAssetData() :
    _asset(nullptr),
    _modelCommand(-1),
    _billboardCommand(-1)
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
    _revision(0)
{
    //nop
}

void
BiomeManager::ref(const Biome* biome)
{
    ScopedMutexLock lock(_mutex);

    auto item = _refs.emplace(biome, 0);
    ++item.first->second;
    if (item.second == true) // true == new insertion
        ++_revision;
}

void
BiomeManager::unref(const Biome* biome)
{
    ScopedMutexLock lock(_mutex);

    auto iter = _refs.find(biome);
    OE_SOFT_ASSERT_AND_RETURN(iter != _refs.end(), __func__, );
    OE_SOFT_ASSERT_AND_RETURN(iter->second > 0, __func__, );

    --iter->second;
    if (iter->second == 0)
        ++_revision;
}

int
BiomeManager::getRevision() const
{
    return _revision;
}

void
BiomeManager::update()
{
    ScopedMutexLock lock(_mutex);

    std::vector<const Biome*> biomes_to_remove;

    // Figure out which biomes we need to load and which we can discard.
    for (auto& ref : _refs)
    {
        const Biome* biome = ref.first;
        int refcount = ref.second;

        auto& biomeData = _residentBiomeData[biome];

        if (refcount > 0)
        {
            if (biomeData.empty())
            {
                // need to load the instances for this biome
            }
        }
        else if (refcount == 0)
        {
            biomes_to_remove.push_back(biome);
        }
    }

    // get rid of biomes we no longer need.
    for (auto biome : biomes_to_remove)
    {
        _residentBiomeData.erase(biome);
    }
}

void
BiomeManager::flush()
{
    // scan the resident data list and destroy anything that
    // is unreferened.
    ScopedMutexLock lock(_mutex);

    std::list<const ModelAsset*> to_delete;

    for (auto& entry : _residentModelAssetData)
    {
        if (entry.second.use_count() == 1)
        {
            to_delete.push_back(entry.first);
        }
    }

    for (auto& asset : to_delete)
    {
        OE_INFO << LC << "Unloading model asset " << asset->name().get() << std::endl;
        _residentModelAssetData.erase(asset);
    }
}

namespace
{
    struct ModelCacheEntry
    {
        osg::ref_ptr<osg::Node> _node;
        osg::BoundingBox _modelAABB;
    };
}

void
BiomeManager::loadCategory(
    const std::string& categoryName,
    std::function<osg::Node*(std::vector<osg::Texture*>&)> createImposterGeometry,
    const osgDB::Options* readOptions)
{
    // make sure we're current
    update();

    // exclusive access to the biome instance map
    ScopedMutexLock lock(_mutex);

    // Any billboard that doesn't have its own normal map will use this one.
    osg::ref_ptr<osg::Texture> defaultNormalMap = osgEarth::createEmptyNormalMapTexture();

    // Some caches to avoid duplicating data
    typedef std::map<URI, std::shared_ptr<ModelAssetData> > TextureShareCache;
    TextureShareCache texcache;
    typedef std::map<URI, ModelCacheEntry> ModelCache;
    ModelCache modelcache;

    // Go through the residency list and make resident any model assets
    // that are not already loaded (and present in _residentModelAssetData);
    // Along the way, build the instances for each biome. (An "instance" is
    // a reference to a resident model asset.)
    for (auto& iter : _residentBiomeData)
    {
        const Biome* biome = iter.first;
        CategoryInstanceTable& categories = iter.second;

        // Locate the table containing this category's instance data:
        const ModelCategory* category = biome->getModelCategory(categoryName);        
        if (category == nullptr)
        {
            OE_WARN << LC
                << "Category \"" << categoryName
                << "\" not found in biome \"" << biome->name().get() << "\"" << std::endl;
            continue;
        }
        ModelAssetInstanceCollection& categoryInstances = categories[category];

        // Clear out this biome's instances so we can start fresh.
        // This is a low-cost operation since anything we can re-use
        // will already by in the _residentModelAssetData collection.
        //biomeInstances.clear();
        categoryInstances.clear();

        // The category points to multiple assets, which we will analyze and load.
        for (const auto& member : category->members())
        {
            const ModelAsset* asset = member.asset;

            if (asset == nullptr)
                continue;

            // Look up this model asset. If it's already in the resident set,
            // do nothing; otherwise make it resident by loading all the data.
            ModelAssetDataPtr& data = _residentModelAssetData[asset];

            // First reference to this instance? Populate it:
            if (data == nullptr)
            {
                OE_INFO << LC << "Loading model asset " << asset->name().get() << std::endl;

                data = ModelAssetData::create();

                data->_asset = asset;

                if (asset->modelURI().isSet())
                {
                    const URI& uri = asset->modelURI().get();
                    ModelCache::iterator ic = modelcache.find(uri);
                    if (ic != modelcache.end())
                    {
                        data->_model = ic->second._node.get();
                        data->_modelAABB = ic->second._modelAABB;
                    }
                    else
                    {
                        data->_model = uri.getNode(readOptions);
                        if (data->_model.valid())
                        {
                            OE_INFO << LC << "Loaded model: " << uri.base() << std::endl;
                            modelcache[uri]._node = data->_model.get();

                            osg::ComputeBoundsVisitor cbv;
                            data->_model->accept(cbv);
                            data->_modelAABB = cbv.getBoundingBox();
                            modelcache[uri]._modelAABB = data->_modelAABB;
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to load asset " << uri.full() << std::endl;
                        }
                    }
                }

                if (asset->sideBillboardURI().isSet())
                {
                    const URI& uri = asset->sideBillboardURI().get();

                    auto ic = texcache.find(uri);
                    if (ic != texcache.end())
                    {
                        data->_sideBillboardTex = ic->second->_sideBillboardTex;
                        data->_sideBillboardNormalMap = ic->second->_sideBillboardNormalMap;
                    }
                    else
                    {
                        osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
                        if (image.valid())
                        {
                            data->_sideBillboardTex = new osg::Texture2D(image.get());

                            OE_INFO << LC << "Loaded BB: " << uri.base() << std::endl;
                            texcache[uri] = data;

                            // normal map is the same file name but with _NML inserted before the extension
                            URI normalMapURI(
                                osgDB::getNameLessExtension(uri.full()) +
                                "_NML." +
                                osgDB::getFileExtension(uri.full()));

                            // silenty fail if no normal map found.
                            osg::ref_ptr<osg::Image> normalMap = normalMapURI.getImage(readOptions);
                            if (normalMap.valid())
                            {
                                data->_sideBillboardNormalMap = new osg::Texture2D(normalMap.get());
                            }
                            else
                            {
                                data->_sideBillboardNormalMap = defaultNormalMap;
                            }
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to load asset " << uri.full() << std::endl;
                        }
                    }


                    if (asset->topBillboardURI().isSet())
                    {
                        const URI& uri = asset->topBillboardURI().get();

                        auto ic = texcache.find(uri);
                        if (ic != texcache.end())
                        {
                            data->_topBillboardTex = ic->second->_topBillboardTex;
                            data->_topBillboardNormalMap = ic->second->_topBillboardNormalMap;
                        }
                        else
                        {
                            osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
                            if (image.valid())
                            {
                                data->_topBillboardTex = new osg::Texture2D(image.get());

                                OE_INFO << LC << "Loaded BB: " << uri.base() << std::endl;
                                texcache[uri] = data;

                                // normal map is the same file name but with _NML inserted before the extension
                                URI normalMapURI(
                                    osgDB::getNameLessExtension(uri.full()) +
                                    "_NML." +
                                    osgDB::getFileExtension(uri.full()));

                                // silenty fail if no normal map found.
                                osg::ref_ptr<osg::Image> normalMap = normalMapURI.getImage(readOptions);
                                if (normalMap.valid())
                                {
                                    data->_topBillboardNormalMap = new osg::Texture2D(normalMap.get());
                                }
                                else
                                {
                                    data->_topBillboardNormalMap = defaultNormalMap;
                                }
                            }
                            else
                            {
                                OE_WARN << LC << "Failed to load asset " << uri.full() << std::endl;
                            }
                        }
                    }

                    std::vector<osg::Texture*> textures(4);
                    textures[0] = data->_sideBillboardTex.get();
                    textures[1] = data->_sideBillboardNormalMap.get();
                    textures[2] = data->_topBillboardTex.get();
                    textures[3] = data->_topBillboardNormalMap.get();

                    data->_billboard = createImposterGeometry(textures);
                }
            }

            // If this data successfully materialized, add it to the
            // biome's instance collection.
            if (data->_sideBillboardTex.valid() || data->_model.valid())
            {
                ModelAssetDataInstance instance;
                instance._data = data;
                instance._weight = member.weight;
                categoryInstances.push_back(std::move(instance));
            }
        }
    }
}

namespace
{
    unsigned getNumVertices(osg::Node* node)
    {
        unsigned count = 0u;
        osg::Geometry* geom = node->asGeometry();
        if (geom) {
            for (int i = 0; i < geom->getNumPrimitiveSets(); ++i)
                count += geom->getVertexArray()->getNumElements();
        }
        else {
            osg::Group* group = node->asGroup();
            if (group) {
                for (int i = 0; i < group->getNumChildren(); ++i)
                    count += getNumVertices(group->getChild(i));
            }
        }
        return count;
    }
}

GeometryCloud*
BiomeManager::createGeometryCloud(
    const std::string& category,
    TextureArena* arena) const
{
    GeometryCloud* cloud = new GeometryCloud(arena);

    // Add each 3D model to the geometry cloud and update the texture
    // atlas along the way.
    std::unordered_set<void*> visited;

    for (const auto& b_iter : _residentBiomeData)
    {
        const ModelCategory* cat = b_iter.first->getModelCategory(category);
        if (cat && b_iter.second.count(cat) != 0)
        {
            const auto& catInstances = b_iter.second.at(cat);

            for (const auto& instance : catInstances)
            {
                const auto& data = instance._data;

                auto model = data->_model.get();
                if (model && visited.count(model) == 0)
                {
                    data->_modelCommand = cloud->add(model);
                    visited.insert(model);
                }

                auto imposter = data->_billboard.get();
                if (imposter && visited.count(imposter) == 0)
                {
                    data->_billboardCommand = cloud->add(
                        imposter,
                        getNumVertices(imposter),   // apply alignment so shader can use gl_VertexID
                        1);                         // normal maps in texture image unit 1
                    visited.insert(imposter);
                }
            }
        }
    }

    return cloud;
}

osg::StateAttribute*
BiomeManager::createGPULookupTables(
    const std::string& categoryName) const
{
    // Our SSBOs to be reflected on the GPU:
    auto luts = new BiomeManager::BiomeGPUData();

    // SSBO bindings corresponding to the "layout" directives in the shader
    luts->biomeLUT().setBindingIndex(6);
    luts->assetLUT().setBindingIndex(7);

    // working cache
    std::unordered_map<ModelAssetData*, BiomeManager::AssetLUT_Data> temp;

    // Collect the category for each biome.
    std::unordered_map<const Biome*, const ModelAssetInstanceCollection*> categoryData;
    for (const auto& b : _residentBiomeData)
    {
        const Biome* biome = b.first;
        const ModelCategory* cat = biome->getModelCategory(categoryName);
        auto c_iter = b.second.find(cat);
        if (c_iter != b.second.end())
        {
            categoryData[biome] = &c_iter->second;
        }
    }

    // First, caculate the weighting numbers by looping through all
    // biomes and adding up the weights.
    float weightTotal = 0.0f;
    float smallestWeight = FLT_MAX;

    for (const auto& b : categoryData)
    {
        const Biome* biome = b.first;
        const ModelAssetInstanceCollection& instances = *b.second;

        for (const auto& instance : instances)
        {
            weightTotal += instance._weight;
            smallestWeight = std::min(smallestWeight, instance._weight);

            auto data = instance._data;

            // record an asset:
            float width = data->_asset->width().get();
            if (data->_asset->width().isSet() == false &&
                data->_modelAABB.valid())
            {
                width = std::max(
                    data->_modelAABB.xMax() - data->_modelAABB.xMin(),
                    data->_modelAABB.yMax() - data->_modelAABB.yMin());
            }

            float height = data->_asset->height().get();
            if (data->_asset->height().isSet() == false &&
                data->_modelAABB.valid())
            {
                height = data->_modelAABB.zMax() - data->_modelAABB.zMin();
            }

            float sizeVariation = data->_asset->sizeVariation().get();

            // store in our temporary LUT
            auto& r = temp[data.get()];
            r.modelCommand = data->_modelCommand;
            r.billboardCommand = data->_billboardCommand;
            r.width = width;
            r.height = height;
            r.sizeVariation = sizeVariation;
        }
    }

    // calculate the weight multiplier
    float weightMultiplier = 1.0f / smallestWeight;
    weightTotal = weightMultiplier * weightTotal;

    // Calcluate the total number of possible slots and allocate them:
    int numElements = 0;
    for (const auto& b : categoryData)
    {
        const auto& instances = *b.second;
        for(const auto& instance : instances)
        {
            numElements += instance._weight*weightMultiplier;
        }
    }
    luts->assetLUT().setNumElements(numElements);

    // Next, build the asset table using weightings.
    constexpr int MAX_NUM_BIOMES = 255;
    luts->biomeLUT().setNumElements(MAX_NUM_BIOMES);

    int offset = 0;
    for (const auto& b : categoryData)
    {
        const Biome* biome = b.first;
        const auto& instances = *b.second;

        int biome_id = biome->id().get();

        int ptr = offset;
        for (const auto& instance : instances)
        {
            int num = (int)(instance._weight*weightMultiplier);

            // copy the asset record into all slots corresponding to
            // this instance. It wastes a bit of memory but no one cares
            for (int w = 0; w < num; ++w)
            {
                auto& r = luts->assetLUT()[ptr++];
                r = temp[instance._data.get()];
            }
        }

        // populate the biome LUT:
        luts->biomeLUT()[biome_id].offset = offset;
        luts->biomeLUT()[biome_id].count = (ptr - offset);

        offset += (ptr - offset);
    }
    luts->assetLUT().dirty();
    luts->biomeLUT().dirty();

    return luts;
}
