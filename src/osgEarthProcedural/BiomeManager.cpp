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
#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osg/TriangleIndexFunctor>

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
    _residentData_mutex("BiomeManager.residentData(OE)"),
    _lodTransitionPixelScale(8.0f),
    _locked(false)
{
    // this arena will hold all the textures for loaded assets.
    _textures = new TextureArena();
    _textures->setName("Biomes");
    _textures->setBindingPoint(1);
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
        OE_INFO << LC << "Hello, " << biome->name().get() << " (" << biome->index() << ")" << std::endl;
    }
}

void
BiomeManager::unref(const Biome* biome)
{
    ScopedMutexLock lock(_refsAndRevision_mutex);

    auto iter = _refs.find(biome);
    
    // silent assertion
    if (iter == _refs.end() || iter->second == 0)
    {
        OE_SOFT_ASSERT(iter != _refs.end() && iter->second != 0);
        return;
    }

    if (!_locked)
    {
        --iter->second;
        if (iter->second == 0)
        {
            // Technically yes, this creates a new revision, but it
            // would also result in re-loading assets that are already
            // resident... think on this -gw
            //++_revision;
            OE_INFO << LC << "Goodbye, " << biome->name().get() << "(" << biome->index() << ")" << std::endl;
        }
    }

    //OE_INFO << LC << "(" << iter->second << ")" << biome->name().get() << std::endl;
}

int
BiomeManager::getRevision() const
{
    return _revision;
}

void
BiomeManager::setLODTransitionPixelScale(float value)
{
    _lodTransitionPixelScale = value;

    // We need to rebuild the assets, so clear everything out,
    // recalculate the biome set, and bump the revision.
    _residentData_mutex.lock();
    _residentModelAssets.clear();
    _residentData_mutex.unlock();

    _refsAndRevision_mutex.lock();
    ++_revision;
    _refsAndRevision_mutex.unlock();
}

float
BiomeManager::getLODTransitionPixelScale() const
{
    return _lodTransitionPixelScale;
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
    recalculateResidentBiomes();
}

void
BiomeManager::recalculateResidentBiomes()
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
            else if (!_locked)
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
            auto& dummy = _residentBiomes[biome];
        }

        // get rid of biomes we no longer need.
        for (auto biome : biomes_to_remove)
        {
            _residentBiomes.erase(biome);
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
            OE_DEBUG << LC << "Unloaded asset " << asset->name() << std::endl;
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
    std::vector<const ModelAsset*> result;

    ScopedMutexLock lock(_residentData_mutex);

    result.reserve(_residentModelAssets.size());
    for (auto& entry : _residentModelAssets)
        result.push_back(entry.first);

    return std::move(result);
}

std::vector<const ModelAsset*>
BiomeManager::getResidentAssetsIfNotLocked() const
{
    std::vector<const ModelAsset*> result;
    if (_residentData_mutex.try_lock())
    {
        result.reserve(_residentModelAssets.size());
        for (auto& entry : _residentModelAssets)
            result.push_back(entry.first);

        _residentData_mutex.unlock();
    }
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

void
BiomeManager::materializeNewAssets(
    const osgDB::Options* readOptions)
{
    OE_PROFILING_ZONE;

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

    // Factory for loading chonk data. It will use our texture arena.
    ChonkFactory factory(_textures.get());

    // Clear out each biome's instances so we can start fresh.
    // This is a low-cost operation since anything we can re-use
    // will already by in the _residentModelAssetData collection.
    OE_DEBUG << LC << "Found " << _residentBiomes.size() << " resident biomes..." << std::endl;
    for (auto& iter : _residentBiomes)
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
            osg::Texture2D* tex = nullptr;

            // compresses the incoming texture if necessary
            if (image->getPixelFormat() != GL_RG)
                tex = new osg::Texture2D(convertNormalMapFromRGBToRG(image));
            else
                tex = new osg::Texture2D(image);

            return tex;
        }
    );

    // Go through the residency list and materialize any model assets
    // that are not already loaded (and present in _residentModelAssets);
    // Along the way, build the instances for each biome.
    for (auto& e : _residentBiomes)
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
                const ModelAsset* assetDef = asset_ptr.asset();

                OE_SOFT_ASSERT(assetDef != nullptr);
                if (assetDef == nullptr)
                    continue;

                // Look up this model asset. If it's already in the resident set,
                // do nothing; otherwise make it resident by loading all the data.
                ResidentModelAsset::Ptr& residentAsset = _residentModelAssets[assetDef];

                // First reference to this instance? Populate it:
                if (residentAsset == nullptr)
                {
                    OE_INFO << LC << "  Loading asset " << assetDef->name() << std::endl;

                    residentAsset = ResidentModelAsset::create();

                    residentAsset->assetDef() = assetDef;

                    osg::BoundingBox bbox;

                    if (assetDef->modelURI().isSet())
                    {
                        const URI& uri = assetDef->modelURI().get();

                        ModelCache::iterator ic = modelcache.find(uri);
                        if (ic != modelcache.end())
                        {
                            residentAsset->model() = ic->second._node.get();
                            residentAsset->boundingBox() = ic->second._modelAABB;
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
                                materialLoader.setReferrer(uri.full());
                                residentAsset->model()->accept(materialLoader);

                                // add flexors:
                                addFlexors(residentAsset->model(), assetDef->stiffness().get());

                                OE_DEBUG << LC << "Loaded model: " << uri.base() << std::endl;
                                modelcache[uri]._node = residentAsset->model().get();

                                osg::ComputeBoundsVisitor cbv;
                                residentAsset->model()->accept(cbv);
                                residentAsset->boundingBox() = cbv.getBoundingBox();
                                modelcache[uri]._modelAABB = residentAsset->boundingBox();
                            }
                            else
                            {
                                OE_WARN << LC << "Failed to load model " << uri.full() << std::endl;
                            }
                        }

                        bbox = residentAsset->boundingBox();
                    }

                    URI sideBB;
                    if (assetDef->sideBillboardURI().isSet())
                        sideBB = assetDef->sideBillboardURI().get();
                    else if (assetDef->modelURI().isSet())
                        sideBB = URI(assetDef->modelURI()->full() + ".side.png", assetDef->modelURI()->context());

                    if (!sideBB.empty())
                    {
                        const URI& uri = sideBB;

                        auto ic = texcache.find(uri);
                        if (ic != texcache.end())
                        {
                            residentAsset->sideBillboardTex() = ic->second->sideBillboardTex();
                            residentAsset->sideBillboardNormalMap() = ic->second->sideBillboardNormalMap();
                        }
                        else
                        {
                            osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
                            if (image.valid())
                            {
                                residentAsset->sideBillboardTex() = new osg::Texture2D(image.get());

                                OE_DEBUG << LC << "Loaded BB: " << uri.base() << std::endl;
                                texcache[uri] = residentAsset;

                                osg::ref_ptr<osg::Image> normalMap;
                                for (int i = 0; i < 2 && !normalMap.valid(); ++i)
                                {
                                    // normal map is the same file name but with _NML inserted before the extension
                                    URI normalMapURI(
                                        osgDB::getNameLessExtension(uri.full()) +
                                        (i == 0 ? "_NML." : ".normal.") +
                                        osgDB::getFileExtension(uri.full()));

                                    // silenty fail if no normal map found.
                                    normalMap = normalMapURI.getImage(readOptions);
                                    if (normalMap.valid())
                                    {
                                        residentAsset->sideBillboardNormalMap() = new osg::Texture2D(normalMap.get());
                                    }
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

                            auto ic = texcache.find(uri);
                            if (ic != texcache.end())
                            {
                                residentAsset->topBillboardTex() = ic->second->topBillboardTex();
                                residentAsset->topBillboardNormalMap() = ic->second->topBillboardNormalMap();
                            }
                            else
                            {
                                osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
                                if (image.valid())
                                {
                                    residentAsset->topBillboardTex() = new osg::Texture2D(image.get());

                                    OE_DEBUG << LC << "Loaded BB: " << uri.base() << std::endl;
                                    texcache[uri] = residentAsset;

                                    osg::ref_ptr<osg::Image> normalMap;
                                    for (int i = 0; i < 2 && !normalMap.valid(); ++i)
                                    {
                                        // normal map is the same file name but with _NML inserted before the extension
                                        URI normalMapURI(
                                            osgDB::getNameLessExtension(uri.full()) +
                                            (i == 0 ? "_NML." : ".normal.") +
                                            osgDB::getFileExtension(uri.full()));

                                        // silenty fail if no normal map found.
                                        normalMap = normalMapURI.getImage(readOptions);
                                        if (normalMap.valid())
                                        {
                                            residentAsset->topBillboardNormalMap() = new osg::Texture2D(normalMap.get());
                                        }
                                    }
                                }
                                else
                                {
                                    OE_WARN << LC << "Failed to load top billboard " << uri.full() << std::endl;
                                }
                            }
                        }

                        std::vector<osg::Texture*> textures(4);
                        textures[0] = residentAsset->sideBillboardTex().get();
                        textures[1] = residentAsset->sideBillboardNormalMap().get();
                        textures[2] = residentAsset->topBillboardTex().get();
                        textures[3] = residentAsset->topBillboardNormalMap().get();

                        if (_createImpostor[group])
                        {
                            if (!bbox.valid())
                            {
                                bbox.set(
                                    -residentAsset->assetDef()->width().get(),
                                    -residentAsset->assetDef()->width().get(),
                                    0,
                                    residentAsset->assetDef()->width().get(),
                                    residentAsset->assetDef()->width().get(),
                                    residentAsset->assetDef()->height().get());
                            }

                            residentAsset->impostor() = _createImpostor[group](
                                bbox,
                                textures);
                        }
                    }

                    // Finally, chonkify.
                    if (residentAsset->model().valid())
                    {
                        // models should disappear 8x closer than the SSE:
                        float far_pixel_scale = getLODTransitionPixelScale();
                        float near_pixel_scale = FLT_MAX;

                        if (residentAsset->chonk() == nullptr)
                            residentAsset->chonk() = Chonk::create();

                        residentAsset->chonk()->add(
                            residentAsset->model().get(),
                            far_pixel_scale,
                            near_pixel_scale,
                            factory);
                    }

                    if (residentAsset->impostor().valid())
                    {
                        float far_pixel_scale = 1.0f;
                        float near_pixel_scale = residentAsset->model().valid() ?
                            getLODTransitionPixelScale() : 
                            FLT_MAX;

                        if (residentAsset->chonk() == nullptr)
                            residentAsset->chonk() = Chonk::create();

                        residentAsset->chonk()->add(
                            residentAsset->impostor().get(),
                            far_pixel_scale,
                            near_pixel_scale,
                            factory);
                    }
                }

                // If this data successfully materialized, add it to the
                // biome's instance collection.
                if (residentAsset->sideBillboardTex().valid() || residentAsset->model().valid())
                {
                    ResidentModelAssetInstance instance;
                    instance.residentAsset() = residentAsset;
                    instance.weight() = asset_ptr.weight();
                    instance.coverage() = asset_ptr.coverage();
                    assetInstances.push_back(std::move(instance));
                }
            }

            if (!assetInstances.empty())
            {
                asset_groups.insert((AssetGroup::Type)group);
            }
        }
    }
}

void
BiomeManager::setCreateFunction(
    AssetGroup::Type group,
    createImpostorFunction func)
{
    ScopedMutexLock lock(_residentData_mutex);
    _createImpostor[group] = func;
}

BiomeManager::ResidentBiomes
BiomeManager::getResidentBiomes(
    const osgDB::Options* readOptions)
{
    OE_PROFILING_ZONE;

    // First refresh the resident biome collection based on current refcounts
    recalculateResidentBiomes();

    // Next go through and load any assets that are not yet loaded
    materializeNewAssets(readOptions);

    // Make a copy:
    ResidentBiomes result = _residentBiomes;

    return std::move(result);
}

namespace
{
    template<class T>
    struct MyVisitor : public osg::NodeVisitor
    {
        using Function = std::function<void(T&, const osg::Matrix&)>;
        Function _func;
        std::stack<osg::Matrix> _transformStack;

        MyVisitor() : osg::NodeVisitor()
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
            _transformStack.push(osg::Matrix::identity());
        }

        void apply(osg::Node& node) override
        {
            T* n = dynamic_cast<T*>(&node);
            if (n) _func(*n, _transformStack.top());
            traverse(node);
        }

        void apply(osg::Transform& node) override
        {
            osg::Matrix m = _transformStack.empty() ? osg::Matrix() : _transformStack.top();
            node.computeLocalToWorldMatrix(m, this);
            _transformStack.push(m);
            apply(static_cast<osg::Group&>(node));
            _transformStack.pop();
        }

        void visit(
            osg::ref_ptr<osg::Node>& node,
            Function func)
        {
            _func = func;
            node->accept(*this);
        }
    };

    template<class T>
    struct TriangleFullSend {
        T* _receiver;
        osg::Geometry* _geom;
        inline void operator () (unsigned i0, unsigned i1, unsigned i2) {
            _receiver->triangle(*_geom, i0, i1, i2);
        }
    };

    //template<class T>
    struct TriangleVisitor : public osg::NodeVisitor
    {        
        using TriangleFunction = std::function<void(
            osg::Geometry& geom,
            unsigned i0,
            unsigned i1,
            unsigned i2,
            const osg::Matrix& l2w)>;

        TriangleFunction _func;

        std::stack<osg::Matrix> _transformStack;

        TriangleVisitor() : osg::NodeVisitor()
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
            _transformStack.push(osg::Matrix::identity());
        }

        void apply(osg::Node& node) override
        {
            //T* n = dynamic_cast<T*>(&node);
            //if (n) _func(*n, _transformStack.top());
            traverse(node);
        }

        void apply(osg::Transform& node) override
        {
            osg::Matrix m = _transformStack.empty() ? osg::Matrix() : _transformStack.top();
            node.computeLocalToWorldMatrix(m, this);
            _transformStack.push(m);
            apply(static_cast<osg::Group&>(node));
            _transformStack.pop();
        }

        void apply(osg::Geometry& geom) override
        {
            osg::TriangleIndexFunctor<TriangleFullSend<TriangleVisitor>> _sender;
            _sender._receiver = this;
            _sender._geom = &geom;
            geom.accept(_sender);
        }

        void visit(
            osg::ref_ptr<osg::Node>& node,
            TriangleFunction func)
        {
            _func = func;
            node->accept(*this);
        }

        void triangle(osg::Geometry& geom, unsigned i0, unsigned i1, unsigned i2)
        {
            auto verts = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
            _func(geom, i0, i1, i2, _transformStack.top());
        }
    };
}

void
BiomeManager::addFlexors(osg::ref_ptr<osg::Node>& node, float stiffness)
{
    // note: stiffness is [0..1].

    osg::ComputeBoundsVisitor cb;
    node->accept(cb);
    auto& bbox = cb.getBoundingBox();

    auto addFlexors = [bbox, stiffness](osg::Geometry& geom, const osg::Matrix& l2w)
    {
        osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
        if (!verts) return;

        osg::Vec3Array* flexors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
        flexors->reserve(verts->size());
        geom.setTexCoordArray(3, flexors);

        osg::Vec3f base(bbox.center().x(), bbox.center().y(), bbox.zMin());
        float xy_radius = std::max(bbox.xMax() - bbox.xMin(), bbox.yMax() - bbox.yMin());

        for (auto& local_vert : *verts)
        {
            auto vert = local_vert * l2w;
            osg::Vec3f anchor(0.0f, 0.0f, vert.z());
            float height_ratio = harden((vert.z() - bbox.zMin()) / (bbox.zMax() - bbox.zMin()));
            auto lateral_vec = (vert - osg::Vec3f(0, 0, vert.z()));
            float lateral_ratio = harden(lateral_vec.length() / xy_radius);
            auto flex =
                normalize(lateral_vec) *
                (lateral_ratio * 3.0f) *
                (height_ratio * 1.5f) *
                (1.0-stiffness);

            flexors->push_back(flex);
        }
    };

    MyVisitor<osg::Geometry> visitor;
    visitor.visit(node, addFlexors);
}
