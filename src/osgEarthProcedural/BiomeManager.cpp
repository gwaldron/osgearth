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
#include <osgEarth/weemesh.h>
#include <osgEarth/MeshConsolidator>
#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osgUtil/SmoothingVisitor>
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#undef LC
#define LC "[BiomeManager] "

#define NORMAL_MAP_TEX_UNIT 1
#define PBR_TEX_UNIT 2

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

BiomeManager::BiomeManager() :
    _revision(0),
    _refsAndRevision_mutex("BiomeManager.refsAndRevision(OE)"),
    _residentData_mutex("BiomeManager.residentData(OE)"),
    _lodTransitionPixelScale(16.0f),
    _locked(false)
{
    // this arena will hold all the textures for loaded assets.
    _textures = new TextureArena();
    _textures->setAutoRelease(true);
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
        OE_DEBUG << LC << "Hello, " << biome->name().get() << " (" << biome->index() << ")" << std::endl;
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
            OE_DEBUG << LC << "Goodbye, " << biome->name().get() << "(" << biome->index() << ")" << std::endl;
        }
    }
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
            OE_DEBUG << LC << "Goodbye, " << biome->name().get() << std::endl;
            iter.second = 0;
        }

        ++_revision;
    }

    // Resolve the references and unload any resident assets from memory.
    recalculateResidentBiomes();
}

void
BiomeManager::flush()
{
    recalculateResidentBiomes();

    if (_textures.valid())
        _textures->flush();
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
            auto& entry = _residentBiomes[biome->id()];
            entry.biome = biome;
        }

        // get rid of biomes we no longer need.
        for (auto biome : biomes_to_remove)
        {
            _residentBiomes.erase(biome->id());
        }


        // finally, update the collection of resident assets to
        // reflect the reference counts.
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
            OE_DEBUG << LC << "Unloaded asset " << asset_name << std::endl;
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
    {
        result.push_back(entry.second->assetDef());
    }

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
        {
            result.push_back(entry.second->assetDef());
        }
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

void
BiomeManager::materializeNewAssets(
    const osgDB::Options* readOptions)
{
    OE_PROFILING_ZONE;

    // exclusive access to the resident dataset
    ScopedMutexLock lock(_residentData_mutex);

    // Some caches to avoid duplicating data
    std::map<URI, ResidentModelAsset::Ptr> texcache;
    std::map<URI, ModelCacheEntry> modelcache;

    // Factory for loading chonk data. It will use our texture arena.
    ChonkFactory factory(_textures.get());

    // Clear out each biome's instances so we can start fresh.
    // This is a low-cost operation since anything we can re-use
    // will already by in the _residentModelAssetData collection.
    OE_DEBUG << LC << "Found " << _residentBiomes.size() << " resident biomes..." << std::endl;
    for (auto& iter : _residentBiomes)
    {
        iter.second.instances.clear();
    }

    // This loader will find material textures and install them on
    // secondary texture image units.. in this case, normal maps.
    // We can expand this later to include other types of material maps.
    Util::MaterialLoader materialLoader;


    auto getNormalMapFileName = MaterialUtils::getDefaultNormalMapNameMangler();

    materialLoader.setMangler(
        NORMAL_MAP_TEX_UNIT, getNormalMapFileName);

#if 0
    // don't do this - it will break the auto-compressed-normal detection
    // in the shader. todo: support RGTC2_EXT instead in PixelWriter.
    materialLoader.setTextureFactory(
        NORMAL_MAP_TEX_UNIT,
        [](osg::Image* image)
        {
            osg::Texture2D* tex = nullptr;

            // compresses the incoming texture if necessary
            if (image->getPixelFormat() != GL_RG && !image->isCompressed())
                tex = new osg::Texture2D(convertNormalMapFromRGBToRG(image));
            else
                tex = new osg::Texture2D(image);

            return tex;
        }
    );
#endif

    auto getPBRMapFileName = MaterialUtils::getDefaultPBRMapNameMangler();

    materialLoader.setMangler(
        PBR_TEX_UNIT, getPBRMapFileName);

    // Go through the residency list and materialize any model assets
    // that are not already loaded (and present in _residentModelAssets);
    // Along the way, build the instances for each biome.
    for (auto& e : _residentBiomes)
    {
        const Biome* biome = e.second.biome;

        auto& assetsToUse = biome->getModelAssets();

        // this is the collection of instances we're going to populate
        ResidentModelAssetInstances& assetInstances = e.second.instances;

        // The group points to multiple assets, which we will analyze and load.
        for (auto& asset_ptr : assetsToUse)
        {
            const ModelAsset* assetDef = asset_ptr->asset();

            OE_SOFT_ASSERT(assetDef != nullptr);
            if (assetDef == nullptr)
                continue;

            // Look up this model asset. If it's already in the resident set,
            // do nothing; otherwise make it resident by loading all the data.
            ResidentModelAsset::Ptr& residentAsset = _residentModelAssets[assetDef->name()];

            // First reference to this instance? Populate it:
            if (residentAsset == nullptr)
            {
                OE_DEBUG << LC << "  Loading asset " << assetDef->name() << std::endl;

                residentAsset = ResidentModelAsset::create();

                residentAsset->assetDef() = assetDef;

                if (assetDef->modelURI().isSet())
                {
                    const URI& uri = assetDef->modelURI().get();

                    auto cache_iter = modelcache.find(uri);
                    if (cache_iter != modelcache.end())
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
                            materialLoader.setReferrer(uri.full());
                            residentAsset->model()->accept(materialLoader);

                            // add flexors:
                            bool isUndergrowth = assetDef->group() == "undergrowth";
                            addFlexors(residentAsset->model(), assetDef->stiffness().get(), isUndergrowth);

                            modelcache[uri]._node = residentAsset->model().get();

                            osg::ComputeBoundsVisitor cbv;
                            residentAsset->model()->accept(cbv);
                            residentAsset->boundingBox() = cbv.getBoundingBox();
                            modelcache[uri]._modelAABB = residentAsset->boundingBox();

                            OE_INFO << LC << "Loaded model: " << uri.base() << 
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

                    auto ic = texcache.find(uri);
                    if (ic != texcache.end())
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

                            OE_DEBUG << LC << "Loaded side BB: " << uri.base() << std::endl;
                            texcache[uri] = residentAsset;

                            // normal map:
                            URI normalMapURI(getNormalMapFileName(uri.full()));
                            osg::ref_ptr<osg::Image> normalMap = normalMapURI.getImage(readOptions);
                            if (normalMap.valid())
                            {
                                OE_DEBUG << LC << "Loaded NML: " << normalMapURI.base() << std::endl;
                                residentAsset->sideBillboardNormalMap() = new osg::Texture2D(normalMap.get());
                            }
                            else
                            {
                                OE_INFO << LC << "Failed to load: " << normalMapURI.base() << std::endl;
                            }

                            URI pbrMapURI(getPBRMapFileName(uri.full()));
                            osg::ref_ptr<osg::Image> pbrMap = pbrMapURI.getImage(readOptions);
                            if (pbrMap.valid())
                            {
                                OE_DEBUG << LC << "Loaded PBR: " << pbrMapURI.base() << std::endl;
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

                        auto ic = texcache.find(uri);
                        if (ic != texcache.end())
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

                                OE_DEBUG << LC << "Loaded top BB: " << uri.base() << std::endl;
                                texcache[uri] = residentAsset;

                                // normal map:
                                URI normalMapURI(getNormalMapFileName(uri.full()));
                                osg::ref_ptr<osg::Image> normalMap = normalMapURI.getImage(readOptions);
                                if (normalMap.valid())
                                {
                                    OE_DEBUG << LC << "Loaded NML: " << normalMapURI.base() << std::endl;
                                    residentAsset->topBillboardNormalMap() = new osg::Texture2D(normalMap.get());
                                }
                                else
                                {
                                    OE_INFO << LC << "Failed to load: " << normalMapURI.base() << std::endl;
                                }

                                // PBR map:
                                URI pbrMapURI(getPBRMapFileName(uri.full()));
                                osg::ref_ptr<osg::Image> pbrMap = pbrMapURI.getImage(readOptions);
                                if (pbrMap.valid())
                                {
                                    OE_DEBUG << LC << "Loaded PBR: " << pbrMapURI.base() << std::endl;
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
                    auto iter = _createImpostorFunctions.find(assetDef->group());
                    if (iter != _createImpostorFunctions.end())
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

                            Impostor imp = createImpostor(
                                bbox,
                                top_z,
                                textures);

                            // post-process if there's a post-URI callback.
                            // This is so the Impostor can have extended materials injected.
                            URIPostReadCallback* post = URIPostReadCallback::from(readOptions);
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
                    float far_pixel_scale = getLODTransitionPixelScale();

                    // Real model can get as close as it wants:
                    float near_pixel_scale = FLT_MAX;

                    if (residentAsset->chonk() == nullptr)
                        residentAsset->chonk() = Chonk::create();

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
                        factory);
#endif
                }

                if (residentAsset->impostor().valid())
                {
                    // Fade out the impostor at this pixel scale (i.e., multiple of SSE)
                    float far_pixel_scale = impostorFarLODScale;

                    // Fade the impostor to the 3D model at this pixel scale:
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
                instance.weight() = asset_ptr->weight();
                instance.coverage() = asset_ptr->coverage();
                assetInstances.push_back(std::move(instance));
            }
        }   
    }
}

void
BiomeManager::setCreateImpostorFunction(
    const std::string& group,
    BiomeManager::CreateImpostorFunction func)
{
    ScopedMutexLock lock(_residentData_mutex);
    _createImpostorFunctions[group] = func;
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
BiomeManager::addFlexors(
    osg::ref_ptr<osg::Node>& node,
    float stiffness,
    bool isUndergrowth)
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
