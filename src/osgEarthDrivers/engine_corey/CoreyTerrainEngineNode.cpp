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
#include "CoreyTerrainEngineNode"
#include "Shaders"
#include "TerrainCuller"
#include "TileGeometry"
#include "CreateTileImplementation"

#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/MapModelChange>
#include <osgEarth/Threading>
#include <osgEarth/Progress>
#include <osgEarth/ShaderLoader>
#include <osgEarth/Utils>
#include <osgEarth/ObjectIndex>
#include <osgEarth/Metrics>
#include <osgEarth/Elevation>
#include <osgEarth/LandCover>
#include <osgEarth/CameraUtils>
#include <osgEarth/ShaderFactory>

#include <osg/Version>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/CullFace>
#include <osg/ValueObject>

#include <cstdlib> // for getenv

#define LC "[CoreyTerrainEngineNode] "

using namespace osgEarth::Corey;
using namespace osgEarth;

#define DEFAULT_MAX_LOD 19u

#define TERRAIN_JOB_POOL "oe.terrain"

//------------------------------------------------------------------------

namespace
{
    // adapter that lets CoreyTerrainEngineNode listen to Map events
    struct CoreyTerrainEngineNodeMapCallbackProxy : public MapCallback
    {
        CoreyTerrainEngineNodeMapCallbackProxy(CoreyTerrainEngineNode* node) : _node(node) { }
        osg::observer_ptr<CoreyTerrainEngineNode> _node;

        void onMapModelChanged(const MapModelChange& change) override {
            osg::ref_ptr<CoreyTerrainEngineNode> node;
            if (_node.lock(node))
                node->onMapModelChanged(change);
        }
    };


    /**
    * Run this visitor whenever you remove a layer, so that each
    * TileNode can update its render model and get rid of passes
    * that no longer exist.
    */
    struct PurgeOrphanedLayers : public osg::NodeVisitor
    {
        const Map* _map;
        const RenderBindings& _bindings;
        unsigned _count;

        PurgeOrphanedLayers(const Map* map, RenderBindings& bindings) : _map(map), _bindings(bindings), _count(0u)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }

        void apply(osg::Node& node)
        {
            TileNode* tileNode = dynamic_cast<TileNode*>(&node);
            if (tileNode)
            {
                apply(*tileNode);
            }
            traverse(node);
        }

        void apply(TileNode& tileNode)
        {
            TileRenderModel& model = tileNode._renderModel;

            for (int p = 0; p < model._passes.size(); ++p)
            {
                RenderingPass& pass = model._passes[p];

                // if the map doesn't contain a layer with a matching UID,
                // or if the layer is now disabled, remove it from the render model.
                Layer* layer = _map->getLayerByUID(pass.sourceUID());
                if (layer == nullptr || layer->isOpen() == false)
                {
                    model._passes.erase(model._passes.begin() + p);
                    --p;
                    _count++;
                }
            }

#if 0
            // For shared samplers we need to refresh the list if one of them
            // goes inactive (as is the case when removing a shared layer)
            tileNode.refreshSharedSamplers(_bindings);
#endif
        }
    };

    class TerrainPagedNode : public PagedNode2
    {
    public:
        TerrainPagedNode(const TileKey& key) : _key(key) { }
        TileKey _key;
        bool _dirty = false;
    };
}

//------------------------------------------------------------------------

CoreyTerrainEngineNode::CoreyTerrainEngineNode() :
    TerrainEngineNode()
{
    // activate update traversals for this node.
    ADJUST_UPDATE_TRAV_COUNT(this, +1);

    // Necessary for pager object data
    // Note: Do not change this value. Apps depend on it to
    // detect being inside a terrain traversal.
    this->setName("corey");

    // unique ID for this engine:
    _uid = osgEarth::createUID();

    // Corey engine does NOT require elevation textures
    _require.fullDataAtFirstLod = true;
    _require.tileMesh = true;
    _require.elevationTextures = true;
    _require.normalTextures = true;
    _require.landCoverTextures = false;

    // static shaders.
    osg::StateSet* stateset = getOrCreateStateSet();
    stateset->setName("Terrain node");

    _surfaceSS = new osg::StateSet();
    _surfaceSS->setName("Terrain surface");

    _imageLayerSS = new osg::StateSet();
    _imageLayerSS->setName("Terrain image layer");

    _updatedThisFrame = false;
}

CoreyTerrainEngineNode::~CoreyTerrainEngineNode()
{
    if (_ppUID > 0)
        Registry::instance()->getShaderFactory()->removePreProcessorCallback(_ppUID);
}

void
CoreyTerrainEngineNode::resizeGLObjectBuffers(unsigned maxSize)
{
    TerrainEngineNode::resizeGLObjectBuffers(maxSize);

    getStateSet()->resizeGLObjectBuffers(maxSize);
    _surfaceSS->resizeGLObjectBuffers(maxSize);
    _imageLayerSS->resizeGLObjectBuffers(maxSize);

    // TODO: where should this live? MapNode?
    LayerVector layers;
    getMap()->getLayers(layers);
    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        if ((*i)->getStateSet()) {
            (*i)->getStateSet()->resizeGLObjectBuffers(maxSize);
        }
    }
}

void
CoreyTerrainEngineNode::releaseGLObjects(osg::State* state) const
{
    if (_imageLayerSS.valid())
        _imageLayerSS->releaseGLObjects(state);

    if (_surfaceSS.valid())
        _surfaceSS->releaseGLObjects(state);

    // release the LayerDrawables
    for (auto& p : _persistent)
    {
        for (auto& d : p.second._drawables)
        {
            d.second->releaseGLObjects(state);
        }
    }

    if (_data.textures.valid())
    {
        _data.textures->releaseGLObjects(state);
    }

    TerrainEngineNode::releaseGLObjects(state);
}

void
CoreyTerrainEngineNode::shutdown()
{
    TerrainEngineNode::shutdown();
}

std::string
CoreyTerrainEngineNode::getJobArenaName() const
{
    return TERRAIN_JOB_POOL;
}

unsigned
CoreyTerrainEngineNode::getNumResidentTiles() const
{
    return _terrain->getNumTrackedNodes();
}

void
CoreyTerrainEngineNode::onSetMap()
{
    OE_SOFT_ASSERT_AND_RETURN(_map.valid(), void());

    int maxSize = std::min((int)getOptions().getMaxTextureSize(), Registry::instance()->getMaxTextureSize());

    // Populate the engine data structure
    _data.map = _map;
    _data.options = getOptions();
    _data.clock = &_clock;
    _data.renderBindings = {};
    _data.selectionInfo = {};
    
    _data.textures = new TextureArena();
    _data.textures->setName("Corey Terrain Engine");
    _data.textures->setBindingPoint(29); // TODO
    _data.textures->setAutoRelease(true);
    _data.textures->setMaxTextureSize(maxSize);

    // callbacks for the scene graph.
    _data.tileNodeCullCallback = new TileNodeCuller(_data);
    _data.tileDrawableCullCallback = new TileDrawableCuller(_data);



    osg::observer_ptr<CoreyTerrainEngineNode> engine_weak = this;

    auto load_tile = [engine_weak](const TileKey& key, ProgressCallback* progress) -> osg::ref_ptr<TileNode>
        {
            osg::ref_ptr<TileNode> tilenode;
            osg::ref_ptr<CoreyTerrainEngineNode> engine;
            if (engine_weak.lock(engine))
            {
                osg::ref_ptr<TerrainTileModel> model = engine->_tileModelDB.getOrCreate(key,
                    [&](const TileKey& key) {
                        return engine->_tileModelFactory->createTileModel(
                            engine->_data.map.get(), key, {}, engine->_require, progress);
                    });

                if (model.valid() && model->mesh.valid())
                {
                    tilenode = new TileNode(key);
                    auto parent = engine->_data.getTileNode(key.createParentKey());
                    auto parentData = parent.valid() ? parent->_fullDataModel.get() : nullptr;
                    tilenode->set(model.get(), parentData, engine->_data);

                    engine->_data.storeTileNode(tilenode);
                }
            }
            return tilenode;
        };
    
    _terrain = new PagingManager(TERRAIN_JOB_POOL);
    addChild(_terrain);

    _pager = new SimplePager(_map.get(), _map->getProfile());
    _pager->setName("oe.corey");
    _pager->setMaxLevel(19u);
    _pager->setAdditive(false);
    _pager->setUsePayloadBoundsForChildren(true);
    _pager->setCreateNodeFunction(load_tile);
    _pager->setCreatePagedNodeFunction([](const TileKey& key) { return new TerrainPagedNode(key); });
    _terrain->addChild(_pager);

    _morphingSupported = true;
    auto options = getOptions();
    if (options.getLODMethod() ==LODMethod::SCREEN_SPACE)
    {
        OE_INFO << LC << "LOD method = pixel size; pixel tile size = " << options.getTilePixelSize() << std::endl;

        // force morphing off for PSOS mode
        _morphingSupported = false;
    }

#if 0
    // There is an (apparent) bug in the mesa 23.1.4 driver that causes display artifacts 
    // when doing morphing; this code will attempt to detect that condition and disable
    // the offending code until we can find a workaround.
    auto gl_version_str = Registry::capabilities().getVersion();
    auto mesa_pos = gl_version_str.find("Mesa");
    if (mesa_pos != std::string::npos)
    {
        auto ver = gl_version_str.substr(mesa_pos + 5);
        Version driver_version = parseVersion(ver.c_str());
        if (driver_version.greaterThanOrEqualTo(23, 1, 4))
        {
            _morphingSupported = false;
            getOrCreateStateSet()->setDefine("OE_MESA_23_WORKAROUND");
        }
    }
#endif

    // morphing imagery LODs requires we bind parent textures to their own unit.
    if (options.getMorphImagery() && _morphingSupported)
    {
        _require.parentTextures = true;
    }

    // Terrain morphing doesn't work in projected maps:
    if (_map->getSRS()->isProjected())
    {
        _morphTerrainSupported = false;
    }

    // Check for normals debugging.
    if (::getenv("OSGEARTH_DEBUG_NORMALS"))
        getOrCreateStateSet()->setDefine("OE_DEBUG_NORMALS");
    else
        if (getStateSet()) getStateSet()->removeDefine("OE_DEBUG_NORMALS");

    // check for normal map generation (required for lighting).
    _require.normalTextures = (options.getUseNormalMaps() == true);

    // don't know how to set this up so just do it
    // always off in corey
    //_require.landCoverTextures = (options.getUseLandCover() == true);

    // ensure we get full coverage at the first LOD.
    _require.fullDataAtFirstLod = true;

    // Loader concurrency (size of the thread pool)
    unsigned concurrency = options.getConcurrency();
    const char* concurrency_str = ::getenv("OSGEARTH_TERRAIN_CONCURRENCY");
    if (concurrency_str)
        concurrency = Strings::as<unsigned>(concurrency_str, concurrency);
    jobs::get_pool(TERRAIN_JOB_POOL)->set_concurrency(concurrency);

    // Initialize the core render bindings.
    setupRenderBindings();

    // install a layer callback for processing further map actions:
    _map->addMapCallback(new CoreyTerrainEngineNodeMapCallbackProxy(this));

    // Prime with existing layers:
    _batchUpdateInProgress = true;

    LayerVector layers;
    _map->getLayers(layers);
    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
        addLayer(i->get());

    _batchUpdateInProgress = false;

    // Calculate the LOD morphing parameters:
    unsigned maxLOD = options.getMaxLOD();

    _data.selectionInfo.initialize(
        0u, // always zero, not the terrain options firstLOD
        maxLOD,
        _map->getProfile(),
        options.getMinTileRangeFactor(),
        true); // restrict polar subdivision for geographic maps

    TerrainResources* res = getResources();
    for (unsigned lod = 0; lod <= maxLOD; ++lod)
        res->setVisibilityRangeHint(lod, _data.selectionInfo.getLOD(lod)._visibilityRange);

    // set up the initial graph
    refresh();

    // now that we have a map, set up to recompute the bounds
    dirtyBound();

    // preprocess shaders to parse the "oe_use_shared_layer" directive
    // for shared layer samplers
    if (_ppUID > 0)
    {
        Registry::instance()->getShaderFactory()->removePreProcessorCallback(_ppUID);
    }

    _ppUID = Registry::instance()->getShaderFactory()->addPreProcessorCallback(
        this,
        [](std::string& source, osg::Referenced* host)
        {
            CoreyTerrainEngineNode* engine = dynamic_cast<CoreyTerrainEngineNode*>(host);
            if (!engine) return;

            std::string line;
            std::vector<std::string> tokens;

            while (ShaderLoader::getPragmaValueAsTokens(
                source,
                "#pragma oe_use_shared_layer",
                line,
                tokens))
            {
                if (tokens.size() == 2)
                {
                    std::ostringstream buf;

                    if (GLUtils::useNVGL())
                    {
                        ShadersGL4 sh;
                        std::string incStrGL4 = ShaderLoader::load(sh.ENGINE_TYPES, sh);
                        if (source.find(incStrGL4) == std::string::npos)
                        {
                            buf << incStrGL4 << "\n";
                        }

                        // find the shared index.
                        int index = -1;
                        const RenderBindings& bindings = engine->_data.renderBindings;
                        for (int i = SamplerBinding::SHARED; i < (int)bindings.size() && index < 0; ++i)
                        {
                            if (bindings[i].samplerName == tokens[0])
                            {
                                index = i - SamplerBinding::SHARED;
                            }
                        }
                        if (index < 0)
                        {
                            OE_WARN << LC << "Cannot find a shared sampler binding for " << tokens[0] << std::endl;
                            Strings::replaceIn(source, line, "// error, no matching sampler binding");
                            continue;
                        }

                        buf << "#define " << tokens[0] << "_HANDLE oe_terrain_tex[oe_tile[oe_tileID].sharedIndex[" << index << "]]\n"
                            << "#define " << tokens[0] << " sampler2D(" << tokens[0] << "_HANDLE)\n"
                            << "#define " << tokens[1] << " oe_tile[oe_tileID].sharedMat[" << index << "]\n";
                    }
                    else
                    {
                        buf << "uniform sampler2D " << tokens[0] << ";\n"
                            << "uniform mat4 " << tokens[1] << ";\n";
                    }

                    Strings::replaceIn(source, line, buf.str());
                }
                else
                {
                    Strings::replaceIn(source, line, "// error, missing token(s)");
                }
            }
        }
    );

    // finally, fire up the pager.
    _pager->build();
}


osg::BoundingSphere
CoreyTerrainEngineNode::computeBound() const
{
    return TerrainEngineNode::computeBound();
}

namespace
{
    struct Invalidater : public osg::NodeVisitor
    {
        GeoExtent extent;
        unsigned minLevel = 0, maxLevel = 99;
        std::function<osg::ref_ptr<TerrainTileModel>(const TileKey&)> getTileModel;
        CreateTileManifest manifest;

        Invalidater() {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }
        void apply(osg::Node& node)
        {
            bool keep_going = true;
            TileNode* tilenode = dynamic_cast<TileNode*>(&node);

            if (tilenode &&
                tilenode->_key.getLOD() >= minLevel &&
                tilenode->_key.getLOD() <= maxLevel &&
                (!extent.isValid() || tilenode->_key.getExtent().intersects(extent)))
            {
                tilenode->_dirty = true;
            }
            else
            {
                traverse(node);
            }
        }
    };
}

void
CoreyTerrainEngineNode::invalidateRegion(
    const GeoExtent& extent,
    unsigned         minLevel,
    unsigned         maxLevel)
{
    _tileModelDB.clear();

    Invalidater v;
    v.extent = extent;
    v.minLevel = minLevel;
    v.maxLevel = maxLevel;

    v.getTileModel = [&](const TileKey& key)
        {
            return _tileModelDB.getOrCreate(key,
                [&](const TileKey& key) {
                    return _tileModelFactory->createTileModel(_map.get(), key, {}, _require, nullptr);
                });
        };

    v.manifest = {};

    this->accept(v);
}

void
CoreyTerrainEngineNode::invalidateRegion(
    const std::vector<const Layer*> layers,
    const GeoExtent& extent,
    unsigned minLevel,
    unsigned maxLevel)
{
    invalidateRegion(extent, minLevel, maxLevel);
    return;

    _tileModelDB.clear();

    Invalidater v;
    v.extent = extent;
    v.minLevel = minLevel;
    v.maxLevel = maxLevel;

    v.getTileModel = [&](const TileKey& key)
        {
            CreateTileManifest manifest;
            for (auto& layer : layers)
                manifest.insert(layer);

            return _tileModelDB.getOrCreate(key,
                [&](const TileKey& key) {
                    return _tileModelFactory->createTileModel(_map.get(), key, manifest, _require, nullptr);
                });
        };

    v.manifest = {};

    this->accept(v);
}

void
CoreyTerrainEngineNode::refresh(bool forceDirty)
{
    OE_SOFT_ASSERT_AND_RETURN("NOT IMPLEMENTED - TODO", void());

    if (_batchUpdateInProgress)
    {
        _refreshRequired = true;
    }
    else
    {
        _refreshRequired = false;
#if 0

        if (_terrain.valid())
        {
            _terrain->releaseGLObjects();
            _terrain->removeChildren(0, _terrain->getNumChildren());
        }

        //// clear the loader:
        //_merger->clear();

        //// clear out the tile registry:
        //if (_tiles)
        //{
        //    _tiles->releaseAll(nullptr);
        //}

        // scrub the geometry pool:
        _geometryPool->clear();

        // Build the first level of the terrain.
        // Collect the tile keys comprising the root tiles of the terrain.
        std::vector<TileKey> keys;
        getMap()->getProfile()->getAllKeysAtLOD(getOptions().getFirstLOD(), keys);

        // create a root node for each root tile key.
        OE_INFO << LC << "Creating " << keys.size() << " root keys." << std::endl;

        // We need to take a self-ref here to ensure that the TileNode's data loader
        // can use its observer_ptr back to the terrain engine.
        this->ref();

        // Load all the root key tiles.
        jobs::context context;
        context.group = jobs::jobgroup::create();
        context.pool = jobs::get_pool(ARENA_LOAD_TILE);

        for (unsigned i = 0; i < keys.size(); ++i)
        {
            TileNode* tileNode = new TileNode(
                keys[i],
                nullptr, // parent
                _EngineData.get(),
                nullptr); // progress

            // Root nodes never expire
            tileNode->setDoNotExpire(true);

            // Add it to the scene graph
            _terrain->addChild(tileNode);

            // Post-add initialization:
            tileNode->initializeData();

            // And load the tile's data
            jobs::dispatch([tileNode]() { tileNode->loadSync(); }, context);

            OE_DEBUG << " - " << (i + 1) << "/" << keys.size() << " : " << keys[i].str() << std::endl;
        }

        // wait for all loadSync calls to complete
        context.group->join();

        // release the self-ref.
        this->unref_nodelete();
#endif

        // Set up the state sets.
        updateState();
    }
}

osg::StateSet*
CoreyTerrainEngineNode::getTerrainStateSet()
{
    OE_SOFT_ASSERT_AND_RETURN(_terrain.valid(), nullptr);
    return _terrain->getOrCreateStateSet();
}

osg::StateSet*
CoreyTerrainEngineNode::getSurfaceStateSet()
{
    return _surfaceSS.get();
}

void
CoreyTerrainEngineNode::setupRenderBindings()
{
    // Release any pre-existing bindings:
    for (unsigned i = 0; i < _data.renderBindings.size(); ++i)
    {
        SamplerBinding& b = _data.renderBindings[i];
        if (b.isActive())
        {
            getResources()->releaseTextureImageUnit(b.unit);
        }
    }
    _data.renderBindings.clear();

    // "SHARED" is the start of shared layers, so we always want the bindings
    // vector to be at least that size.
    _data.renderBindings.resize(SamplerBinding::SHARED);

    SamplerBinding& color = _data.renderBindings[SamplerBinding::COLOR];
    color.usage = SamplerBinding::COLOR;
    color.samplerName = "oe_layer_tex";
    color.matrixName = "oe_layer_texMatrix";
    color.defaultTexture = new osg::Texture2D(ImageUtils::createEmptyImage(1, 1));
    color.defaultTexture->setName("terrain default color");

    if (!GLUtils::useNVGL())
        getResources()->reserveTextureImageUnit(color.unit, "Terrain Color");

    if (_require.elevationTextures)
    {
        SamplerBinding& elevation = _data.renderBindings[SamplerBinding::ELEVATION];
        elevation.usage = SamplerBinding::ELEVATION;
        elevation.samplerName = "oe_tile_elevationTex";
        elevation.matrixName = "oe_tile_elevationTexMatrix";
        elevation.defaultTexture = osgEarth::createEmptyElevationTexture();
        elevation.defaultTexture->setName("terrain default elevation");

        if (!GLUtils::useNVGL())
            getResources()->reserveTextureImageUnit(elevation.unit, "Terrain Elevation");
    }

    if (_require.normalTextures)
    {
        SamplerBinding& normal = _data.renderBindings[SamplerBinding::NORMAL];
        normal.usage = SamplerBinding::NORMAL;
        normal.samplerName = "oe_tile_normalTex";
        normal.matrixName = "oe_tile_normalTexMatrix";
        normal.defaultTexture = osgEarth::createEmptyNormalMapTexture();
        normal.defaultTexture->setName("terrain default normalmap");

        if (!GLUtils::useNVGL())
            getResources()->reserveTextureImageUnit(normal.unit, "Terrain Normals");
    }

    if (_require.parentTextures)
    {
        SamplerBinding& colorParent = _data.renderBindings[SamplerBinding::COLOR_PARENT];
        colorParent.usage = SamplerBinding::COLOR_PARENT;
        colorParent.samplerName = "oe_layer_texParent";
        colorParent.matrixName = "oe_layer_texParentMatrix";

        if (!GLUtils::useNVGL())
            getResources()->reserveTextureImageUnit(colorParent.unit, "Terrain Parent Color");
    }

    if (_require.landCoverTextures)
    {
        SamplerBinding& landCover = _data.renderBindings[SamplerBinding::LANDCOVER];
        landCover.usage = SamplerBinding::LANDCOVER;
        landCover.samplerName = "oe_tile_landCoverTex";
        landCover.matrixName = "oe_tile_landCoverTexMatrix";
        landCover.defaultTexture = LandCover::createEmptyTexture();
        landCover.defaultTexture->setName("terrain default landcover");
        getOrCreateStateSet()->setDefine("OE_LANDCOVER_TEX", landCover.samplerName);
        getOrCreateStateSet()->setDefine("OE_LANDCOVER_TEX_MATRIX", landCover.matrixName);

        if (!GLUtils::useNVGL())
            getResources()->reserveTextureImageUnit(landCover.unit, "Terrain Land Cover");
    }

    // Apply a default, empty texture to each render binding.
    if (!GLUtils::useNVGL())
    {
        OE_DEBUG << LC << "Render Bindings:\n";
        for (unsigned i = 0; i < _data.renderBindings.size(); ++i)
        {
            SamplerBinding& b = _data.renderBindings[i];
            if (b.isActive())
            {
                _terrain->getOrCreateStateSet()->addUniform(new osg::Uniform(b.samplerName.c_str(), b.unit));
                _terrain->getOrCreateStateSet()->setTextureAttribute(b.unit, b.defaultTexture);
                OE_DEBUG << LC << " > Bound \"" << b.samplerName << "\" to unit " << b.unit << "\n";
            }
        }
    }
}

void
CoreyTerrainEngineNode::dirtyState()
{
    // TODO: perhaps defer this until the next update traversal so we don't
    // reinitialize the state multiple times unnecessarily.
    updateState();
}

void
CoreyTerrainEngineNode::dirtyTerrainOptions()
{
    TerrainEngineNode::dirtyTerrainOptions();

    auto options = getOptions();

    if (_data.textures.valid())
    {
        _data.textures->setMaxTextureSize(options.getMaxTextureSize());
    }

    //_tiles->setNotifyNeighbors(options.getNormalizeEdges() == true);

    //_merger->setMergesPerFrame(options.getMergesPerFrame());

    //jobs::get_pool(ARENA_LOAD_TILE)->set_concurrency(options.getConcurrency());

    getSurfaceStateSet()->getOrCreateUniform(
        "oe_terrain_tess", osg::Uniform::FLOAT)->set(options.getTessellationLevel());

    getSurfaceStateSet()->getOrCreateUniform(
        "oe_terrain_tess_range", osg::Uniform::FLOAT)->set(options.getTessellationRange());

    _pager->setLODMethod(options.getLODMethod());

    if (_pager->getLODMethod() == LODMethod::SCREEN_SPACE)
    {
        _pager->setMinPixels(options.getTilePixelSize());
    }
}

void
CoreyTerrainEngineNode::cacheAllLayerExtentsInMapSRS()
{
    // Only call during update
    LayerVector layers;
    getMap()->getLayers(layers);
    for (LayerVector::const_iterator i = layers.begin();
        i != layers.end();
        ++i)
    {
        cacheLayerExtentInMapSRS(i->get());
    }
}

void
CoreyTerrainEngineNode::cull_traverse(osg::NodeVisitor& nv)
{
    OE_PROFILING_ZONE;

    auto* cv = static_cast<osgUtil::CullVisitor*>(&nv);

    // fetch the persistent data associated with this traversal.
    _persistent.lock();
    TerrainRenderData::PersistentData& pd = _persistent[cv->getCurrentCamera()];
    pd._lastCull = *nv.getFrameStamp();
    _persistent.unlock();

    // Prepare the culler:
    auto& cullData = _data.cullData;
    cullData.reset(cv, pd, _data);
    
    // Assemble the terrain drawables:
    TerrainEngineNode::traverse(nv);

    // If we're using geometry pooling, optimize the drawable forf shared state
    // by sorting the draw commands.
    // Skip if using GL4/indirect rendering. Actually seems to hurt?
    // TODO: benchmark this further to see whether it's worthwhile
    if (!GLUtils::useNVGL())
        //getEngineData()->getGeometryPool()->isEnabled())    
    {
        cullData._renderData.sortDrawCommands();
    }

    // The common stateset for the terrain group:
    cv->pushStateSet(_terrain->getOrCreateStateSet());

    // Push all the layers to draw on to the cull visitor in the order in which
    // they appear in the map.
    LayerDrawable* lastLayer = nullptr;
    unsigned order = 0;
    bool surfaceStateSetPushed = false;
    bool imageLayerStateSetPushed = false;
    int layersDrawn = 0;
    unsigned surfaceDrawOrder = 0;

    std::vector<LayerDrawable*> patchLayers;

    for (auto layerDrawable : cullData._renderData._layerList)
    {
        if (layerDrawable->_tiles.empty() == false &&
            layerDrawable->_patchLayer)
        {
            patchLayers.push_back(layerDrawable);
        }
    }

    for (auto layerDrawable : cullData._renderData._layerList)
    {
        if (!layerDrawable->_tiles.empty())
        {
            // skip patch layers for now
            if (layerDrawable->_patchLayer)
                continue;

            lastLayer = layerDrawable;

            // if this is a RENDERTYPE_TERRAIN_SURFACE, we need to activate either the
            // default surface state set or the image layer state set.
            if (layerDrawable->_renderType == Layer::RENDERTYPE_TERRAIN_SURFACE)
            {
                layerDrawable->_surfaceDrawOrder = surfaceDrawOrder++;

                if (!surfaceStateSetPushed)
                {
                    cv->pushStateSet(_surfaceSS.get());
                    surfaceStateSetPushed = true;
                }

                if (layerDrawable->_imageLayer || layerDrawable->_layer == nullptr)
                {
                    if (!imageLayerStateSetPushed)
                    {
                        cv->pushStateSet(_imageLayerSS.get());
                        imageLayerStateSetPushed = true;
                    }
                }
                else
                {
                    if (imageLayerStateSetPushed)
                    {
                        cv->popStateSet();
                        imageLayerStateSetPushed = false;
                    }
                }
            }

            else
            {
                if (imageLayerStateSetPushed)
                {
                    cv->popStateSet();
                    imageLayerStateSetPushed = false;
                }
                if (surfaceStateSetPushed)
                {
                    cv->popStateSet();
                    surfaceStateSetPushed = false;
                }
            }

            if (layerDrawable->_layer)
            {
                layerDrawable->_layer->apply(layerDrawable, cv);
            }
            else
            {
                layerDrawable->accept(*cv);
            }

            ++layersDrawn;
        }
    }

    // clear out the statesets:
    if (imageLayerStateSetPushed)
    {
        cv->popStateSet();
        imageLayerStateSetPushed = false;
    }

    if (surfaceStateSetPushed)
    {
        cv->popStateSet();
        surfaceStateSetPushed = false;
    }

    // patch layers go last
    for (auto layerDrawable : patchLayers)
    {
        lastLayer = layerDrawable;

        TileBatch batch(nullptr);
        for (auto& tile : layerDrawable->_tiles)
            batch._tiles.push_back(&tile);

        if (layerDrawable->_patchLayer->getStateSet())
            cv->pushStateSet(layerDrawable->_patchLayer->getStateSet());

        if (layerDrawable->_patchLayer->getCullCallback()) // backwards compat
            layerDrawable->_patchLayer->apply(layerDrawable, cv);
        else
            layerDrawable->_patchLayer->cull(batch, *cv);

        if (layerDrawable->_patchLayer->getStateSet())
            cv->popStateSet();
    }

    // The last layer to render must clear up the OSG state,
    // otherwise it will be corrupt and can lead to crashing.
    if (lastLayer)
    {
        lastLayer->_clearOsgState = true;
    }

    // pop the common terrain state set
    cv->popStateSet();

    // If the culler found any orphaned data, we need to update the render model
    // during the next update cycle.
    if (cullData._orphanedPassesDetected > 0u)
    {
        _renderModelUpdateRequired = true;
        OE_DEBUG << LC << "Detected " << cullData._orphanedPassesDetected << " orphaned rendering passes\n";
    }

    // Finally, if the cull traversal spotted any stale tiles, add them to the
    // consolidated stale tiles list. We will process that set during the next
    // update traversal.
    if (cullData._staleTiles.size() > 0)
    {
        std::lock_guard<std::mutex> lock(_staleTiles_mutex);
        _staleTiles.insert(cullData._staleTiles.begin(), cullData._staleTiles.end());
    }
    cullData._staleTiles.clear();
}

void
CoreyTerrainEngineNode::update_traverse(osg::NodeVisitor& nv)
{
    if (_renderModelUpdateRequired)
    {
        PurgeOrphanedLayers visitor(getMap(), _data.renderBindings);
        _terrain->accept(visitor);
        _renderModelUpdateRequired = false;
    }

    // Called once on the first update pass to ensure that all existing
    // layers have their extents cached properly
    if (_data.cachedLayerExtentsComputeRequired)
    {
        cacheAllLayerExtentsInMapSRS();
        _data.cachedLayerExtentsComputeRequired = false;
    }
    else
    {
        // Update the cached layer extents as necessary.
        osg::ref_ptr<const Layer> layer;
        for (auto& layerExtent : _data.cachedLayerExtents)
        {
            layerExtent.second._layer.lock(layer);
            if (layer.valid() && layer->getRevision() > layerExtent.second._revision)
            {
                layerExtent.second._extent = _map->getProfile()->clampAndTransformExtent(layer->getExtent());
                layerExtent.second._revision = layer->getRevision();
            }
        }
    }

    // Call update() on all open layers
    LayerVector layers;
    _map->getOpenLayers(layers);
    for (auto& layer : layers)
    {
        layer->update(nv);
    }

    // check on the persistent data cache
    _persistent.lock();
    const osg::FrameStamp* fs = nv.getFrameStamp();
    for (auto iter : _persistent)
    {
        if (fs->getFrameNumber() - iter.second._lastCull.getFrameNumber() > 60)
        {
            _persistent.erase(iter.first);
            OE_DEBUG << LC << "Releasing orphaned view data" << std::endl;
            break;
        }
    }
    _persistent.unlock();

    // traverse the texture arena since it's not in the scene graph.
    if (_data.textures.valid())
    {
        _data.textures->update(nv);
    }

    // Queue update requests for any stale tiles.
    {
        std::lock_guard<std::mutex> lock(_staleTiles_mutex);
        
        osg::observer_ptr<CoreyTerrainEngineNode> weak_this = this;

        for (auto& key : _staleTiles)
        {
            jobs::context c;
            c.pool = jobs::get_pool(TERRAIN_JOB_POOL);
            c.name = "update tilenode " + key.str();
            float lod = key.getLOD();
            c.priority = [lod]() { return lod; };

            auto update = [weak_this, key](Cancelable& c)
                {
                    osg::ref_ptr<TerrainTileModel> result;
                    osg::ref_ptr<CoreyTerrainEngineNode> engine;
                    if (weak_this.lock(engine))
                    {
                        auto tilenode = engine->_data.getTileNode(key);
                        if (tilenode.valid() && tilenode->_partialDataModel.valid())
                        {
                            // make a shallow copy of the current partial data model,
                            // i.e. the last model that the tile loaded.
                            osg::ref_ptr<TerrainTileModel> model_to_update =
                                new TerrainTileModel(*tilenode->_partialDataModel.get());

                            //OE_INFO << LC << "Updating: " << key.str() << std::endl;
                            bool updated = false;
                            result = engine->_tileModelDB.createAndStore(key, [&](const TileKey& key)
                                {
                                    updated = engine->_tileModelFactory->updateTileModel(
                                        model_to_update, engine->_data.map, {}, engine->_require, nullptr);
                                    return model_to_update.get();
                                });
                        }
                    }
                    return result;
                };

            _loadQueue.emplace_back(jobs::dispatch(update, c));
        }
        _staleTiles.clear();
    }

    // process the merge queue
    unsigned count = std::max(4u, getOptions().getMergesPerFrame());

    for(auto iter = _loadQueue.begin(); iter != _loadQueue.end() && count > 0u;)
    {
        auto& result = *iter;
        if (result.available())
        {
            merge(result.value().get());
            iter = _loadQueue.erase(iter);
            --count;
        }
        else if (result.canceled())
        {
            OE_INFO << LC << "Canceled tile load" << std::endl;
            iter = _loadQueue.erase(iter);
        }
        else
        {
            break; // so we keep everything in order!!

            //OE_INFO << LC << "Waiting on tile load" << std::endl;
            //++iter;
        }
    }
}

void
CoreyTerrainEngineNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        if (!_updatedThisFrame.exchange(true))
        {
            _clock.update();
            update_traverse(nv);
            TerrainEngineNode::traverse(nv);
        }
    }

#if 1
    else if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        _updatedThisFrame.exchange(false);
        _clock.cull();
        cull_traverse(nv);
    }
#endif
    else
    {
        TerrainEngineNode::traverse(nv);
    }
}

void
CoreyTerrainEngineNode::onMapModelChanged(const MapModelChange& change)
{
    if (change.getAction() == MapModelChange::BEGIN_BATCH_UPDATE)
    {
        _batchUpdateInProgress = true;
    }

    else if (change.getAction() == MapModelChange::END_BATCH_UPDATE)
    {
        _batchUpdateInProgress = false;

        if (_refreshRequired)
            refresh();

        if (_stateUpdateRequired)
            updateState();
    }

    else
    {

        // dispatch the change handler
        if (change.getLayer())
        {
            // then apply the actual change:
            switch (change.getAction())
            {
            case MapModelChange::ADD_LAYER:
            case MapModelChange::OPEN_LAYER:
                addLayer(change.getLayer());
                break;

            case MapModelChange::REMOVE_LAYER:
            case MapModelChange::CLOSE_LAYER:
                if (change.getImageLayer())
                    removeImageLayer(change.getImageLayer());
                else if (change.getElevationLayer() || change.getConstraintLayer())
                    removeElevationLayer(change.getLayer());
                break;

            case MapModelChange::MOVE_LAYER:
                if (change.getElevationLayer())
                    moveElevationLayer(change.getElevationLayer());
                break;

            default:
                break;
            }
        }
    }
}

void
CoreyTerrainEngineNode::cacheLayerExtentInMapSRS(Layer* layer)
{
    OE_SOFT_ASSERT_AND_RETURN(layer != nullptr, void());

    // Store the layer's extent in the map's SRS:
    LayerExtent& le = _data.cachedLayerExtents[layer->getUID()];
    le._layer = layer;
    le._extent = getMap()->getProfile()->clampAndTransformExtent(layer->getExtent());
}

void
CoreyTerrainEngineNode::addLayer(Layer* layer)
{
    if (layer)
    {
        if (layer->isOpen())
        {
            if (layer->getRenderType() == Layer::RENDERTYPE_TERRAIN_SURFACE)
                addSurfaceLayer(layer);
            else if (dynamic_cast<ElevationLayer*>(layer) || dynamic_cast<TerrainConstraintLayer*>(layer))
                addElevationLayer(layer);
        }

        cacheLayerExtentInMapSRS(layer);
    }
}

void
CoreyTerrainEngineNode::addSurfaceLayer(Layer* layer)
{
    if (layer && layer->isOpen())
    {
        ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer);
        if (imageLayer)
        {
            // for a shared layer, allocate a shared image unit if necessary.
            if (imageLayer->isShared())
            {
                if (!imageLayer->sharedImageUnit().isSet() && !GLUtils::useNVGL())
                {
                    int temp;
                    if (getResources()->reserveTextureImageUnit(temp, imageLayer->getName().c_str()))
                    {
                        imageLayer->sharedImageUnit() = temp;
                        //OE_INFO << LC << "Image unit " << temp << " assigned to shared layer " << imageLayer->getName() << std::endl;
                    }
                    else
                    {
                        OE_WARN << LC << "Insufficient GPU image units to share layer " << imageLayer->getName() << std::endl;
                    }
                }

                // Build a sampler binding for the shared layer.
                if (imageLayer->sharedImageUnit().isSet() || GLUtils::useNVGL())
                {
                    // Find the next empty SHARED slot:
                    unsigned newIndex = SamplerBinding::SHARED;
                    while (_data.renderBindings[newIndex].isActive())
                        ++newIndex;

                    // Put the new binding there:
                    SamplerBinding& newBinding = _data.renderBindings[newIndex];
                    newBinding.usage = SamplerBinding::SHARED;
                    newBinding.sourceUID = imageLayer->getUID();
                    newBinding.unit = imageLayer->sharedImageUnit().get();
                    newBinding.samplerName = imageLayer->getSharedTextureUniformName();
                    newBinding.matrixName = imageLayer->getSharedTextureMatrixUniformName();

                    OE_INFO << LC
                        << "Shared Layer \"" << imageLayer->getName() << "\" : sampler=\"" << newBinding.samplerName << "\", "
                        << "matrix=\"" << newBinding.matrixName << "\", "
                        << "unit=" << newBinding.unit << "\n";

                    // Install an empty texture for this binding at the top of the graph, so that
                    // a texture is always defined even when the data source supplies no real data.
                    if (newBinding.isActive() && !GLUtils::useNVGL())
                    {
                        osg::ref_ptr<osg::Texture> tex;
                        if (osg::Image* emptyImage = imageLayer->getEmptyImage())
                        {
                            if (emptyImage->r() > 1)
                            {
                                tex = ImageUtils::makeTexture2DArray(emptyImage);
                            }
                            else
                            {
                                tex = new osg::Texture2D(emptyImage);
                            }
                        }
                        else
                        {
                            tex = new osg::Texture2D(ImageUtils::createEmptyImage(1, 1));
                        }
                        tex->setName("default:" + imageLayer->getName());
                        tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
                        _terrain->getOrCreateStateSet()->addUniform(new osg::Uniform(newBinding.samplerName.c_str(), newBinding.unit));
                        _terrain->getOrCreateStateSet()->setTextureAttribute(newBinding.unit, tex.get(), 1);
                        OE_INFO << LC << "Bound shared sampler " << newBinding.samplerName << " to unit " << newBinding.unit << std::endl;
                    }
                }
            }
        }

        else
        {
            // non-image tile layer.
        }

        if (_terrain)
        {
            // Update the existing render models, and trigger a data reload.
            // Later we can limit the reload to an update of only the new data.
            std::vector<const Layer*> layers;
            layers.push_back(layer);
            invalidateRegion(layers, GeoExtent::INVALID, 0u, INT_MAX);
        }

        updateState();
    }
}


void
CoreyTerrainEngineNode::removeImageLayer(ImageLayer* layerRemoved)
{
    if (layerRemoved)
    {
        // release its layer drawable
        _persistent.scoped_lock([&]() {
            for (auto& e : _persistent)
                e.second._drawables.erase(layerRemoved);
            });

        // for a shared layer, release the shared image unit.
        if (layerRemoved->isOpen() && layerRemoved->isShared())
        {
            if (layerRemoved->sharedImageUnit().isSet())
            {
                getResources()->releaseTextureImageUnit(*layerRemoved->sharedImageUnit());
                layerRemoved->sharedImageUnit().unset();
            }

            // Remove from RenderBindings (mark as unused)
            for (unsigned i = 0; i < _data.renderBindings.size(); ++i)
            {
                SamplerBinding& binding = _data.renderBindings[i];
                if (binding.isActive() && binding.sourceUID == layerRemoved->getUID())
                {
                    OE_INFO << LC << "Binding (" << binding.samplerName << " unit " << binding.unit << ") cleared\n";
                    binding.usage.clear();
                    binding.unit = -1;

                    // Request an update to reset the shared sampler in the scene graph
                    // GW: running this anyway below (PurgeOrphanedLayers), so no need..?
                    _renderModelUpdateRequired = true;
                }
            }
        }

        updateState();
    }

    if (_terrain)
    {
        // Run the update visitor, which will clean out any rendering passes
        // associated with the layer we just removed. This would happen
        // automatically during cull/update anyway, but it's more efficient
        // to do it all at once.
        PurgeOrphanedLayers updater(getMap(), _data.renderBindings);
        _terrain->accept(updater);
    }

    //OE_INFO << LC << " Updated " << updater._count << " tiles\n";
}

void
CoreyTerrainEngineNode::addElevationLayer(Layer* layer)
{
    if (layer && layer->isOpen())
    {
        std::vector<const Layer*> layers;
        layers.push_back(layer);
        invalidateRegion(layers, GeoExtent::INVALID, 0u, INT_MAX);
    }
}

void
CoreyTerrainEngineNode::removeElevationLayer(Layer* layer)
{
    // only need to refresh is the elevation layer is visible.
    if (layer)
    {
        std::vector<const Layer*> layers;
        layers.push_back(layer);
        invalidateRegion(layers, GeoExtent::INVALID, 0u, INT_MAX);
    }
}

void
CoreyTerrainEngineNode::moveElevationLayer(Layer* layer)
{
    if (layer && layer->isOpen())
    {
        std::vector<const Layer*> layers;
        layers.push_back(layer);
        invalidateRegion(layers, GeoExtent::INVALID, 0u, INT_MAX);
    }
}

// Generates the main shader code for rendering the terrain.
void
CoreyTerrainEngineNode::updateState()
{
    if (_batchUpdateInProgress)
    {
        _stateUpdateRequired = true;
    }
    else
    {
        // Load up the appropriate shader package:
        CoreyShaders& shaders = CoreyShadersFactory::get(GLUtils::useNVGL());

        auto options = getOptions();

        auto* terrainSS = _terrain->getOrCreateStateSet();

        // State that affects any terrain layer (surface, patch, other)
        // AND compute shaders
        {
            // activate standard mix blending.
            terrainSS->setAttributeAndModes(
                new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
                osg::StateAttribute::ON);

            VirtualProgram* terrainVP = VirtualProgram::getOrCreate(terrainSS);
            shaders.load(terrainVP, shaders.sdk());

            // GL4 rendering?
            if (GLUtils::useNVGL())
            {
                terrainSS->setDefine("OE_USE_GL4");
            }

            // vertex-dimension of each standard terrain tile.
            terrainSS->setDefine("OE_TILE_SIZE",
                std::to_string(options.getTileSize()));

            // uniform that conveys the layer UID to the shaders; necessary
            // for per-layer branching (like color filters)
            // UID -1 => no image layer (no texture)
            terrainSS->addUniform(new osg::Uniform(
                "oe_layer_uid", (int)-1));

            // uniform that conveys the render order, since the shaders
            // need to know which is the first layer in order to blend properly
            terrainSS->addUniform(new osg::Uniform(
                "oe_layer_order", (int)0));

            if (_require.elevationTextures)
            {
                // Compute an elevation texture sampling scale/bias so we sample elevation data on center
                // instead of on edge (as we do with color, etc.)
                float bias = 0.5; // getEngineData()->getUseTextureBorder() ? 1.5 : 0.5;
                float size = (float)ELEVATION_TILE_SIZE;

                terrainSS->addUniform(new osg::Uniform(
                    "oe_tile_elevTexelCoeff",
                    osg::Vec2f((size - (2.0 * bias)) / size, bias / size)));
            }
        }


        // State that affects surface layers only:
        {
            // required for multipass tile rendering to work
            _surfaceSS->setAttributeAndModes(
                new osg::Depth(osg::Depth::LEQUAL, 0, 1, true));

            // backface culling on
            _surfaceSS->setAttributeAndModes(
                new osg::CullFace(), osg::StateAttribute::ON);

            // untextured terrain skin color
            _surfaceSS->addUniform(new osg::Uniform(
                "oe_terrain_color", options.getColor()));

            // vertical offset of the terrain verts (cloud layer e.g.)
            _surfaceSS->addUniform(new osg::Uniform(
                "oe_terrain_altitude", (float)0.0f));

            // exists do you can override it from above
            _surfaceSS->setDefine("OE_TERRAIN_RENDER_IMAGERY");

            // RENDERTYPE_TERRAIN_SURFACE shaders
            VirtualProgram* surfaceVP = VirtualProgram::getOrCreate(_surfaceSS.get());
            shaders.load(surfaceVP, shaders.vert());
            shaders.load(surfaceVP, shaders.elevation());
            shaders.load(surfaceVP, shaders.normal_map());

            // GPU tessellation:
            if (options.getGPUTessellation())
            {
                shaders.load(surfaceVP, shaders.tessellation());

                // Default tess level
                _surfaceSS->addUniform(new osg::Uniform("oe_terrain_tess", options.getTessellationLevel()));
                _surfaceSS->addUniform(new osg::Uniform("oe_terrain_tess_range", options.getTessellationRange()));

#ifdef HAVE_PATCH_PARAMETER
                // backwards compatibility
                _surfaceSS->setAttributeAndModes(new osg::PatchParameter(3));
#endif
            }

            // Elevation
            // Corey does NOT elevate the terrain in the shader
            if (_require.elevationTextures)
            {
                _surfaceSS->setDefine("OE_TERRAIN_RENDER_ELEVATION");
            }

            // Normal mapping
            if (_require.normalTextures)
            {
                _surfaceSS->setDefine("OE_TERRAIN_RENDER_NORMAL_MAP");
            }

            // Imagery blending
            if (options.getEnableBlending())
            {
                _surfaceSS->setDefine("OE_TERRAIN_BLEND_IMAGERY");
            }

            // Compressed normal maps
            if (options.getCompressNormalMaps())
            {
                _surfaceSS->setDefine("OE_COMPRESSED_NORMAL_MAP");
            }

            // Morphing (imagery and terrain)
            if (_morphingSupported)
            {
                if ((options.getMorphTerrain() && _morphTerrainSupported) ||
                    (options.getMorphImagery()))
                {
                    // GL4 morphing is built into another shader (vert.GL4.glsl)
                    if (!GLUtils::useNVGL())
                        shaders.load(surfaceVP, shaders.morphing());

                    if ((options.getMorphTerrain() && _morphTerrainSupported))
                    {
                        _surfaceSS->setDefine("OE_TERRAIN_MORPH_GEOMETRY");
                    }
                    if (options.getMorphImagery())
                    {
                        _surfaceSS->setDefine("OE_TERRAIN_MORPH_IMAGERY");
                    }
                }
            }

            // Shadowing
            if (options.getCastShadows())
            {
                _surfaceSS->setDefine("OE_TERRAIN_CAST_SHADOWS");
            }

            // special object ID that denotes the terrain surface.
            _surfaceSS->addUniform(new osg::Uniform(
                Registry::objectIndex()->getObjectIDUniformName().c_str(),
                OSGEARTH_OBJECTID_TERRAIN));
        }

        // STATE for image layers
        VirtualProgram* vp = VirtualProgram::getOrCreate(_imageLayerSS.get());
        shaders.load(vp, shaders.imagelayer());

        // The above shader will integrate opacity itself.
        _imageLayerSS->setDefine("OE_SELF_MANAGE_LAYER_OPACITY");

        _stateUpdateRequired = false;
    }
}

osg::Node*
CoreyTerrainEngineNode::createStandaloneTile(
    const TerrainTileModel* model,
    int createTileFlags,
    unsigned referenceLOD,
    const TileKey& subRegion)
{
    return nullptr;
    //CreateTileImplementation impl;
    //return impl.createTile(getEngineData(), model, createTileFlags, referenceLOD, subRegion);
}

void
CoreyTerrainEngineNode::merge(TerrainTileModel* model)
{
    if (model)
    {
        //OE_INFO << "Merging " << model->key.str() << std::endl;

        auto tilenode = _data.getTileNode(model->key);
        if (tilenode.valid())
        {
            auto parent = _data.getTileNode(tilenode->_key.createParentKey());
            auto parentData = parent.valid() ? parent->_fullDataModel.get() : nullptr;
            tilenode->set(model, parentData, _data);
        }
    }
}

#if 0
void
CoreyTerrainEngineNode::refreshTileNode(const TileKey& key)
{
    // look up the tile node:
    osg::ref_ptr<TileNode> tilenode;
    {
        std::lock_guard<std::mutex> lock(_EngineData->tilesMutex);
        auto i = _EngineData->tiles.find(key);
        if (i != _EngineData->tiles.end())
        {
            tilenode = i->second;
        }
    }

    if (!tilenode)
        return;

    OE_SOFT_ASSERT_AND_RETURN(tilenode->_partialDataModel.valid(), void());

    auto model = tilenode->_partialDataModel;

    // make a shallow copy of the current partial data model,
    // i.e. the last model that the tile loaded.
    osg::ref_ptr<TerrainTileModel> model_to_update =
        new TerrainTileModel(*model.get());

    OE_INFO << LC << "Updating: " << key.str() << std::endl;
    bool updated = false;
    _tileModelDB.createAndStore(key, [&](const TileKey& key)
        {
            updated = _tileModelFactory->updateTileModel(
                model_to_update, _EngineData->getMap(), {}, _require, nullptr);
            return model_to_update.get();
        });

    if (updated)
    {
        osg::observer_ptr<TileNode> tilenode_weak;
        auto merge_function = [this, tilenode_weak, model_to_update]()
            {
                osg::ref_ptr<TileNode> tilenode;
                if (tilenode_weak.lock(tilenode))
                {
                    osg::ref_ptr<TileNode> parent;

                    auto parentKey = tilenode->_key.createParentKey();
                    if (parentKey.valid())
                    {
                        std::lock_guard<std::mutex> lock(_EngineData->tilesMutex);
                        parent = _EngineData->tiles[parentKey];
                        if (!parent.valid())
                            _EngineData->tiles.erase(parentKey);
                    }

                    auto parentData = parent.valid() ? parent->_fullDataModel.get() : nullptr;
                    tilenode->set(model_to_update.get(), parentData, _EngineData.get());
                }
            };

        std::lock_guard<std::mutex> lock(_mergeQueue_mutex);
        _mergeQueue.emplace(std::move(merge_function));
    }

    tilenode->_dirty = false;
}
#endif
