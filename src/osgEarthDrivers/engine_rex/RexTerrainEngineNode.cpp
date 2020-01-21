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
#include "RexTerrainEngineNode"
#include "Shaders"
#include "TerrainCuller"
#include "GeometryPool"
#include "CreateTileImplementation"

#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/MapModelChange>
#include <osgEarth/Progress>
#include <osgEarth/ShaderLoader>
#include <osgEarth/Utils>
#include <osgEarth/ObjectIndex>

#include <osg/Version>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/CullFace>
#include <osg/ValueObject>

#include <cstdlib> // for getenv

#define LC "[RexTerrainEngineNode] "

using namespace osgEarth::REX;
using namespace osgEarth;

#define DEFAULT_MAX_LOD 19u

//#define PROFILE

//------------------------------------------------------------------------

namespace
{
    // adapter that lets RexTerrainEngineNode listen to Map events
    struct RexTerrainEngineNodeMapCallbackProxy : public MapCallback
    {
        RexTerrainEngineNodeMapCallbackProxy(RexTerrainEngineNode* node) : _node(node) { }
        osg::observer_ptr<RexTerrainEngineNode> _node;

        void onMapModelChanged( const MapModelChange& change ) {
            osg::ref_ptr<RexTerrainEngineNode> node;
            if ( _node.lock(node) )
                node->onMapModelChanged( change );
        }
    };


    /**
     * Run this visitor whenever you remove a layer, so that each
     * TileNode can update its render model and get rid of passes
     * that no longer exist.
     */
    struct UpdateRenderModels : public osg::NodeVisitor
    {
        const Map* _map;
        const RenderBindings& _bindings;
        unsigned _count;
        bool _reload;
        std::set<UID> _layersToLoad;

        UpdateRenderModels(const Map* map, RenderBindings& bindings) : _map(map), _bindings(bindings), _count(0u), _reload(false)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }

        std::set<UID>& layersToLoad()
        {
            return _layersToLoad;
        }

        void setReloadData(bool value)
        {
            _reload = value;
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
            TileRenderModel& model = tileNode.renderModel();
            for (int p = 0; p < model._passes.size(); ++p)
            {
                RenderingPass& pass = model._passes[p];

                // if the map doesn't contain a layer with a matching UID,
                // or if the layer is now disabled, remove it from the render model.
                Layer* layer = _map->getLayerByUID(pass.sourceUID());
                if (layer == NULL || layer->getEnabled() == false)
                {
                    model._passes.erase(model._passes.begin()+p);
                    --p;
                    _count++;
                }
            }

            // For shared samplers we need to refresh the list if one of them
            // goes inactive (as is the case when removing a shared layer)
            tileNode.refreshSharedSamplers(_bindings);

            // todo. Might be better to use a Revision here though.
            if (_reload)
            {
                tileNode.setDirty(true);
            }

            else if (!_layersToLoad.empty())
            {
                tileNode.refreshLayers(_layersToLoad);
            }

        }
    };

}

//------------------------------------------------------------------------

RexTerrainEngineNode::RexTerrainEngineNode() :
TerrainEngineNode     ( ),
_terrain              ( 0L ),
_batchUpdateInProgress( false ),
_refreshRequired      ( false ),
_stateUpdateRequired  ( false ),
_renderModelUpdateRequired( false ),
_rasterizer(0L),
_morphTerrainSupported(true)
{
    // Necessary for pager object data
    this->setName("osgEarth.RexTerrainEngineNode");

    // unique ID for this engine:
    _uid = Registry::instance()->createUID();

    // always require elevation.
    _requireElevationTextures = true;

    // install an elevation callback so we can update elevation data
    //_elevationCallback = new ElevationChangedCallback( this );

    // static shaders.
    if ( Registry::capabilities().supportsGLSL() )
    {
        osg::StateSet* stateset = getOrCreateStateSet();
        stateset->setName("RexTerrainEngineNode");
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setName("RexTerrainEngineNode");
        vp->setIsAbstract(true);    // cannot run by itself, requires additional children
        Shaders package;
        package.load(vp, package.SDK);
    }

    // TODO: replace with a "renderer" object that can return statesets
    // for different layer types, or something.
    _surfaceStateSet = new osg::StateSet();
    _surfaceStateSet->setName("Surface");

    _terrain = new osg::Group();
    addChild(_terrain.get());

    // force an update traversal in order to compute layer extents.
    _cachedLayerExtentsComputeRequired = true;
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}

RexTerrainEngineNode::~RexTerrainEngineNode()
{
    OE_DEBUG << LC << "~RexTerrainEngineNode\n";
}

void
RexTerrainEngineNode::resizeGLObjectBuffers(unsigned maxSize)
{
    TerrainEngineNode::resizeGLObjectBuffers(maxSize);

    getStateSet()->resizeGLObjectBuffers(maxSize);

    _terrain->getStateSet()->resizeGLObjectBuffers(maxSize);

    _imageLayerStateSet.get()->resizeGLObjectBuffers(maxSize);

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
RexTerrainEngineNode::releaseGLObjects(osg::State* state) const
{
    if (_imageLayerStateSet.valid())
    {
        _imageLayerStateSet.get()->releaseGLObjects(state);
    }

    if (_geometryPool.valid())
    {
        _geometryPool->clear();
    }

    TerrainEngineNode::releaseGLObjects(state);
}

void
RexTerrainEngineNode::setMap(const Map* map, const TerrainOptions& inOptions)
{
    if (!map) return;

    _map = map;

    // Invoke the base class first:
    TerrainEngineNode::setMap(map, inOptions);

    // merge in the custom options:
    _terrainOptions = &inOptions;
    //_terrainOptions.merge( options );

    _morphingSupported = true;
    if (options().rangeMode() == osg::LOD::PIXEL_SIZE_ON_SCREEN)
    {
        OE_INFO << LC << "Range mode = pixel size; pixel tile size = " << options().tilePixelSize().get() << std::endl;

        // force morphing off for PSOS mode
        _morphingSupported = false;
    }

    // morphing imagery LODs requires we bind parent textures to their own unit.
    if (options().morphImagery() == true && _morphingSupported)
    {
        _requireParentTextures = true;
    }

    // Terrain morphing doesn't work in projected maps:
    if (map->getSRS()->isProjected())
    {
        _morphTerrainSupported = false;
    }

    // Check for normals debugging.
    if (::getenv("OSGEARTH_DEBUG_NORMALS"))
        getOrCreateStateSet()->setDefine("OE_DEBUG_NORMALS");
    else
        if (getStateSet()) getStateSet()->removeDefine("OE_DEBUG_NORMALS");

    // check for normal map generation (required for lighting).
    if (options().normalMaps() == true )
    {
        this->_requireNormalTextures = true;
    }

    // don't know how to set this up so just do it
    this->_requireLandCoverTextures = true;

    // ensure we get full coverage at the first LOD.
    this->_requireFullDataAtFirstLOD = true;

    // A shared registry for tile nodes in the scene graph. Enable revision tracking
    // if requested in the options. Revision tracking lets the registry notify all
    // live tiles of the current map revision so they can inrementally update
    // themselves if necessary.
    _liveTiles = new TileNodeRegistry("live");
    _liveTiles->setMapRevision(map->getDataModelRevision());
    _liveTiles->setNotifyNeighbors(options().normalizeEdges() == true);

    // A resource releaser that will call releaseGLObjects() on expired objects.
    _releaser = new ResourceReleaser();
    this->addChild(_releaser.get());

    // A shared geometry pool.
    _geometryPool = new GeometryPool(options());
    _geometryPool->setReleaser( _releaser.get());
    this->addChild( _geometryPool.get() );

    // Make a tile loader
    PagerLoader* loader = new PagerLoader( this );
    loader->setNumLODs(options().maxLOD().getOrUse(DEFAULT_MAX_LOD));
    loader->setMergesPerFrame(options().mergesPerFrame().get() );

#if 0
    for (std::vector<TerrainOptions::LODOptions>::const_iterator i = _terrainOptions.lods().begin(); i != _terrainOptions.lods().end(); ++i) {
        if (i->_lod.isSet()) {
            loader->setLODPriorityScale(i->_lod.get(), i->_priorityScale.getOrUse(1.0f));
            loader->setLODPriorityOffset(i->_lod.get(), i->_priorityOffset.getOrUse(0.0f));
        }
    }
#endif

    _loader = loader;
    this->addChild( _loader.get() );

    // if the envvar for tile expiration is set, override the options setting
    unsigned expirationThreshold = options().expirationThreshold().get();
    const char* val = ::getenv("OSGEARTH_EXPIRATION_THRESHOLD");
    if (val)
    {
        expirationThreshold = as<unsigned>(val, options().expirationThreshold().get());
        OE_INFO << LC << "Expiration threshold set by env var = " << options().expirationThreshold().get() << "\n";
    }

    // Make a tile unloader
    _unloader = new UnloaderGroup( _liveTiles.get() );
    _unloader->setThreshold(expirationThreshold);
    _unloader->setReleaser(_releaser.get());
    this->addChild( _unloader.get() );

    // Tile rasterizer in case we need one
    _rasterizer = new TileRasterizer();
    this->addChild( _rasterizer );

    // Initialize the core render bindings.
    setupRenderBindings();

    // install a layer callback for processing further map actions:
    map->addMapCallback( new RexTerrainEngineNodeMapCallbackProxy(this) );

    // Prime with existing layers:
    _batchUpdateInProgress = true;

    LayerVector layers;
    map->getLayers(layers);
    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
        addLayer(i->get());

    _batchUpdateInProgress = false;

    // Establish a new engine context
    _engineContext = new EngineContext(
        map,
        this, // engine
        _geometryPool.get(),
        _loader.get(),
        _unloader.get(),
        _rasterizer,
        _liveTiles.get(),
        _renderBindings,
        options(),
        _selectionInfo);

    // Calculate the LOD morphing parameters:
    unsigned maxLOD = options().maxLOD().getOrUse(DEFAULT_MAX_LOD);

    _selectionInfo.initialize(
        0u, // always zero, not the terrain options firstLOD
        osg::minimum(options().maxLOD().get(), maxLOD ),
        map->getProfile(),
        options().minTileRangeFactor().get(),
        true); // restrict polar subdivision for geographic maps

    // set up the initial graph
    refresh();

    // now that we have a map, set up to recompute the bounds
    dirtyBound();
}


osg::BoundingSphere
RexTerrainEngineNode::computeBound() const
{
    return TerrainEngineNode::computeBound();
}

void
RexTerrainEngineNode::invalidateRegion(const GeoExtent& extent,
                                       unsigned         minLevel,
                                       unsigned         maxLevel)
{
    if ( _liveTiles.valid() )
    {
        GeoExtent extentLocal = extent;

        if ( extent.isValid() && !extent.getSRS()->isHorizEquivalentTo(this->getMap()->getSRS()) )
        {
            extent.transform(this->getMap()->getSRS(), extentLocal);
        }

        std::set<UID> layers;
        _liveTiles->setDirty(extentLocal, minLevel, maxLevel, layers);
    }
}

void
RexTerrainEngineNode::invalidateLayerRegion(const Layer* layer,
                                            const GeoExtent& extent,
                                            unsigned minLevel,
                                            unsigned maxLevel)
{
    if ( _liveTiles.valid() )
    {
        GeoExtent extentLocal = extent;

        if ( extent.isValid() && !extent.getSRS()->isHorizEquivalentTo(this->getMap()->getSRS()) )
        {
            extent.transform(this->getMap()->getSRS(), extentLocal);
        }

        std::set<UID> layers;
        if (layer)
        {
            layers.insert(layer->getUID());
        }

        _liveTiles->setDirty(extentLocal, minLevel, maxLevel, layers);
    }
}

void
RexTerrainEngineNode::refresh(bool forceDirty)
{
    if ( _batchUpdateInProgress )
    {
        _refreshRequired = true;
    }
    else
    {
        dirtyTerrain();

        _refreshRequired = false;
    }
}

osg::StateSet*
RexTerrainEngineNode::getSurfaceStateSet()
{
    return _surfaceStateSet.get();
}

void
RexTerrainEngineNode::setupRenderBindings()
{
    // Release any pre-existing bindings:
    for (unsigned i = 0; i < _renderBindings.size(); ++i)
    {
        SamplerBinding& b = _renderBindings[i];
        if (b.isActive())
        {
            getResources()->releaseTextureImageUnit(b.unit());
        }
    }
    _renderBindings.clear();

    // "SHARED" is the start of shared layers, so we always want the bindings
    // vector to be at least that size.
    _renderBindings.resize(SamplerBinding::SHARED);

    SamplerBinding& color = _renderBindings[SamplerBinding::COLOR];
    color.usage()       = SamplerBinding::COLOR;
    color.samplerName() = "oe_layer_tex";
    color.matrixName()  = "oe_layer_texMatrix";
    getResources()->reserveTextureImageUnit( color.unit(), "Terrain Color" );

    SamplerBinding& elevation = _renderBindings[SamplerBinding::ELEVATION];
    elevation.usage()       = SamplerBinding::ELEVATION;
    elevation.samplerName() = "oe_tile_elevationTex";
    elevation.matrixName()  = "oe_tile_elevationTexMatrix";
    if (this->elevationTexturesRequired())
        getResources()->reserveTextureImageUnit( elevation.unit(), "Terrain Elevation" );

    SamplerBinding& normal = _renderBindings[SamplerBinding::NORMAL];
    normal.usage()       = SamplerBinding::NORMAL;
    normal.samplerName() = "oe_tile_normalTex";
    normal.matrixName()  = "oe_tile_normalTexMatrix";
    if (this->normalTexturesRequired())
        getResources()->reserveTextureImageUnit( normal.unit(), "Terrain Normals" );

    SamplerBinding& colorParent = _renderBindings[SamplerBinding::COLOR_PARENT];
    colorParent.usage()       = SamplerBinding::COLOR_PARENT;
    colorParent.samplerName() = "oe_layer_texParent";
    colorParent.matrixName()  = "oe_layer_texParentMatrix";
    if (this->parentTexturesRequired())
        getResources()->reserveTextureImageUnit(colorParent.unit(), "Terrain Parent Color");

    SamplerBinding& landCover = _renderBindings[SamplerBinding::LANDCOVER];
    landCover.usage()       = SamplerBinding::LANDCOVER;
    landCover.samplerName() = "oe_tile_landCoverTex";
    landCover.matrixName()  = "oe_tile_landCoverTexMatrix";
    if (this->landCoverTexturesRequired())
        getResources()->reserveTextureImageUnit(landCover.unit(), "Terrain Land Cover");
    getOrCreateStateSet()->setDefine("OE_LANDCOVER_TEX", landCover.samplerName());
    getOrCreateStateSet()->setDefine("OE_LANDCOVER_TEX_MATRIX", landCover.matrixName());

    // Apply a default, empty texture to each render binding.
    OE_DEBUG << LC << "Render Bindings:\n";
    osg::StateSet* terrainSS = _terrain->getOrCreateStateSet();
    osg::ref_ptr<osg::Texture> tex = new osg::Texture2D(ImageUtils::createEmptyImage(1, 1));
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    for (unsigned i = 0; i < _renderBindings.size(); ++i)
    {
        SamplerBinding& b = _renderBindings[i];
        if (b.isActive())
        {
            terrainSS->addUniform(new osg::Uniform(b.samplerName().c_str(), b.unit()));
            terrainSS->setTextureAttribute(b.unit(), tex.get());
            OE_DEBUG << LC << " > Bound \"" << b.samplerName() << "\" to unit " << b.unit() << "\n";
        }
    }
}

void
RexTerrainEngineNode::dirtyTerrain()
{
    if (_terrain.valid())
    {
        _terrain->releaseGLObjects();
        _terrain->removeChildren(0, _terrain->getNumChildren());
    }

    // clear the loader:
    _loader->clear();

    // clear out the tile registry:
    if ( _liveTiles.valid() )
    {
        _liveTiles->releaseAll(_releaser.get());
    }

    // scrub the geometry pool:
    _geometryPool->clear();

    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    std::vector<TileKey> keys;
    getMap()->getProfile()->getAllKeysAtLOD(options().firstLOD().get(), keys );

    // create a root node for each root tile key.
    OE_INFO << LC << "Creating " << keys.size() << " root keys." << std::endl;

    // We need to take a self-ref here to ensure that the TileNode's data loader
    // can use its observer_ptr back to the terrain engine.
    this->ref();

    for( unsigned i=0; i<keys.size(); ++i )
    {
        TileNode* tileNode = new TileNode();

        // Next, build the surface geometry for the node.
        tileNode->create( keys[i], 0L, _engineContext.get() );

        // Add it to the scene graph
        _terrain->addChild( tileNode );

        // And load the tile's data synchronously (only for root tiles)
        tileNode->loadSync();

        OE_DEBUG << " - " << (i+1) << "/" << keys.size() << " : " << keys[i].str() << std::endl;
    }

    // release the self-ref.
    this->unref_nodelete();

    // Set up the state sets.
    updateState();

    // Call the base class
    TerrainEngineNode::dirtyTerrain();
}

namespace
{
    // debugging
    struct CheckForOrphans : public TileNodeRegistry::ConstOperation {
        void operator()( const TileNodeRegistry::TileNodeMap& tiles ) const {
            unsigned count = 0;
            for(TileNodeRegistry::TileNodeMap::const_iterator i = tiles.begin(); i != tiles.end(); ++i ) {
                if ( i->second.tile->referenceCount() == 1 ) {
                    count++;
                }
            }
            if ( count > 0 )
                OE_WARN << LC << "Oh no! " << count << " orphaned tiles in the reg" << std::endl;
        }
    };
}


void
RexTerrainEngineNode::dirtyState()
{
    // TODO: perhaps defer this until the next update traversal so we don't
    // reinitialize the state multiple times unnecessarily.
    updateState();
}

void
RexTerrainEngineNode::cacheAllLayerExtentsInMapSRS()
{
    // Only call during update
    LayerVector layers;
    getMap()->getLayers(layers);
    for(LayerVector::const_iterator i = layers.begin();
        i != layers.end();
        ++i)
    {
        cacheLayerExtentInMapSRS(i->get());
    }
}

void
RexTerrainEngineNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        if (_renderModelUpdateRequired)
        {
            UpdateRenderModels visitor(getMap(), _renderBindings);
            _terrain->accept(visitor);
            _renderModelUpdateRequired = false;
        }

        // Called once on the first update pass to ensure that all existing
        // layers have their extents cached properly
        if (_cachedLayerExtentsComputeRequired)
        {
            cacheAllLayerExtentsInMapSRS();
            _cachedLayerExtentsComputeRequired = false;
            ADJUST_UPDATE_TRAV_COUNT(this, -1);
        }

        TerrainEngineNode::traverse( nv );
    }

    else if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // Inform the registry of the current frame so that Tiles have access
        // to the information.
        if ( _liveTiles.valid() && nv.getFrameStamp() )
        {
            _liveTiles->setTraversalFrame( nv.getFrameStamp()->getFrameNumber() );
        }

        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);

        // Marks the start of the cull pass
        getEngineContext()->startCull( cv );

        // Initialize a new culler
        TerrainCuller culler(cv, this->getEngineContext());

        // Prepare the culler with the set of renderable layers:
        culler.setup(getMap(), _cachedLayerExtents, this->getEngineContext()->getRenderBindings());

#ifdef PROFILE
        static std::vector<double> times;
        static double times_total = 0.0;
        osg::Timer_t s1 = osg::Timer::instance()->tick();
#endif

        // Assemble the terrain drawables:
        _terrain->accept(culler);

        // If we're using geometry pooling, optimize the drawable for shared state
        // by sorting the draw commands.
        // TODO: benchmark this further to see whether it's worthwhile
        unsigned totalTiles = 0L;
        if (getEngineContext()->getGeometryPool()->isEnabled())
        {
            totalTiles = culler._terrain.sortDrawCommands();
        }

#ifdef PROFILE
        osg::Timer_t s2 = osg::Timer::instance()->tick();
        double delta = osg::Timer::instance()->delta_m(s1, s2);
        times_total += delta;
        times.push_back(delta);
        if (times.size() == 60)
        {
            Registry::instance()->startActivity("CULL(ms)", Stringify()<<(times_total/times.size()));
            Registry::instance()->startActivity("Tiles:", Stringify()<<totalTiles);
            times.clear();
            times_total = 0;
        }
#endif

        // The common stateset for the terrain group:
        cv->pushStateSet(_terrain->getOrCreateStateSet());

        // Push all the layers to draw on to the cull visitor in the order in which
        // they appear in the map.
        LayerDrawable* lastLayer = 0L;
        unsigned order = 0;
        bool surfaceStateSetPushed = false;
        bool imageLayerStateSetPushed = false;
        int layersDrawn = 0;

        osg::State::StateSetStack stateSetStack;

        for(LayerDrawableList::iterator i = culler._terrain.layers().begin();
            i != culler._terrain.layers().end();
            ++i)
        {
            // Note: Cannot save lastLayer here because its _tiles may be empty, which can lead to a crash later
            if (!i->get()->_tiles.empty())
            {
                lastLayer = i->get();

                // if this is a RENDERTYPE_TERRAIN_SURFACE, we need to activate either the
                // default surface state set or the image layer state set.
                if (lastLayer->_renderType == Layer::RENDERTYPE_TERRAIN_SURFACE)
                {
                    if (!surfaceStateSetPushed)
                    {
                        cv->pushStateSet(_surfaceStateSet.get());
                        surfaceStateSetPushed = true;
                    }

                    if (lastLayer->_imageLayer || lastLayer->_layer == NULL)
                    {
                        if (!imageLayerStateSetPushed)
                        {
                            cv->pushStateSet(_imageLayerStateSet.get());
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

                //OE_INFO << "   Apply: " << (lastLayer->_layer ? lastLayer->_layer->getName() : "-1") << "; tiles=" << lastLayer->_tiles.size() << std::endl;
                //buf << (lastLayer->_layer ? lastLayer->_layer->getName() : "none") << " (" << lastLayer->_tiles.size() << ")\n";

                if (lastLayer->_layer)
                {
                    lastLayer->_layer->apply(lastLayer, cv);                    
                }
                else
                {
                    lastLayer->accept(*cv);
                }

                ++layersDrawn;
            }

            //buf << (lastLayer->_layer ? lastLayer->_layer->getName() : "none") << " (" << lastLayer->_tiles.size() << ")\n";
        }

        // Uncomment this to see how many layers were drawn
        //Registry::instance()->startActivity("Layers", Stringify()<<layersDrawn);

        // The last layer to render must clear up the OSG state,
        // otherwise it will be corrupt and can lead to crashing.
        if (lastLayer)
        {
            lastLayer->_clearOsgState = true;
        }

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

        // pop the common terrain state set
        cv->popStateSet();

        // marks the end of the cull pass
        this->getEngineContext()->endCull( cv );

        // If the culler found any orphaned data, we need to update the render model
        // during the next update cycle.
        if (culler._orphanedPassesDetected > 0u)
        {
            _renderModelUpdateRequired = true;
            OE_INFO << LC << "Detected " << culler._orphanedPassesDetected << " orphaned rendering passes\n";
        }

        // we don't call this b/c we don't want _terrain
        //TerrainEngineNode::traverse(nv);

        // traverse all the other children (geometry pool, loader/unloader, etc.)
        _geometryPool->accept(nv);
        _loader->accept(nv);
        _unloader->accept(nv);
        _releaser->accept(nv);

        if (_rasterizer)
            _rasterizer->accept(nv);
    }

    else
    {
        TerrainEngineNode::traverse( nv );
    }
}

unsigned int
RexTerrainEngineNode::computeSampleSize(unsigned int levelOfDetail)
{
    unsigned maxLevel = osg::minimum(options().maxLOD().get(), 19u ); // beyond LOD 19 or 20, morphing starts to lose precision.
    unsigned int meshSize = options().tileSize().get();

    unsigned int sampleSize = meshSize;
    int level = maxLevel; // make sure it's signed for the loop below to work

    while( level >= 0 && levelOfDetail != level)
    {
        sampleSize = sampleSize * 2 - 1;
        level--;
    }

    return sampleSize;
}

osg::Vec3d getWorld( const GeoHeightField& geoHF, unsigned int c, unsigned int r)
{
    double x = geoHF.getExtent().xMin() + (double)c * geoHF.getXInterval();
    double y = geoHF.getExtent().yMin() + (double)r * geoHF.getYInterval();
    double h = geoHF.getHeightField()->getHeight(c,r);

    osg::Vec3d world;
    GeoPoint point(geoHF.getExtent().getSRS(), x, y, h );
    point.toWorld( world );
    return world;
}

osg::Node* renderHeightField(const GeoHeightField& geoHF)
{
    osg::MatrixTransform* mt = new osg::MatrixTransform;

    GeoPoint centroid;
    geoHF.getExtent().getCentroid(centroid);

    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal( world2local );
    local2world.invert( world2local );

    mt->setMatrix( local2world );

    osg::Geometry* geometry = new osg::Geometry;
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geometry );
    mt->addChild( geode );

    osg::Vec3Array* verts = new osg::Vec3Array;
    geometry->setVertexArray( verts );

    for (unsigned int c = 0; c < geoHF.getHeightField()->getNumColumns() - 1; c++)
    {
        for (unsigned int r = 0; r < geoHF.getHeightField()->getNumRows() - 1; r++)
        {
            // Add two triangles
            verts->push_back( getWorld( geoHF, c,     r    ) * world2local );
            verts->push_back( getWorld( geoHF, c + 1, r    ) * world2local );
            verts->push_back( getWorld( geoHF, c + 1, r + 1) * world2local );

            verts->push_back( getWorld( geoHF, c,     r    ) * world2local );
            verts->push_back( getWorld( geoHF, c + 1, r + 1) * world2local );
            verts->push_back( getWorld( geoHF, c,     r + 1) * world2local );
        }
    }
    geode->setCullingActive(false);
    mt->setCullingActive(false);

    geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, verts->size()));

    osg::Vec4ubArray* colors = new osg::Vec4ubArray();
    colors->push_back(osg::Vec4ub(255,0,0,255));
    geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
    mt->getOrCreateStateSet()->setRenderBinDetails(99, "RenderBin");

    return mt;
}

void
RexTerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
    if ( change.getAction() == MapModelChange::BEGIN_BATCH_UPDATE )
    {
        _batchUpdateInProgress = true;
    }

    else if ( change.getAction() == MapModelChange::END_BATCH_UPDATE )
    {
        _batchUpdateInProgress = false;

        if ( _refreshRequired )
            refresh();

        if ( _stateUpdateRequired )
            updateState();
    }

    else
    {
        _liveTiles->setMapRevision(getMap()->getDataModelRevision());

        // dispatch the change handler
        if ( change.getLayer() )
        {
            // then apply the actual change:
            switch( change.getAction() )
            {
            case MapModelChange::ADD_LAYER:
            case MapModelChange::ENABLE_LAYER:
                addLayer(change.getLayer());
                break;

            case MapModelChange::REMOVE_LAYER:
            case MapModelChange::DISABLE_LAYER:
                if (change.getImageLayer())
                    removeImageLayer( change.getImageLayer() );
                else if (change.getElevationLayer())
                    removeElevationLayer(change.getElevationLayer());
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
RexTerrainEngineNode::cacheLayerExtentInMapSRS(Layer* layer)
{
    if (layer->getUID() + 1 > _cachedLayerExtents.size())
    {
        _cachedLayerExtents.resize(layer->getUID()+1);
    }

    // Store the layer's extent in the map's SRS:
    LayerExtent& le = _cachedLayerExtents[layer->getUID()];

    if (!le._computed && getMap()->getProfile())
    {
        le._extent = getMap()->getProfile()->clampAndTransformExtent(layer->getExtent());
        le._computed = true;
    }
}

void
RexTerrainEngineNode::addLayer(Layer* layer)
{
    if (layer)
    {
        if (layer->getEnabled())
        {
            if (layer->getRenderType() == Layer::RENDERTYPE_TERRAIN_SURFACE)
                addTileLayer(layer);
            else if (dynamic_cast<ElevationLayer*>(layer))
                addElevationLayer(dynamic_cast<ElevationLayer*>(layer));
        }

        cacheLayerExtentInMapSRS(layer);        
    }
}

void
RexTerrainEngineNode::addTileLayer(Layer* tileLayer)
{
    if ( tileLayer && tileLayer->getEnabled() )
    {
        ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(tileLayer);
        if (imageLayer)
        {
            // for a shared layer, allocate a shared image unit if necessary.
            if ( imageLayer->isShared() )
            {
                if (!imageLayer->sharedImageUnit().isSet())
                {
                    int temp;
                    if ( getResources()->reserveTextureImageUnit(temp, imageLayer->getName().c_str()) )
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
                if ( imageLayer->sharedImageUnit().isSet() )
                {
                    // Find the next empty SHARED slot:
                    unsigned newIndex = SamplerBinding::SHARED;
                    while (_renderBindings[newIndex].isActive())
                        ++newIndex;

                    // Put the new binding there:
                    SamplerBinding& newBinding = _renderBindings[newIndex];
                    newBinding.usage()       = SamplerBinding::SHARED;
                    newBinding.sourceUID()   = imageLayer->getUID();
                    newBinding.unit()        = imageLayer->sharedImageUnit().get();
                    newBinding.samplerName() = imageLayer->getSharedTextureUniformName();
                    newBinding.matrixName()  = imageLayer->getSharedTextureMatrixUniformName();

                    OE_INFO << LC
                        << "Shared Layer \"" << imageLayer->getName() << "\" : sampler=\"" << newBinding.samplerName() << "\", "
                        << "matrix=\"" << newBinding.matrixName() << "\", "
                        << "unit=" << newBinding.unit() << "\n";

                    // Install an empty texture for this binding at the top of the graph, so that
                    // a texture is always defined even when the data source supplies no real data.
                    if (newBinding.isActive())
                    {
                        osg::StateSet* terrainSS = _terrain->getOrCreateStateSet();
                        osg::ref_ptr<osg::Texture> tex = new osg::Texture2D(ImageUtils::createEmptyImage(1,1));
                        tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
                        terrainSS->addUniform(new osg::Uniform(newBinding.samplerName().c_str(), newBinding.unit()));
                        terrainSS->setTextureAttribute(newBinding.unit(), tex.get(), 1);
                        OE_INFO << LC << "Bound shared sampler " << newBinding.samplerName() << " to unit " << newBinding.unit() << std::endl;
                    }
                }
            }
        }

        else
        {
            // non-image tile layer. Keep track of these..
        }

        if (_terrain)
        {
            // Update the existing render models, and trigger a data reload.
            // Later we can limit the reload to an update of only the new data.
            UpdateRenderModels updateModels(getMap(), _renderBindings);
            updateModels.setReloadData(true);
            _terrain->accept(updateModels);
        }

        updateState();
    }
}


void
RexTerrainEngineNode::removeImageLayer( ImageLayer* layerRemoved )
{
    if ( layerRemoved )
    {
        // for a shared layer, release the shared image unit.
        if ( layerRemoved->getEnabled() && layerRemoved->isShared() )
        {
            if ( layerRemoved->sharedImageUnit().isSet() )
            {
                getResources()->releaseTextureImageUnit( *layerRemoved->sharedImageUnit() );
                layerRemoved->sharedImageUnit().unset();
            }

            // Remove from RenderBindings (mark as unused)
            for (unsigned i = 0; i < _renderBindings.size(); ++i)
            {
                SamplerBinding& binding = _renderBindings[i];
                if (binding.isActive() && binding.sourceUID() == layerRemoved->getUID())
                {
                    OE_INFO << LC << "Binding (" << binding.samplerName() << " unit " << binding.unit() << ") cleared\n";
                    binding.usage().clear();
                    binding.unit() = -1;

                    // Request an update to reset the shared sampler in the scene graph
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
        UpdateRenderModels updater(getMap(), _renderBindings);
        _terrain->accept(updater);
    }

    //OE_INFO << LC << " Updated " << updater._count << " tiles\n";
}

void
RexTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    if (layer && layer->getEnabled() && layer->getVisible())
    {
        refresh();
    }
}

void
RexTerrainEngineNode::removeElevationLayer( ElevationLayer* layer)
{
    // only need to refresh is the elevation layer is visible.
    if (layer)
    {
        refresh();
    }
}

void
RexTerrainEngineNode::moveElevationLayer(ElevationLayer* layer)
{
    if (layer && layer->getEnabled() && layer->getVisible())
    {
        refresh();
    }
}

// Generates the main shader code for rendering the terrain.
void
RexTerrainEngineNode::updateState()
{
    if (_batchUpdateInProgress)
    {
        _stateUpdateRequired = true;
    }
    else
    {
        osg::StateSet* terrainStateSet = _terrain->getOrCreateStateSet();   // everything
        terrainStateSet->setName("Terrain Group");

        osg::StateSet* surfaceStateSet = getSurfaceStateSet();    // just the surface

        // required for multipass tile rendering to work
        surfaceStateSet->setAttributeAndModes(
            new osg::Depth(osg::Depth::LEQUAL, 0, 1, true));

        surfaceStateSet->setAttributeAndModes(
            new osg::CullFace(), osg::StateAttribute::ON);

        // activate standard mix blending.
        terrainStateSet->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON);

        // install patch param if we are tessellation on the GPU.
        if (options().gpuTessellation() == true)
        {
#ifdef HAVE_PATCH_PARAMETER
            terrainStateSet->setAttributeAndModes(new osg::PatchParameter(3));
#endif
        }

        Shaders package;

        VirtualProgram* terrainVP = VirtualProgram::getOrCreate(terrainStateSet);
        terrainVP->setName("Rex Terrain");
        package.load(terrainVP, package.ENGINE_VERT_MODEL);

        surfaceStateSet->addUniform(new osg::Uniform("oe_terrain_color", options().color().get()));

        surfaceStateSet->addUniform(new osg::Uniform("oe_terrain_altitude", (float)0.0f));

        surfaceStateSet->setDefine("OE_TERRAIN_RENDER_IMAGERY");

        // Functions that affect only the terrain surface:
        VirtualProgram* surfaceVP = VirtualProgram::getOrCreate(surfaceStateSet);
        surfaceVP->setName("Rex Surface");

        // Functions that affect the terrain surface only:
        package.load(surfaceVP, package.ENGINE_VERT_VIEW);
        package.load(surfaceVP, package.ENGINE_ELEVATION_MODEL);
        //package.load(surfaceVP, package.ENGINE_FRAG);

        // Elevation?
        if (this->elevationTexturesRequired())
        {
            surfaceStateSet->setDefine("OE_TERRAIN_RENDER_ELEVATION");
        }

        // Normal mapping shaders:
        //if (this->normalTexturesRequired())
        {
            package.load(surfaceVP, package.NORMAL_MAP_VERT);
            package.load(surfaceVP, package.NORMAL_MAP_FRAG);

            if (this->normalTexturesRequired())
                surfaceStateSet->setDefine("OE_TERRAIN_RENDER_NORMAL_MAP");
        }

        if (options().enableBlending() == true)
        {
            surfaceStateSet->setDefine("OE_TERRAIN_BLEND_IMAGERY");
        } 

        if (options().compressNormalMaps() == true)
        {
            surfaceStateSet->setDefine("OE_COMPRESSED_NORMAL_MAP");
        }

        // Morphing?
        if (_morphingSupported)
        {
            if ((options().morphTerrain() == true && _morphTerrainSupported == true) ||
                options().morphImagery() == true)
            {
                package.load(surfaceVP, package.MORPHING_VERT);

                if ((options().morphTerrain() == true && _morphTerrainSupported == true))
                {
                    surfaceStateSet->setDefine("OE_TERRAIN_MORPH_GEOMETRY");
                }
                if (options().morphImagery() == true)
                {
                    surfaceStateSet->setDefine("OE_TERRAIN_MORPH_IMAGERY");
                }
            }
        }

        // Shadowing?
        if (options().castShadows() == true)
        {
            surfaceStateSet->setDefine("OE_TERRAIN_CAST_SHADOWS");
        }

        // assemble color filter code snippets.
        bool haveColorFilters = false;
        {
            // Color filter frag function:
            std::string fs_colorfilters =
                "#version " GLSL_VERSION_STR "\n"
                GLSL_DEFAULT_PRECISION_FLOAT "\n"
                "uniform int oe_layer_uid; \n"
                "$COLOR_FILTER_HEAD"
                "void oe_rexEngine_applyFilters(inout vec4 color) \n"
                "{ \n"
                    "$COLOR_FILTER_BODY"
                "} \n";

            std::stringstream cf_head;
            std::stringstream cf_body;
            const char* I = "    ";

            bool ifStarted = false;
            ImageLayerVector imageLayers;
            getMap()->getLayers(imageLayers);

            for( int i=0; i<imageLayers.size(); ++i )
            {
                ImageLayer* layer = imageLayers[i].get();
                if ( layer->getEnabled() )
                {
                    // install Color Filter function calls:
                    const ColorFilterChain& chain = layer->getColorFilters();
                    if ( chain.size() > 0 )
                    {
                        haveColorFilters = true;
                        if ( ifStarted ) cf_body << I << "else if ";
                        else             cf_body << I << "if ";
                        cf_body << "(oe_layer_uid == " << layer->getUID() << ") {\n";
                        for( ColorFilterChain::const_iterator j = chain.begin(); j != chain.end(); ++j )
                        {
                            const ColorFilter* filter = j->get();
                            cf_head << "void " << filter->getEntryPointFunctionName() << "(inout vec4 color);\n";
                            cf_body << I << I << filter->getEntryPointFunctionName() << "(color);\n";
                            filter->install( surfaceStateSet );
                        }
                        cf_body << I << "}\n";
                        ifStarted = true;
                    }
                }
            }

            if ( haveColorFilters )
            {
                std::string cf_head_str, cf_body_str;
                cf_head_str = cf_head.str();
                cf_body_str = cf_body.str();

                replaceIn( fs_colorfilters, "$COLOR_FILTER_HEAD", cf_head_str );
                replaceIn( fs_colorfilters, "$COLOR_FILTER_BODY", cf_body_str );

                surfaceVP->setFunction(
                    "oe_rexEngine_applyFilters",
                    fs_colorfilters,
                    ShaderComp::LOCATION_FRAGMENT_COLORING,
                    0.6 );
            }
        }

#if 0
        // Apply uniforms for sampler bindings:
        OE_DEBUG << LC << "Render Bindings:\n";
        osg::ref_ptr<osg::Texture> tex = new osg::Texture2D(ImageUtils::createEmptyImage(1,1));
        for (unsigned i = 0; i < _renderBindings.size(); ++i)
        {
            SamplerBinding& b = _renderBindings[i];
            if (b.isActive())
            {
                osg::Uniform* u = new osg::Uniform(b.samplerName().c_str(), b.unit());
                terrainStateSet->addUniform( u );
                OE_DEBUG << LC << " > Bound \"" << b.samplerName() << "\" to unit " << b.unit() << "\n";
                terrainStateSet->setTextureAttribute(b.unit(), tex.get());
            }
        }
#endif

        // uniform that conveys the layer UID to the shaders; necessary
        // for per-layer branching (like color filters)
        // UID -1 => no image layer (no texture)
        terrainStateSet->addUniform( new osg::Uniform("oe_layer_uid", (int)-1 ) );

        // uniform that conveys the render order, since the shaders
        // need to know which is the first layer in order to blend properly
        terrainStateSet->addUniform( new osg::Uniform("oe_layer_order", (int)0) );

        // uniform that conveys the tile vertex dimensions
        terrainStateSet->addUniform(new osg::Uniform("oe_tile_size", (float)options().tileSize().get()));

        // special object ID that denotes the terrain surface.
        surfaceStateSet->addUniform( new osg::Uniform(
            Registry::objectIndex()->getObjectIDUniformName().c_str(), OSGEARTH_OBJECTID_TERRAIN) );
        
        // For an image layer, attach the default fragment shader:
        //_imageLayerStateSet = osg::clone(surfaceStateSet, osg::CopyOp::DEEP_COPY_ALL);
        _imageLayerStateSet = new osg::StateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(_imageLayerStateSet.get());
        package.load(vp, package.ENGINE_FRAG);

        _stateUpdateRequired = false;
    }
}


osg::Node*
RexTerrainEngineNode::createStandaloneTile(
    const TerrainTileModel* model,
    int createTileFlags,
    unsigned referenceLOD,
    const TileKey& subRegion)
{
    CreateTileImplementation impl;
    return impl.createTile(getEngineContext(), model, createTileFlags, referenceLOD, subRegion);
}
