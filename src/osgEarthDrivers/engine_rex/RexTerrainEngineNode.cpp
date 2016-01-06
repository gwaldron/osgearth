/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include "QuickReleaseGLObjects"
#include "SelectionInfo"

#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderFactory>
#include <osgEarth/MapModelChange>
#include <osgEarth/Progress>
#include <osgEarth/ShaderLoader>
#include <osgEarth/Utils>
#include <osgEarth/ObjectIndex>
#include <osgEarth/TraversalData>

#include <osg/Version>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osgUtil/RenderBin>

#if OSG_VERSION_GREATER_OR_EQUAL(3,1,8)
#   define HAVE_OSG_PATCH_PARAMETER
#   include <osg/PatchParameter>
#endif

#include <cstdlib> // for getenv

#define LC "[RexTerrainEngineNode] "

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;


// TODO: bins don't work with SSDK. No idea why.
#define USE_RENDER_BINS 1

//------------------------------------------------------------------------

namespace
{
    // adapter that lets RexTerrainEngineNode listen to Map events
    struct RexTerrainEngineNodeMapCallbackProxy : public MapCallback
    {
        RexTerrainEngineNodeMapCallbackProxy(RexTerrainEngineNode* node) : _node(node) { }
        osg::observer_ptr<RexTerrainEngineNode> _node;

        void onMapInfoEstablished( const MapInfo& mapInfo ) {
            osg::ref_ptr<RexTerrainEngineNode> node;
            if ( _node.lock(node) )
                node->onMapInfoEstablished( mapInfo );
        }

        void onMapModelChanged( const MapModelChange& change ) {
            osg::ref_ptr<RexTerrainEngineNode> node;
            if ( _node.lock(node) )
                node->onMapModelChanged( change );
        }
    };

#if 0
    // Render bin for terrain surface geometry
    class SurfaceBin : public osgUtil::RenderBin
    {
    public:
        SurfaceBin()
        {
            this->setName( "oe.SurfaceBin" );
            this->setStateSet( new osg::StateSet() );
            this->setSortMode(SORT_FRONT_TO_BACK);
        }

        osg::Object* clone(const osg::CopyOp& copyop) const
        {
            return new SurfaceBin(*this, copyop);
        }

        SurfaceBin(const SurfaceBin& rhs, const osg::CopyOp& copy) :
            osgUtil::RenderBin(rhs, copy)
        {
        }
    };
#endif
}

//---------------------------------------------------------------------------

static Threading::ReadWriteMutex s_engineNodeCacheMutex;
//Caches the MapNodes that have been created
typedef std::map<UID, osg::observer_ptr<RexTerrainEngineNode> > EngineNodeCache;

static
EngineNodeCache& getEngineNodeCache()
{
    static EngineNodeCache s_cache;
    return s_cache;
}

void
RexTerrainEngineNode::registerEngine(RexTerrainEngineNode* engineNode)
{
    Threading::ScopedWriteLock exclusiveLock( s_engineNodeCacheMutex );
    getEngineNodeCache()[engineNode->_uid] = engineNode;
    OE_DEBUG << LC << "Registered engine " << engineNode->_uid << std::endl;
}

void
RexTerrainEngineNode::unregisterEngine( UID uid )
{
    Threading::ScopedWriteLock exclusiveLock( s_engineNodeCacheMutex );
    EngineNodeCache::iterator k = getEngineNodeCache().find( uid );
    if (k != getEngineNodeCache().end())
    {
        getEngineNodeCache().erase(k);
        OE_DEBUG << LC << "Unregistered engine " << uid << std::endl;
    }
}

// since this method is called in a database pager thread, we use a ref_ptr output
// parameter to avoid the engine node being destructed between the time we 
// return it and the time it's accessed; this could happen if the user removed the
// MapNode from the scene during paging.
void
RexTerrainEngineNode::getEngineByUID( UID uid, osg::ref_ptr<RexTerrainEngineNode>& output )
{
    Threading::ScopedReadLock sharedLock( s_engineNodeCacheMutex );
    EngineNodeCache::const_iterator k = getEngineNodeCache().find( uid );
    if (k != getEngineNodeCache().end())
        output = k->second.get();
}

UID
RexTerrainEngineNode::getUID() const
{
    return _uid;
}

//------------------------------------------------------------------------

RexTerrainEngineNode::ElevationChangedCallback::ElevationChangedCallback( RexTerrainEngineNode* terrain ):
_terrain( terrain )
{
    //nop
}

void
RexTerrainEngineNode::ElevationChangedCallback::onVisibleChanged( TerrainLayer* layer )
{
    _terrain->refresh(true); // true => force a dirty
}

//------------------------------------------------------------------------

RexTerrainEngineNode::RexTerrainEngineNode() :
TerrainEngineNode     ( ),
_terrain              ( 0L ),
_update_mapf          ( 0L ),
_tileCount            ( 0 ),
_tileCreationTime     ( 0.0 ),
_batchUpdateInProgress( false ),
_refreshRequired      ( false ),
_stateUpdateRequired  ( false ),
_selectionInfo        ( 0L )
{
    // unique ID for this engine:
    _uid = Registry::instance()->createUID();

    // always require elevation.
    _requireElevationTextures = true;

    // install an elevation callback so we can update elevation data
    _elevationCallback = new ElevationChangedCallback( this );

    // static shaders.
    if ( Registry::capabilities().supportsGLSL() )
    {
        osg::StateSet* stateset = getOrCreateStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        Shaders package;
        package.load(vp, package.SDK);
    }

    _surfaceSS = new osg::StateSet();
}

RexTerrainEngineNode::~RexTerrainEngineNode()
{
    unregisterEngine( _uid );

    if ( _update_mapf )
    {
        delete _update_mapf;
    }
    destroySelectionInfo();
}

void
RexTerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::preInitialize( map, options );
    //nop.
}

void
RexTerrainEngineNode::postInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::postInitialize( map, options );

    // Initialize the map frames. We need one for the update thread and one for the
    // cull thread. Someday we can detect whether these are actually the same thread
    // (depends on the viewer's threading mode).
    _update_mapf = new MapFrame( map, Map::ENTIRE_MODEL );

    // merge in the custom options:
    _terrainOptions.merge( options );

    // morphing imagery LODs requires we bind parent textures to their own unit.
    if ( _terrainOptions.morphImagery() == true )
    {
        _requireParentTextures = true;
    }

    // if the envvar for tile expiration is set, overide the options setting
    const char* val = ::getenv("OSGEARTH_EXPIRATION_THRESHOLD");
    if ( val )
    {
        _terrainOptions.expirationThreshold() = as<unsigned>(val, _terrainOptions.expirationThreshold().get());
        OE_INFO << LC << "Expiration threshold set by env var = " << _terrainOptions.expirationThreshold().get() << "\n";
    }

    // if the envvar for hires prioritization is set, override the options setting
    const char* hiresFirst = ::getenv("OSGEARTH_HIGH_RES_FIRST");
    if ( hiresFirst )
    {
        _terrainOptions.highResolutionFirst() = true;
    }

    // check for normal map generation (required for lighting).
    if ( _terrainOptions.normalMaps() == true )
    {
        this->_requireNormalTextures = true;
    }

    // A shared registry for tile nodes in the scene graph. Enable revision tracking
    // if requested in the options. Revision tracking lets the registry notify all
    // live tiles of the current map revision so they can inrementally update
    // themselves if necessary.
    _liveTiles = new TileNodeRegistry("live");
    _liveTiles->setMapRevision( _update_mapf->getRevision() );

    if ( _terrainOptions.quickReleaseGLObjects() == true )
    {
        _deadTiles = new TileNodeRegistry("dead");
        _quickReleaseInstalled = false;
        ADJUST_UPDATE_TRAV_COUNT( this, +1 );
    }

    // A shared geometry pool.
    if ( ::getenv("OSGEARTH_REX_NO_POOL") == 0L )
    {
        _geometryPool = new GeometryPool( _terrainOptions );
    }

    // Make a tile loader
    PagerLoader* loader = new PagerLoader( this );
    loader->setMergesPerFrame( _terrainOptions.mergesPerFrame().get() );
    _loader = loader;
    this->addChild( _loader.get() );

    // Make a tile unloader
    _unloader = new UnloaderGroup( _liveTiles.get(), _deadTiles.get() );
    _unloader->setThreshold( _terrainOptions.expirationThreshold().get() );
    this->addChild( _unloader.get() );
    
    // handle an already-established map profile:
    MapInfo mapInfo( map );
    if ( _update_mapf->getProfile() )
    {
        // NOTE: this will initialize the map with the startup layers
        onMapInfoEstablished( mapInfo );
    }

    // install a layer callback for processing further map actions:
    map->addMapCallback( new RexTerrainEngineNodeMapCallbackProxy(this) );

    // Prime with existing layers:
    _batchUpdateInProgress = true;

    ElevationLayerVector elevationLayers;
    map->getElevationLayers( elevationLayers );
    for( ElevationLayerVector::const_iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i )
        addElevationLayer( i->get() );

    ImageLayerVector imageLayers;
    map->getImageLayers( imageLayers );
    for( ImageLayerVector::iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
        addImageLayer( i->get() );

    _batchUpdateInProgress = false;

    // set up the initial shaders
    updateState();

    // register this instance to the osgDB plugin can find it.
    registerEngine( this );

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

        if ( !extent.getSRS()->isEquivalentTo(this->getMap()->getSRS()) )
        {
            extent.transform(this->getMap()->getSRS(), extentLocal);
        }
        
        _liveTiles->setDirty(extentLocal, minLevel, maxLevel);
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

void
RexTerrainEngineNode::onMapInfoEstablished( const MapInfo& mapInfo )
{
    dirtyTerrain();
}

osg::StateSet*
RexTerrainEngineNode::getSurfaceStateSet()
{
    return _surfaceSS.get();
}

void
RexTerrainEngineNode::setupRenderBindings()
{
    _renderBindings.push_back( SamplerBinding() );
    SamplerBinding& color = _renderBindings.back();
    color.usage()       = SamplerBinding::COLOR;
    color.samplerName() = "oe_layer_tex";
    color.matrixName()  = "oe_layer_texMatrix";
    this->getResources()->reserveTextureImageUnit( color.unit(), "Terrain Color" );

    _renderBindings.push_back( SamplerBinding() );
    SamplerBinding& elevation = _renderBindings.back();
    elevation.usage()       = SamplerBinding::ELEVATION;
    elevation.samplerName() = "oe_tile_elevationTex";
    elevation.matrixName()  = "oe_tile_elevationTexMatrix";
    this->getResources()->reserveTextureImageUnit( elevation.unit(), "Terrain Elevation" );

    _renderBindings.push_back( SamplerBinding() );
    SamplerBinding& normal = _renderBindings.back();
    normal.usage()       = SamplerBinding::NORMAL;
    normal.samplerName() = "oe_tile_normalTex";
    normal.matrixName()  = "oe_tile_normalTexMatrix";
    this->getResources()->reserveTextureImageUnit( normal.unit(), "Terrain Normals" );

    _renderBindings.push_back( SamplerBinding() );
    SamplerBinding& colorParent = _renderBindings.back();
    colorParent.usage()       = SamplerBinding::COLOR_PARENT;
    colorParent.samplerName() = "oe_layer_texParent";
    colorParent.matrixName()  = "oe_layer_texParentMatrix";
    this->getResources()->reserveTextureImageUnit( colorParent.unit(), "Terrain Color (Parent)" );
}

void RexTerrainEngineNode::destroySelectionInfo()
{
    if (_selectionInfo)
    {
        delete _selectionInfo; _selectionInfo = 0;
    }
}

void RexTerrainEngineNode::buildSelectionInfo()
{
    _selectionInfo = new SelectionInfo;
}

void
RexTerrainEngineNode::dirtyTerrain()
{
    //TODO: scrub the geometry pool?

    // clear the loader:
    _loader->clear();

    if ( _terrain )
    {
        this->removeChild( _terrain );
    }

    // New terrain
    _terrain = new osg::Group();
    this->addChild( _terrain );

    // are we LOD blending?
    bool setupParentData = 
        _terrainOptions.morphImagery() == true || // gw: redundant?
        this->parentTexturesRequired();
    
    // reserve GPU unit for the main color texture:
    if ( _renderBindings.empty() )
    {
        setupRenderBindings();
    }

    // recalculate the LOD morphing parameters:
    destroySelectionInfo();
    buildSelectionInfo();

    // clear out the tile registry:
    if ( _liveTiles.valid() )
    {
        _liveTiles->moveAll( _deadTiles.get() );
    }

    // Factory to create the root keys:
    EngineContext* context = getEngineContext();

    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    std::vector<TileKey> keys;
    _update_mapf->getProfile()->getAllKeysAtLOD( *_terrainOptions.firstLOD(), keys );

    // create a root node for each root tile key.
    OE_INFO << LC << "Creating " << keys.size() << " root keys.." << std::endl;

    unsigned child = 0;
    for( unsigned i=0; i<keys.size(); ++i )
    {
        TileNode* tileNode = new TileNode();
                
        // Next, build the surface geometry for the node.
        tileNode->create( keys[i], context );

        _terrain->addChild( tileNode );
    }

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
                if ( i->second->referenceCount() == 1 ) {
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
RexTerrainEngineNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR && _quickReleaseInstalled == false )
    {
        osg::Camera* cam = findFirstParentOfType<osg::Camera>( this );
        if ( cam )
        {
            // get the installed PDC so we can nest them:
            osg::Camera::DrawCallback* cbToNest = cam->getPostDrawCallback();

            // if it's another QR callback, we'll just replace it.
            QuickReleaseGLObjects* previousQR = dynamic_cast<QuickReleaseGLObjects*>(cbToNest);
            if ( previousQR )
                cbToNest = previousQR->_next.get();

            cam->setPostDrawCallback( new QuickReleaseGLObjects(_deadTiles.get(), cbToNest) );

            _quickReleaseInstalled = true;
            OE_INFO << LC << "Quick release enabled" << std::endl;

            // knock down the trav count set in the constructor.
            ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        }
    }

    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // Inform the registry of the current frame so that Tiles have access
        // to the information.
        if ( _liveTiles.valid() && nv.getFrameStamp() )
        {
            _liveTiles->setTraversalFrame( nv.getFrameStamp()->getFrameNumber() );
        }
    }

#if 0
    static int c = 0;
    if ( ++c % 60 == 0 )
    {
        OE_NOTICE << LC << "Live = " << _liveTiles->size() << ", Dead = " << _deadTiles->size() << std::endl;
        _liveTiles->run( CheckForOrphans() );
    }
#endif
    
    if ( nv.getVisitorType() == nv.CULL_VISITOR && _loader.valid() ) // ensures that postInitialize has run
    {
        // Pass the tile creation context to the traversal.
        osg::ref_ptr<osg::Referenced> data = nv.getUserData();
        nv.setUserData( this->getEngineContext() );

        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);

        this->getEngineContext()->_surfaceSS = _surfaceSS.get();

        this->getEngineContext()->startCull( cv );
        TerrainEngineNode::traverse( nv );
        this->getEngineContext()->endCull( cv );

        if ( data.valid() )
            nv.setUserData( data.get() );
    }

    else
    {
        TerrainEngineNode::traverse( nv );
    }
}


EngineContext*
RexTerrainEngineNode::getEngineContext()
{
    osg::ref_ptr<EngineContext>& context = _perThreadTileGroupFactories.get(); // thread-safe get
    if ( !context.valid() )
    {
        // initialize a key node factory.
        context = new EngineContext(
            getMap(),
            this, // engine
            _geometryPool.get(),
            _loader.get(),
            _unloader.get(),
            _liveTiles.get(),
            _deadTiles.get(),
            _renderBindings,
            _terrainOptions,
            *_selectionInfo,
            _tilePatchCallbacks);
    }

    return context.get();
}


// no longer used.
osg::Node*
RexTerrainEngineNode::createTile( const TileKey& key )
{
    // TODO: implement again.
    OE_WARN << LC << "createTile is not implemented.\n";
    return 0L;
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
        // update the thread-safe map model copy:
        if ( _update_mapf->sync() )
        {
            _liveTiles->setMapRevision( _update_mapf->getRevision() );
        }

        // dispatch the change handler
        if ( change.getLayer() )
        {
            // then apply the actual change:
            switch( change.getAction() )
            {
            case MapModelChange::ADD_IMAGE_LAYER:
                addImageLayer( change.getImageLayer() );
                break;
            case MapModelChange::REMOVE_IMAGE_LAYER:
                removeImageLayer( change.getImageLayer() );
                break;
            case MapModelChange::ADD_ELEVATION_LAYER:
                addElevationLayer( change.getElevationLayer() );
                break;
            case MapModelChange::REMOVE_ELEVATION_LAYER:
                removeElevationLayer( change.getElevationLayer() );
                break;
            case MapModelChange::MOVE_IMAGE_LAYER:
                moveImageLayer( change.getFirstIndex(), change.getSecondIndex() );
                break;
            case MapModelChange::MOVE_ELEVATION_LAYER:
                moveElevationLayer( change.getFirstIndex(), change.getSecondIndex() );
                break;
            case MapModelChange::TOGGLE_ELEVATION_LAYER:
                toggleElevationLayer( change.getElevationLayer() );
                break;
            case MapModelChange::ADD_MODEL_LAYER:
            case MapModelChange::REMOVE_MODEL_LAYER:
            case MapModelChange::MOVE_MODEL_LAYER:
            default: 
                break;
            }
        }
    }
}


void
RexTerrainEngineNode::addImageLayer( ImageLayer* layerAdded )
{
    if ( layerAdded && layerAdded->getEnabled() )
    {
        // for a shared layer, allocate a shared image unit if necessary.
        if ( layerAdded->isShared() )
        {
            optional<int>& unit = layerAdded->shareImageUnit();
            if ( !unit.isSet() )
            {
                int temp;
                if ( getResources()->reserveTextureImageUnit(temp) )
                {
                    layerAdded->shareImageUnit() = temp;
                    OE_INFO << LC << "Image unit " << temp << " assigned to shared layer " << layerAdded->getName() << std::endl;
                }
                else
                {
                    OE_WARN << LC << "Insufficient GPU image units to share layer " << layerAdded->getName() << std::endl;
                }
            }

            // Build a sampler binding for the layer.
            if ( unit.isSet() )
            {
                _renderBindings.push_back( SamplerBinding() );
                SamplerBinding& binding = _renderBindings.back();

                //binding.usage()     = binding.MATERIAL; // ?... not COLOR at least
                binding.sourceUID() = layerAdded->getUID();
                binding.unit()      = unit.get();

                binding.samplerName() = layerAdded->shareTexUniformName().get();
                binding.matrixName()  = layerAdded->shareTexMatUniformName().get();

                OE_INFO << LC 
                    << " .. Sampler=\"" << binding.samplerName() << "\", "
                    << "Matrix=\"" << binding.matrixName() << ", "
                    << "unit=" << binding.unit() << "\n";                
            }
        }
    }

    refresh();
}


void
RexTerrainEngineNode::removeImageLayer( ImageLayer* layerRemoved )
{
    if ( layerRemoved )
    {
        // for a shared layer, release the shared image unit.
        if ( layerRemoved->getEnabled() && layerRemoved->isShared() )
        {
            if ( layerRemoved->shareImageUnit().isSet() )
            {
                getResources()->releaseTextureImageUnit( *layerRemoved->shareImageUnit() );
                layerRemoved->shareImageUnit().unset();
            }

            //TODO: remove the sampler/matrix uniforms
        }
    }

    refresh();
}

void
RexTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
    updateState();
}

void
RexTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    if ( layer == 0L || layer->getEnabled() == false )
        return;

    layer->addCallback( _elevationCallback.get() );

    refresh();
}

void
RexTerrainEngineNode::removeElevationLayer( ElevationLayer* layerRemoved )
{
    if ( layerRemoved->getEnabled() == false )
        return;

    layerRemoved->removeCallback( _elevationCallback.get() );

    refresh();
}

void
RexTerrainEngineNode::moveElevationLayer( unsigned int oldIndex, unsigned int newIndex )
{
    refresh();
}

void
RexTerrainEngineNode::toggleElevationLayer( ElevationLayer* layer )
{
    refresh();
}

// Generates the main shader code for rendering the terrain.
void
RexTerrainEngineNode::updateState()
{
    if ( _batchUpdateInProgress )
    {
        _stateUpdateRequired = true;
    }
    else
    {
        osg::StateSet* terrainStateSet   = _terrain->getOrCreateStateSet();   // everything
        osg::StateSet* surfaceStateSet   = getSurfaceStateSet();    // just the surface
        
        // required for multipass tile rendering to work
        surfaceStateSet->setAttributeAndModes(
            new osg::Depth(osg::Depth::LEQUAL, 0, 1, true) );

        // activate standard mix blending.
        terrainStateSet->setAttributeAndModes( 
            new osg::BlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON );

        // install patch param if we are tessellation on the GPU.
        if ( _terrainOptions.gpuTessellation() == true )
        {
            #ifdef HAVE_PATCH_PARAMETER
              terrainStateSet->setAttributeAndModes( new osg::PatchParameter(3) );
            #endif
        }

        // install shaders, if we're using them.
        if ( Registry::capabilities().supportsGLSL() )
        {
            Shaders package;

            VirtualProgram* terrainVP = VirtualProgram::getOrCreate(terrainStateSet);
            terrainVP->setName( "Rex Terrain" );
            package.load(terrainVP, package.ENGINE_VERT_MODEL);
            
            surfaceStateSet->addUniform(new osg::Uniform("oe_terrain_color", _terrainOptions.color().get()));

            bool useBlending = _terrainOptions.enableBlending().get();
            package.define("OE_REX_GL_BLENDING", useBlending);

            bool morphImagery = _terrainOptions.morphImagery().get();
            package.define("OE_REX_MORPH_IMAGERY", morphImagery);

            // Funtions that affect only the terrain surface:
            VirtualProgram* surfaceVP = VirtualProgram::getOrCreate(surfaceStateSet);
            surfaceVP->setName("Rex Surface");

            // Functions that affect the terrain surface only:
            package.load(surfaceVP, package.ENGINE_VERT_VIEW);
            package.load(surfaceVP, package.ENGINE_FRAG);

            // Normal mapping shaders:
            if ( this->normalTexturesRequired() )
            {
                package.load(surfaceVP, package.NORMAL_MAP_VERT);
                package.load(surfaceVP, package.NORMAL_MAP_FRAG);
            }

            // Morphing?
            if (_terrainOptions.morphTerrain() == true ||
                _terrainOptions.morphImagery() == true)
            {
                package.define("OE_REX_VERTEX_MORPHING", (_terrainOptions.morphTerrain() == true));
                package.load(surfaceVP, package.MORPHING_VERT);
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

                // second, install the per-layer color filter functions AND shared layer bindings.
                bool ifStarted = false;
                int numImageLayers = _update_mapf->imageLayers().size();
                for( int i=0; i<numImageLayers; ++i )
                {
                    ImageLayer* layer = _update_mapf->getImageLayerAt(i);
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
                        0.0 );
                }
            }

            // Apply uniforms for sampler bindings:
            OE_DEBUG << LC << "Render Bindings:\n";
            for(RenderBindings::const_iterator b = _renderBindings.begin(); b != _renderBindings.end(); ++b)
            {
                if ( b->isActive() )
                {
                    terrainStateSet->addUniform( new osg::Uniform(b->samplerName().c_str(), b->unit()) );
                    OE_DEBUG << LC << " > Bound \"" << b->samplerName() << "\" to unit " << b->unit() << "\n";
                }
            }

            // uniform that controls per-layer opacity
            terrainStateSet->addUniform( new osg::Uniform("oe_layer_opacity", 1.0f) );

            // uniform that conveys the layer UID to the shaders; necessary
            // for per-layer branching (like color filters)
            // UID -1 => no image layer (no texture)
            terrainStateSet->addUniform( new osg::Uniform("oe_layer_uid", (int)-1 ) );

            // uniform that conveys the render order, since the shaders
            // need to know which is the first layer in order to blend properly
            terrainStateSet->addUniform( new osg::Uniform("oe_layer_order", (int)0) );

            // default min/max range uniforms. (max < min means ranges are disabled)
            terrainStateSet->addUniform( new osg::Uniform("oe_layer_minRange", 0.0f) );
            terrainStateSet->addUniform( new osg::Uniform("oe_layer_maxRange", -1.0f) );
            
            terrainStateSet->getOrCreateUniform(
                "oe_min_tile_range_factor",
                osg::Uniform::FLOAT)->set( *_terrainOptions.minTileRangeFactor() );

            terrainStateSet->addUniform(new osg::Uniform("oe_tile_size", (float)_terrainOptions.tileSize().get()));

            // special object ID that denotes the terrain surface.
            surfaceStateSet->addUniform( new osg::Uniform(
                Registry::objectIndex()->getObjectIDUniformName().c_str(), OSGEARTH_OBJECTID_TERRAIN) );
        }

        _stateUpdateRequired = false;
    }
}
