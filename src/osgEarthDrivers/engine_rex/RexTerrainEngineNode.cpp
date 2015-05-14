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
#include "TilePagedLOD"
#include "Shaders"

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

#include <osg/Depth>
#include <osg/BlendFunc>
#include <osg/PatchParameter>
#include <osgUtil/RenderBin>

#define LC "[RexTerrainEngineNode] "

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;


// TODO: bins don't work with SSDK. No idea why. Disable until further notice.
//#define USE_RENDER_BINS 1

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


    // Render bin for terrain surface geometry
    class TerrainBin : public osgUtil::RenderBin
    {
    public:
        TerrainBin()
        {
            this->setStateSet( new osg::StateSet() );
            this->setSortMode(SORT_FRONT_TO_BACK);
        }

        osg::Object* clone(const osg::CopyOp& copyop) const
        {
            return new TerrainBin(*this, copyop);
        }

        TerrainBin(const TerrainBin& rhs, const osg::CopyOp& copy) :
            osgUtil::RenderBin(rhs, copy)
        {
        }
    };


    // Render bin for terrain payload geometry
    class PayloadBin : public osgUtil::RenderBin
    {
    public:
        PayloadBin()
        {
            this->setStateSet( new osg::StateSet() );
        }

        osg::Object* clone(const osg::CopyOp& copyop) const
        {
            return new PayloadBin(*this, copyop);
        }

        PayloadBin(const PayloadBin& rhs, const osg::CopyOp& copy) :
            osgUtil::RenderBin(rhs, copy)
        {
        }
    };
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
_stateUpdateRequired  ( false )
{
    // unique ID for this engine:
    _uid = Registry::instance()->createUID();

    // always require elevation.
    _requireElevationTextures = true;

    // Register our render bins protos.
    {
        // Mutex because addRenderBinPrototype isn't thread-safe.
        Threading::ScopedMutexLock lock(_renderBinMutex);

        // generate uniquely named render bin prototypes for this engine:
        _terrainRenderBinPrototype = new TerrainBin();
        _terrainRenderBinPrototype->setName( Stringify() << "oe.TerrainBin." << _uid );
        osgUtil::RenderBin::addRenderBinPrototype( _terrainRenderBinPrototype->getName(), _terrainRenderBinPrototype.get() );

        _payloadRenderBinPrototype = new PayloadBin();
        _payloadRenderBinPrototype->setName( Stringify() << "oe.PayloadBin." << _uid );
        osgUtil::RenderBin::addRenderBinPrototype( _payloadRenderBinPrototype->getName(), _payloadRenderBinPrototype.get() );
    }

    // install an elevation callback so we can update elevation data
    _elevationCallback = new ElevationChangedCallback( this );
}

RexTerrainEngineNode::~RexTerrainEngineNode()
{
    unregisterEngine( _uid );

    osgUtil::RenderBin::removeRenderBinPrototype( _terrainRenderBinPrototype.get() );
    osgUtil::RenderBin::removeRenderBinPrototype( _payloadRenderBinPrototype.get() );

    if ( _update_mapf )
    {
        delete _update_mapf;
    }
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
    _update_mapf = new MapFrame( map, Map::ENTIRE_MODEL, "mp-update" );

    // merge in the custom options:
    _terrainOptions.merge( options );

    if ( _terrainOptions.enableLODBlending() == true )
        _requireParentTextures = true;

    // A shared registry for tile nodes in the scene graph. Enable revision tracking
    // if requested in the options. Revision tracking lets the registry notify all
    // live tiles of the current map revision so they can inrementally update
    // themselves if necessary.
    _liveTiles = new TileNodeRegistry("live");
    _liveTiles->setMapRevision( _update_mapf->getRevision() );

    // A shared geometry pool.
    if ( ::getenv("OSGEARTH_REX_NO_POOL") == 0L )
    {
        const unsigned tileSize = 17;
        _geometryPool = new GeometryPool( tileSize, _terrainOptions );
    }
    
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

    // install some terrain-wide uniforms
    this->getOrCreateStateSet()->getOrCreateUniform(
        "oe_min_tile_range_factor",
        osg::Uniform::FLOAT)->set( *_terrainOptions.minTileRangeFactor() );

    this->getOrCreateStateSet()->getOrCreateUniform(
        "oe_lodblend_delay",
        osg::Uniform::FLOAT)->set( *_terrainOptions.lodBlendDelay() );

    this->getOrCreateStateSet()->getOrCreateUniform(
        "oe_lodblend_duration",
        osg::Uniform::FLOAT)->set( *_terrainOptions.lodBlendDuration() );

    // set up the initial shaders
    updateState();

    // register this instance to the osgDB plugin can find it.
    registerEngine( this );

    // now that we have a map, set up to recompute the bounds
    dirtyBound();

    OE_INFO << LC << "Edge normalization is " << (_terrainOptions.normalizeEdges() == true? "ON" : "OFF") << std::endl;
}


osg::BoundingSphere
RexTerrainEngineNode::computeBound() const
{
    //if ( _terrain && _terrain->getNumChildren() > 0 )
    //{
    //    return _terrain->getBound();
    //}
    //else
    {
        return TerrainEngineNode::computeBound();
    }
}

void
RexTerrainEngineNode::invalidateRegion(const GeoExtent& extent,
                                       unsigned         minLevel,
                                       unsigned         maxLevel)
{
    OE_WARN << LC << "invalidateRegion() is not implemented\n";
    return;

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
RexTerrainEngineNode::getTerrainStateSet()
{
#ifdef USE_RENDER_BINS
    return _terrainRenderBinPrototype->getStateSet();
#else
    return _terrain ? _terrain->getOrCreateStateSet() : 0L;
#endif
}


osg::StateSet*
RexTerrainEngineNode::getPayloadStateSet()
{
    return _payloadRenderBinPrototype->getStateSet();
}

void
RexTerrainEngineNode::dirtyTerrain()
{
    //TODO: scrub the geometry pool?

    //TODO: scrub any heightfield caches in the factory objects?

    // remove existing:
    if ( _terrain )
    {
        this->removeChild( _terrain );
    }

    // New terrain
    _terrain = new osg::Group();

#ifdef USE_RENDER_BINS
    _terrain->getOrCreateStateSet()->setRenderBinDetails( 0, _terrainRenderBinPrototype->getName() );
    _terrain->getOrCreateStateSet()->setNestRenderBins(false);
#else
    _terrain->getOrCreateStateSet()->setRenderBinDetails(0, "SORT_FRONT_TO_BACK");
#endif

    // are we LOD blending?
    bool setupParentData = 
        _terrainOptions.enableLODBlending() == true ||
        this->parentTexturesRequired();

    this->addChild( _terrain );
    
    // reserve GPU unit for the main color texture:
    if ( _renderBindings.color().unit() < 0 )
    {
        _renderBindings.color().samplerName() = "oe_layer_tex";
        _renderBindings.color().matrixName()  = "oe_layer_texMatrix";

        this->getTextureCompositor()->reserveTextureImageUnit(
            _renderBindings.color().unit(), "Rex Engine Color" );
    }

    // reserve GPU unit for the secondary (parent blending) texture:
    if ( _renderBindings.parentColor().unit() < 0 && setupParentData )
    {
        _renderBindings.parentColor().samplerName() = "oe_layer_parentTex";
        _renderBindings.parentColor().matrixName()  = "oe_layer_parentTexMatrix";

        this->getTextureCompositor()->reserveTextureImageUnit(
            _renderBindings.parentColor().unit(), "Rex Engine Parent Color" );
    }

    // reserve a GPU unit for the primary elevation texture:
    if ( _renderBindings.elevation().unit() < 0 )
    {
        _renderBindings.elevation().samplerName() = "oe_tile_elevationTex";
        _renderBindings.elevation().matrixName()  = "oe_tile_elevationTexMatrix";

        this->getResources()->reserveTextureImageUnit(
            _renderBindings.elevation().unit(), "Rex Engine Elevation" );
    }

    // reserve a GPU unit for the secondary (parent blending) elevation texture:
    if ( _renderBindings.parentElevation().unit() < 0 && setupParentData )
    {
        _renderBindings.parentElevation().samplerName() = "oe_tile_parentElevationTex";
        _renderBindings.parentElevation().matrixName()  = "oe_tile_parentElevationTexMatrix";

        this->getResources()->reserveTextureImageUnit(
            _renderBindings.parentElevation().unit(), "Rex Engine Parent Elevation" );
    }

    // reserve a GPU unit for the normal texture:
    if ( this->normalTexturesRequired() )
    {
        if ( _renderBindings.normal().unit() < 0 )
        {
            _renderBindings.normal().samplerName() = "oe_tile_normalTex";
            _renderBindings.normal().matrixName()  = "oe_tile_normalTexMatrix";

            this->getResources()->reserveTextureImageUnit(
                _renderBindings.normal().unit(), "Rex Engine Normals" );
        }

        // reserve a GPU unit for the secondary (parent blending) elevation texture:
        if ( _renderBindings.parentNormal().unit() < 0 && setupParentData )
        {
            _renderBindings.parentNormal().samplerName() = "oe_tile_parentNormalTex";
            _renderBindings.parentNormal().matrixName()  = "oe_tile_parentNormalTexMatrix";

            this->getResources()->reserveTextureImageUnit(
                _renderBindings.parentNormal().unit(), "Rex Engine Parent Normals" );
        }
    }

    // Factory to create the root keys:
    TileGroupFactory* factory = getTileGroupFactory();

    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    std::vector< TileKey > keys;
    _update_mapf->getProfile()->getAllKeysAtLOD( *_terrainOptions.firstLOD(), keys );

    // create a root node for each root tile key.
    OE_INFO << LC << "Creating " << keys.size() << " root keys.." << std::endl;

    TilePagedLOD* root = new TilePagedLOD( _uid, _liveTiles, _deadTiles );
    _terrain->addChild( root );

    osg::ref_ptr<osgDB::Options> dbOptions = Registry::instance()->cloneOrCreateOptions();

    unsigned child = 0;
    for( unsigned i=0; i<keys.size(); ++i )
    {
        osg::ref_ptr<osg::Node> node = factory->createTileGroup( keys[i], true, true, 0L );
        if ( node.valid() )
        {
            root->addChild( node.get() );
            root->setRange( child++, 0.0f, FLT_MAX );
            root->setCenter( node->getBound().center() );
            root->setNumChildrenThatCannotBeExpired( child );
        }
        else
        {
            OE_WARN << LC << "Couldn't make tile for root key: " << keys[i].str() << std::endl;
        }
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
RexTerrainEngineNode::traverse(osg::NodeVisitor& nv)
{
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

    TerrainEngineNode::traverse( nv );
}


TileGroupFactory*
RexTerrainEngineNode::getTileGroupFactory()
{
    osg::ref_ptr<TileGroupFactory>& factory = _perThreadTileGroupFactories.get(); // thread-safe get
    if ( !factory.valid() )
    {
        // create a compiler for compiling tile models into geometry
        // TODO: pass this somehow...?
        bool optimizeTriangleOrientation = 
            getMap()->getMapOptions().elevationInterpolation() != INTERP_TRIANGULATE;

        // initialize a key node factory.
        factory = new TileGroupFactory(
            getMap(),
            this, // engine
            _geometryPool.get(),
            _liveTiles.get(),
            _deadTiles.get(),
            _renderBindings,
            _terrainOptions );
    }

    return factory.get();
}

osg::Node*
RexTerrainEngineNode::createNode(const TileKey&    key,
                                ProgressCallback* progress)
{
    // if the engine has been disconnected from the scene graph, bail out and don't
    // create any more tiles
    if ( getNumParents() == 0 )
        return 0L;

    OE_DEBUG << LC << "Create node for \"" << key.str() << "\"" << std::endl;

    // create the node:
    osg::ref_ptr<osg::Node> node = getTileGroupFactory()->createTileGroup( key, true, true, progress );
    
    // release the reference and return it.
    return node.release();
}

osg::Node*
RexTerrainEngineNode::createStandaloneNode(const TileKey&    key,
                                          ProgressCallback* progress)
{
    // if the engine has been disconnected from the scene graph, bail out and don't
    // create any more tiles
    if ( getNumParents() == 0 )
        return 0L;

    OE_DEBUG << LC << "Create standalone node for \"" << key.str() << "\"" << std::endl;

    return getTileGroupFactory()->createTileGroup( key, true, false, progress );
}

osg::Node*
RexTerrainEngineNode::createTile( const TileKey& key )
{
    // TODO: implement again.
    return 0L;

#if 0
    osg::ref_ptr<TileModel> model = new TileModel( _update_mapf->getRevision(), _update_mapf->getMapInfo() );
    model->_tileKey = key;
    model->_tileLocator = GeoLocator::createForKey(key, _update_mapf->getMapInfo());

    // Build the heightfield

    const MapInfo& mapInfo = _update_mapf->getMapInfo();

    const osgEarth::ElevationInterpolation& interp = _update_mapf->getMapOptions().elevationInterpolation().get();

    // Request a heightfield from the map, falling back on lower resolution tiles
    osg::ref_ptr<osg::HeightField> hf;    

    TileKey sampleKey = key;
    bool populated = false;
    if (_update_mapf->elevationLayers().size() > 0)
    {
        while (!populated)
        {
            populated = _update_mapf->populateHeightField(hf, sampleKey, true, SARexLE_FIRST_VALID);
            if (!populated)
            {
                // Fallback on the parent
                sampleKey = sampleKey.createParentKey();
                if (!sampleKey.valid())
                {
                    return 0;
                }
            }
        }       
    }

    if (!populated)
    {
        // We have no heightfield so just create a reference heightfield.
        hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), 15, 15 );
        sampleKey = key;
    }

    model->_elevationData = TileModel::ElevationData(
            hf,
            GeoLocator::createForKey( sampleKey, mapInfo ),
            false );        

    bool optimizeTriangleOrientation = getMap()->getMapOptions().elevationInterpolation() != INTERP_TRIANGULATE;

    osg::ref_ptr<TileModelCompiler> compiler = new TileModelCompiler(
            _update_mapf->terrainMaskLayers(),
            _update_mapf->modelLayers(),
            _primaryUnit,
            optimizeTriangleOrientation,
            _terrainOptions );

    return compiler->compile(model.get(), *_update_mapf, 0L);
#endif
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
                if ( getTextureCompositor()->reserveTextureImageUnit(temp) )
                {
                    unit = temp;
                    OE_INFO << LC << "Image unit " << temp << " assigned to shared layer " << layerAdded->getName() << std::endl;
                }
                else
                {
                    OE_WARN << LC << "Insufficient GPU image units to share layer " << layerAdded->getName() << std::endl;
                }
            }

            //TODO: need this or not??
            optional<std::string>& texMatUniformName = layerAdded->shareMatrixName();
            if ( !texMatUniformName.isSet() )
            {
                texMatUniformName = Stringify() << "oe_layer_" << layerAdded->getUID() << "_texMatrix";
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
                getTextureCompositor()->releaseTextureImageUnit( *layerRemoved->shareImageUnit() );
                layerRemoved->shareImageUnit().unset();
            }
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
        osg::StateSet* terrainStateSet = getTerrainStateSet();
        
        // required for multipass tile rendering to work
        terrainStateSet->setAttributeAndModes(
            new osg::Depth(osg::Depth::LEQUAL, 0, 1, true) );

        // activate standard mix blending.
        terrainStateSet->setAttributeAndModes( 
            new osg::BlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON );

        // install patch param if we are tessellation on the GPU.
        if ( _terrainOptions.gpuTessellation() == true )
        {
            terrainStateSet->setAttributeAndModes( new osg::PatchParameter(16) );
        }

        // install shaders, if we're using them.
        if ( Registry::capabilities().supportsGLSL() )
        {
            VirtualProgram* vp = new VirtualProgram();
            vp->setName( "osgEarth.RexTerrainEngineNode" );
            terrainStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );
            
            Shaders package;
            
            bool useTerrainColor = _terrainOptions.color().isSet();
            package.define("OE_REX_USE_TERRAIN_COLOR", useTerrainColor);
            if ( useTerrainColor )
            {
                terrainStateSet->addUniform(new osg::Uniform("oe_terrain_color", _terrainOptions.color().get()));
            }

            bool useBlending = _terrainOptions.enableBlending().get();
            package.define("OE_REX_USE_BLENDING", useBlending);

            // Vertex shader:
            package.loadFunction(vp, package.VERT_MODEL);
            package.loadFunction(vp, package.VERT_VIEW);
            package.loadFunction(vp, package.FRAG);

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
                                filter->install( terrainStateSet );
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

                    vp->setFunction(
                        "oe_rexEngine_applyFilters",
                        fs_colorfilters,
                        ShaderComp::LOCATION_FRAGMENT_COLORING,
                        0.0 );
                }
            }

            // binding for the terrain texture
            if ( _renderBindings.color().isActive() )
            {
                terrainStateSet->addUniform( new osg::Uniform(
                    _renderBindings.color().samplerName().c_str(),
                    _renderBindings.color().unit() ));
            }

            // binding for the secondary texture (for LOD blending)
            if ( _renderBindings.parentColor().isActive() )
            {
                terrainStateSet->addUniform( new osg::Uniform(
                    _renderBindings.parentColor().samplerName().c_str(),
                    _renderBindings.parentColor().unit() ));

                // binding for the default secondary texture matrix
                osg::Matrixf emptyMat;
                emptyMat(0,0) = 0.0f;
                terrainStateSet->addUniform( new osg::Uniform(
                    _renderBindings.parentColor().matrixName().c_str(),
                    emptyMat ));
            }
            
            // uniform for the elevation texture sampler.
            if ( _renderBindings.elevation().isActive() )
            {
                terrainStateSet->addUniform( new osg::Uniform(
                    _renderBindings.elevation().samplerName().c_str(),
                    _renderBindings.elevation().unit() ));
            }

            // uniform for the parent elevation texture sampler.
            if ( _renderBindings.parentElevation().isActive() )
            {
                terrainStateSet->addUniform( new osg::Uniform(
                    _renderBindings.parentElevation().samplerName().c_str(),
                    _renderBindings.parentElevation().unit() ));
            }
            
            // uniform for the normal texture sampler.
            if ( _renderBindings.normal().isActive() )
            {
                terrainStateSet->addUniform( new osg::Uniform(
                    _renderBindings.normal().samplerName().c_str(),
                    _renderBindings.normal().unit() ));
            }

            // uniform for the parent normal texture sampler.
            if ( _renderBindings.parentNormal().isActive() )
            {
                terrainStateSet->addUniform( new osg::Uniform(
                    _renderBindings.parentNormal().samplerName().c_str(),
                    _renderBindings.parentNormal().unit() ));
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

            // special object ID that denotes the terrain surface.
            terrainStateSet->addUniform( new osg::Uniform(
                Registry::objectIndex()->getObjectIDUniformName().c_str(), OSGEARTH_OBJECTID_TERRAIN) );

            // bind the shared layer uniforms.
            for(ImageLayerVector::const_iterator i = _update_mapf->imageLayers().begin(); i != _update_mapf->imageLayers().end(); ++i)
            {
                const ImageLayer* layer = i->get();
                if ( layer->isShared() )
                {
                    std::string texName = Stringify() << "oe_layer_" << layer->getUID() << "_tex";
                    terrainStateSet->addUniform( new osg::Uniform(texName.c_str(), layer->shareImageUnit().get()) );
                    OE_INFO << LC << "Layer \"" << layer->getName() << "\" in uniform \"" << texName << "\"\n";
                }
            }
        }

        _stateUpdateRequired = false;
    }
}
