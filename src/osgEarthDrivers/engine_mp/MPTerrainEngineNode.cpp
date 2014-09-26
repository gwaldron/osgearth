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
#include "MPTerrainEngineNode"
#include "SingleKeyNodeFactory"
#include "TerrainNode"
#include "TileModelFactory"
#include "TileModelCompiler"
#include "TilePagedLOD"

#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderFactory>
#include <osgEarth/MapModelChange>
#include <osgEarth/Progress>

#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/PagedLOD>
#include <osg/Timer>
#include <osg/Depth>
#include <osg/BlendFunc>
#include <osgDB/DatabasePager>

#define LC "[MPTerrainEngineNode] "

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    // adapter that lets MPTerrainEngineNode listen to Map events
    struct MPTerrainEngineNodeMapCallbackProxy : public MapCallback
    {
        MPTerrainEngineNodeMapCallbackProxy(MPTerrainEngineNode* node) : _node(node) { }
        osg::observer_ptr<MPTerrainEngineNode> _node;

        void onMapInfoEstablished( const MapInfo& mapInfo ) {
            osg::ref_ptr<MPTerrainEngineNode> node;
            if ( _node.lock(node) )
                node->onMapInfoEstablished( mapInfo );
        }

        void onMapModelChanged( const MapModelChange& change ) {
            osg::ref_ptr<MPTerrainEngineNode> node;
            if ( _node.lock(node) )
                node->onMapModelChanged( change );
        }
    };
}

//---------------------------------------------------------------------------

static Threading::ReadWriteMutex s_engineNodeCacheMutex;
//Caches the MapNodes that have been created
typedef std::map<UID, osg::observer_ptr<MPTerrainEngineNode> > EngineNodeCache;

static
EngineNodeCache& getEngineNodeCache()
{
    static EngineNodeCache s_cache;
    return s_cache;
}

void
MPTerrainEngineNode::registerEngine(MPTerrainEngineNode* engineNode)
{
    Threading::ScopedWriteLock exclusiveLock( s_engineNodeCacheMutex );
    getEngineNodeCache()[engineNode->_uid] = engineNode;
    OE_DEBUG << LC << "Registered engine " << engineNode->_uid << std::endl;
}

void
MPTerrainEngineNode::unregisterEngine( UID uid )
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
MPTerrainEngineNode::getEngineByUID( UID uid, osg::ref_ptr<MPTerrainEngineNode>& output )
{
    Threading::ScopedReadLock sharedLock( s_engineNodeCacheMutex );
    EngineNodeCache::const_iterator k = getEngineNodeCache().find( uid );
    if (k != getEngineNodeCache().end())
        output = k->second.get();
}

UID
MPTerrainEngineNode::getUID() const
{
    return _uid;
}

//------------------------------------------------------------------------

MPTerrainEngineNode::ElevationChangedCallback::ElevationChangedCallback( MPTerrainEngineNode* terrain ):
_terrain( terrain )
{
    //nop
}

void
MPTerrainEngineNode::ElevationChangedCallback::onVisibleChanged( TerrainLayer* layer )
{
    _terrain->refresh(true); // true => force a dirty
}

//------------------------------------------------------------------------

MPTerrainEngineNode::MPTerrainEngineNode() :
TerrainEngineNode     ( ),
_terrain              ( 0L ),
_update_mapf          ( 0L ),
_tileCount            ( 0 ),
_tileCreationTime     ( 0.0 ),
_primaryUnit          ( -1 ),
_secondaryUnit        ( -1 ),
_batchUpdateInProgress( false ),
_refreshRequired      ( false ),
_stateUpdateRequired  ( false )
{
    _uid = Registry::instance()->createUID();

    // install an elevation callback so we can update elevation data
    _elevationCallback = new ElevationChangedCallback( this );
}

MPTerrainEngineNode::~MPTerrainEngineNode()
{
    unregisterEngine( _uid );

    if ( _update_mapf )
    {
        delete _update_mapf;
    }
}

void
MPTerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::preInitialize( map, options );
    //nop.
}

void
MPTerrainEngineNode::postInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::postInitialize( map, options );

    // Initialize the map frames. We need one for the update thread and one for the
    // cull thread. Someday we can detect whether these are actually the same thread
    // (depends on the viewer's threading mode).
    _update_mapf = new MapFrame( map, Map::ENTIRE_MODEL, "mp-update" );

    // merge in the custom options:
    _terrainOptions.merge( options );

    // A shared registry for tile nodes in the scene graph. Enable revision tracking
    // if requested in the options. Revision tracking lets the registry notify all
    // live tiles of the current map revision so they can inrementally update
    // themselves if necessary.
    _liveTiles = new TileNodeRegistry("live");
    _liveTiles->setRevisioningEnabled( _terrainOptions.incrementalUpdate() == true );
    _liveTiles->setMapRevision( _update_mapf->getRevision() );

    // set up a registry for quick release:
    if ( _terrainOptions.quickReleaseGLObjects() == true )
    {
        _deadTiles = new TileNodeRegistry("dead");
    }
    
    // initialize the model factory:
    _tileModelFactory = new TileModelFactory(_liveTiles.get(), _terrainOptions );

    // handle an already-established map profile:
    if ( _update_mapf->getProfile() )
    {
        // NOTE: this will initialize the map with the startup layers
        onMapInfoEstablished( MapInfo(map) );
    }

    // install a layer callback for processing further map actions:
    map->addMapCallback( new MPTerrainEngineNodeMapCallbackProxy(this) );

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

    // set up the initial shaders
    updateState();

    // register this instance to the osgDB plugin can find it.
    registerEngine( this );

    // now that we have a map, set up to recompute the bounds
    dirtyBound();

    OE_INFO << LC << "Edge normalization is " << (_terrainOptions.normalizeEdges() == true? "ON" : "OFF") << std::endl;
}


osg::BoundingSphere
MPTerrainEngineNode::computeBound() const
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
MPTerrainEngineNode::invalidateRegion(const GeoExtent& extent,
                                      unsigned         minLevel,
                                      unsigned         maxLevel)
{
    if (_terrainOptions.incrementalUpdate() == true && _liveTiles.valid())
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
MPTerrainEngineNode::refresh(bool forceDirty)
{
    if ( _batchUpdateInProgress )
    {
        _refreshRequired = true;
    }
    else
    {
        if ( _terrainOptions.incrementalUpdate() == true )
        {
            // run an atomic "dirty" operation:
            //_update_mapf->sync();
            //_liveTiles->setMapRevision( _update_mapf->getRevision(), forceDirty );
        }
        else
        {
            // remove the old one:
            this->removeChild( _terrain );

            // and create a new one.
            createTerrain();
        }

        _refreshRequired = false;
    }
}

void
MPTerrainEngineNode::onMapInfoEstablished( const MapInfo& mapInfo )
{
    createTerrain();
}

bool reg = false;

void
MPTerrainEngineNode::createTerrain()
{
    // scrub the heightfield cache.
    if (_tileModelFactory)
        _tileModelFactory->getHeightFieldCache()->clear();

    // New terrain
    _terrain = new TerrainNode( _deadTiles.get() );
    this->addChild( _terrain );

    // Enable blending on the terrain node; this will result in the underlying
    // "empty" globe being transparent instead of white.
    if (_terrainOptions.enableBlending().value())
    {
        _terrain->getOrCreateStateSet()->setMode(GL_BLEND , osg::StateAttribute::ON);
    }

    // reserve GPU space.
    if ( _primaryUnit < 0 )
    {
        this->getTextureCompositor()->reserveTextureImageUnit( _primaryUnit );
    }
    if ( _secondaryUnit < 0 )
    {
        this->getTextureCompositor()->reserveTextureImageUnit( _secondaryUnit );
    }

    // Factory to create the root keys:
    KeyNodeFactory* factory = getKeyNodeFactory();

    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    std::vector< TileKey > keys;
    _update_mapf->getProfile()->getAllKeysAtLOD( *_terrainOptions.firstLOD(), keys );

    // create a root node for each root tile key.
    OE_INFO << LC << "Creating " << keys.size() << " root keys.." << std::endl;

    TilePagedLOD* root = new TilePagedLOD( _uid, _liveTiles, _deadTiles );
    //osg::Group* root = new osg::Group();
    _terrain->addChild( root );

    osg::ref_ptr<osgDB::Options> dbOptions = Registry::instance()->cloneOrCreateOptions();

    unsigned child = 0;
    for( unsigned i=0; i<keys.size(); ++i )
    {
        osg::ref_ptr<osg::Node> node = factory->createNode( keys[i], true, true, 0L );
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

    _rootTilesRegistered = false;

    updateState();
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
MPTerrainEngineNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {

#if 0 // believe this is now unnecessary

        // since the root tiles are manually added, the pager never has a chance to 
        // register the PagedLODs in their children. So we have to do it manually here.
        if ( !_rootTilesRegistered )
        {
            Threading::ScopedMutexLock lock(_rootTilesRegisteredMutex);

            if ( !_rootTilesRegistered )
            {
                osgDB::DatabasePager* pager = dynamic_cast<osgDB::DatabasePager*>(nv.getDatabaseRequestHandler());
                if ( pager )
                {
                    //OE_WARN << "Registering plods." << std::endl;
                    pager->registerPagedLODs( _terrain );
                    _rootTilesRegistered = true;
                }
            }
        }
#endif

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


KeyNodeFactory*
MPTerrainEngineNode::getKeyNodeFactory()
{
    osg::ref_ptr<KeyNodeFactory>& knf = _perThreadKeyNodeFactories.get(); // thread-safe get
    if ( !knf.valid() )
    {
        // create a compiler for compiling tile models into geometry
        bool optimizeTriangleOrientation = 
            getMap()->getMapOptions().elevationInterpolation() != INTERP_TRIANGULATE;

        // A compiler specific to this thread:
        TileModelCompiler* compiler = new TileModelCompiler(
            _update_mapf->terrainMaskLayers(),
            _update_mapf->modelLayers(),
            _primaryUnit,
            optimizeTriangleOrientation,
            _terrainOptions );

        // initialize a key node factory.
        knf = new SingleKeyNodeFactory(
            getMap(),
            _tileModelFactory.get(),
            compiler,
            _liveTiles.get(),
            _deadTiles.get(),
            _terrainOptions,
            _uid );
    }

    return knf.release();
}

osg::Node*
MPTerrainEngineNode::createNode(const TileKey&    key,
                                ProgressCallback* progress)
{
    // if the engine has been disconnected from the scene graph, bail out and don't
    // create any more tiles
    if ( getNumParents() == 0 )
        return 0L;

    OE_DEBUG << LC << "Create node for \"" << key.str() << "\"" << std::endl;

    return getKeyNodeFactory()->createNode( key, true, true, progress );
}

osg::Node*
MPTerrainEngineNode::createStandaloneNode(const TileKey&    key,
                                          ProgressCallback* progress)
{
    // if the engine has been disconnected from the scene graph, bail out and don't
    // create any more tiles
    if ( getNumParents() == 0 )
        return 0L;

    OE_DEBUG << LC << "Create standalone node for \"" << key.str() << "\"" << std::endl;

    return getKeyNodeFactory()->createNode( key, true, false, progress );
}

osg::Node*
MPTerrainEngineNode::createTile( const TileKey& key )
{
    // make a node, but don't include any subtile information and don't
    // accumulate data from parents.
    return getKeyNodeFactory()->createNode( key, false, false, 0L );
}


void
MPTerrainEngineNode::onMapModelChanged( const MapModelChange& change )
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
MPTerrainEngineNode::addImageLayer( ImageLayer* layerAdded )
{
    if ( layerAdded )
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
        }
    }

    refresh();
}


void
MPTerrainEngineNode::removeImageLayer( ImageLayer* layerRemoved )
{
    if ( layerRemoved )
    {
        // for a shared layer, release the shared image unit.
        if ( layerRemoved->isShared() )
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
MPTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
    updateState();
}

void
MPTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    if ( !layer )
        return;

    layer->addCallback( _elevationCallback.get() );

    refresh();
}

void
MPTerrainEngineNode::removeElevationLayer( ElevationLayer* layerRemoved )
{
    layerRemoved->removeCallback( _elevationCallback.get() );

    refresh();
}

void
MPTerrainEngineNode::moveElevationLayer( unsigned int oldIndex, unsigned int newIndex )
{
    refresh();
}

void
MPTerrainEngineNode::toggleElevationLayer( ElevationLayer* layer )
{
    refresh();
}


// Generates the main shader code for rendering the terrain.
void
MPTerrainEngineNode::updateState()
{
    if ( _batchUpdateInProgress )
    {
        _stateUpdateRequired = true;
    }
    else
    {
        osg::StateSet* terrainStateSet = _terrain->getOrCreateStateSet();
        
        // required for multipass tile rendering to work
        terrainStateSet->setAttributeAndModes(
            new osg::Depth(osg::Depth::LEQUAL, 0, 1, true) );

        // activate standard mix blending.
        terrainStateSet->setAttributeAndModes( 
            new osg::BlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON );

        // install shaders, if we're using them.
        if ( Registry::capabilities().supportsGLSL() )
        {
            VirtualProgram* vp = new VirtualProgram();
            vp->setName( "osgEarth.engine_mp.TerrainNode" );
            terrainStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );

            // bind the vertex attributes generated by the tile compiler.
            vp->addBindAttribLocation( "oe_terrain_attr",  osg::Drawable::ATTRIBUTE_6 );
            vp->addBindAttribLocation( "oe_terrain_attr2", osg::Drawable::ATTRIBUTE_7 );

            // Vertex shader:
            std::string vs = Stringify() <<
                "#version " GLSL_VERSION_STR "\n"
                GLSL_DEFAULT_PRECISION_FLOAT "\n"
                "varying vec4 oe_layer_texc;\n"
                "varying vec4 oe_layer_tilec;\n"
                "void oe_mp_setup_coloring(inout vec4 VertexModel) \n"
                "{ \n"
                "    oe_layer_texc  = gl_MultiTexCoord" << _primaryUnit << ";\n"
                "    oe_layer_tilec = gl_MultiTexCoord" << _secondaryUnit << ";\n"
                "}\n";

            bool useTerrainColor = _terrainOptions.color().isSet();

            bool useBlending = _terrainOptions.enableBlending() == true;

            // Fragment Shader for normal blending:
            std::string fs = Stringify() <<
                "#version " GLSL_VERSION_STR "\n"
                GLSL_DEFAULT_PRECISION_FLOAT "\n"
                "varying vec4 oe_layer_texc; \n"
                "uniform sampler2D oe_layer_tex; \n"
                "uniform int oe_layer_uid; \n"
                "uniform int oe_layer_order; \n"
                "uniform float oe_layer_opacity; \n"
                << (useTerrainColor ?
                "uniform vec4 oe_terrain_color; \n" : ""
                ) <<
                "void oe_mp_apply_coloring(inout vec4 color) \n"
                "{ \n"
                << (useTerrainColor ?
                "    color = oe_terrain_color; \n" : ""
                ) <<
                "    vec4 texel; \n"
                "    if ( oe_layer_uid >= 0 ) { \n"
                "        texel = texture2D(oe_layer_tex, oe_layer_texc.st); \n"
                "        texel.a *= oe_layer_opacity; \n"
                "    } \n"
                "    else { \n"
                "        texel = color; \n"
                "    }\n"
                << (useBlending ?
                "    if ( oe_layer_order == 0 ) { \n"
                "        color = texel*texel.a + color*(1.0-texel.a); \n" // simulate src_alpha, 1-src_alpha blens
                "    } \n"
                "    else \n" : ""
                ) <<
                "        color = texel; \n"
                "} \n";

            // Color filter frag function:
            std::string fs_colorfilters =
                "#version " GLSL_VERSION_STR "\n"
                GLSL_DEFAULT_PRECISION_FLOAT "\n"
                "uniform int oe_layer_uid; \n"
                "__COLOR_FILTER_HEAD__"
                "void oe_mp_apply_filters(inout vec4 color) \n"
                "{ \n"
                    "__COLOR_FILTER_BODY__"
                "} \n";

            vp->setFunction( "oe_mp_setup_coloring", vs, ShaderComp::LOCATION_VERTEX_MODEL, 0.0 );
            vp->setFunction( "oe_mp_apply_coloring", fs, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.0 );

            // assemble color filter code snippets.
            bool haveColorFilters = false;
            {
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

                    replaceIn( fs_colorfilters, "__COLOR_FILTER_HEAD__", cf_head_str );
                    replaceIn( fs_colorfilters, "__COLOR_FILTER_BODY__", cf_body_str );

                    vp->setFunction( "oe_mp_apply_filters", fs_colorfilters, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.0 );
                }
            }

            // binding for the terrain texture
            terrainStateSet->getOrCreateUniform( 
                "oe_layer_tex", osg::Uniform::SAMPLER_2D )->set( _primaryUnit );

            // binding for the secondary texture (for LOD blending)
            terrainStateSet->getOrCreateUniform(
                "oe_layer_tex_parent", osg::Uniform::SAMPLER_2D )->set( _secondaryUnit );

            // binding for the default secondary texture matrix
            osg::Matrixf parent_mat;
            parent_mat(0,0) = 0.0f;
            terrainStateSet->getOrCreateUniform(
                "oe_layer_parent_matrix", osg::Uniform::FLOAT_MAT4 )->set( parent_mat );

            // uniform that controls per-layer opacity
            terrainStateSet->getOrCreateUniform(
                "oe_layer_opacity", osg::Uniform::FLOAT )->set( 1.0f );

            // uniform that conveys the layer UID to the shaders; necessary
            // for per-layer branching (like color filters)
            // UID -1 => no image layer (no texture)
            terrainStateSet->getOrCreateUniform(
                "oe_layer_uid", osg::Uniform::INT )->set( -1 );

            // uniform that conveys the render order, since the shaders
            // need to know which is the first layer in order to blend properly
            terrainStateSet->getOrCreateUniform(
                "oe_layer_order", osg::Uniform::INT )->set( 0 );

            // base terrain color.
            if ( useTerrainColor )
            {
                terrainStateSet->getOrCreateUniform(
                    "oe_terrain_color", osg::Uniform::FLOAT_VEC4 )->set( *_terrainOptions.color() );
            }
        }

        _stateUpdateRequired = false;
    }
}
