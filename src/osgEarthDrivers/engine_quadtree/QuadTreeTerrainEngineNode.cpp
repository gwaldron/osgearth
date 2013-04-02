/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include "QuadTreeTerrainEngineNode"
#include "SerialKeyNodeFactory"
#include "TerrainNode"
#include "TileModelFactory"
#include "TileModelCompiler"

#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderFactory>
#include <osgEarth/MapModelChange>

#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/PagedLOD>
#include <osg/Timer>

#define LC "[QuadTreeTerrainEngineNode] "

using namespace osgEarth_engine_quadtree;
using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    // adapter that lets QuadTreeTerrainEngineNode listen to Map events
    struct QuadTreeTerrainEngineNodeMapCallbackProxy : public MapCallback
    {
        QuadTreeTerrainEngineNodeMapCallbackProxy(QuadTreeTerrainEngineNode* node) : _node(node) { }
        osg::observer_ptr<QuadTreeTerrainEngineNode> _node;

        void onMapInfoEstablished( const MapInfo& mapInfo ) {
            _node->onMapInfoEstablished( mapInfo );
        }

        void onMapModelChanged( const MapModelChange& change ) {
            _node->onMapModelChanged( change );
        }
    };
}

//---------------------------------------------------------------------------

static Threading::ReadWriteMutex s_engineNodeCacheMutex;
//Caches the MapNodes that have been created
typedef std::map<UID, osg::observer_ptr<QuadTreeTerrainEngineNode> > EngineNodeCache;

static
EngineNodeCache& getEngineNodeCache()
{
    static EngineNodeCache s_cache;
    return s_cache;
}

void
QuadTreeTerrainEngineNode::registerEngine(QuadTreeTerrainEngineNode* engineNode)
{
    Threading::ScopedWriteLock exclusiveLock( s_engineNodeCacheMutex );
    getEngineNodeCache()[engineNode->_uid] = engineNode;
    OE_DEBUG << LC << "Registered engine " << engineNode->_uid << std::endl;
}

void
QuadTreeTerrainEngineNode::unregisterEngine( UID uid )
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
QuadTreeTerrainEngineNode::getEngineByUID( UID uid, osg::ref_ptr<QuadTreeTerrainEngineNode>& output )
{
    Threading::ScopedReadLock sharedLock( s_engineNodeCacheMutex );
    EngineNodeCache::const_iterator k = getEngineNodeCache().find( uid );
    if (k != getEngineNodeCache().end())
        output = k->second.get();
}

UID
QuadTreeTerrainEngineNode::getUID() const
{
    return _uid;
}

//------------------------------------------------------------------------

QuadTreeTerrainEngineNode::ElevationChangedCallback::ElevationChangedCallback( QuadTreeTerrainEngineNode* terrain ):
_terrain( terrain )
{
}

void
QuadTreeTerrainEngineNode::ElevationChangedCallback::onVisibleChanged( TerrainLayer* layer )
{    
    osgEarth::Registry::instance()->clearBlacklist();
    _terrain->refresh();
}

//------------------------------------------------------------------------

QuadTreeTerrainEngineNode::QuadTreeTerrainEngineNode() :
TerrainEngineNode(),
_terrain         ( 0L ),
_update_mapf     ( 0L ),
_tileCount       ( 0 ),
_tileCreationTime( 0.0 )
{
    _uid = Registry::instance()->createUID();

    // install an elevation callback so we can update elevation data
    _elevationCallback = new ElevationChangedCallback( this );
}

QuadTreeTerrainEngineNode::~QuadTreeTerrainEngineNode()
{
    unregisterEngine( _uid );

    if ( _update_mapf )
    {
        delete _update_mapf;
    }
}

void
QuadTreeTerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::preInitialize( map, options );
}

void
QuadTreeTerrainEngineNode::postInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::postInitialize( map, options );

    // Initialize the map frames. We need one for the update thread and one for the
    // cull thread. Someday we can detect whether these are actually the same thread
    // (depends on the viewer's threading mode).
    _update_mapf = new MapFrame( map, Map::MASKED_TERRAIN_LAYERS, "quadtree-update" );

    // merge in the custom options:
    _terrainOptions.merge( options );

    // a shared registry for tile nodes in the scene graph.
    _liveTiles = new TileNodeRegistry("live");

    // set up a registry for quick release:
    if ( _terrainOptions.quickReleaseGLObjects() == true )
    {
        _deadTiles = new TileNodeRegistry("dead");
    }
    
    // initialize the model factory:
    _tileModelFactory = new TileModelFactory(getMap(), _liveTiles.get(), _terrainOptions );


    // handle an already-established map profile:
    if ( _update_mapf->getProfile() )
    {
        // NOTE: this will initialize the map with the startup layers
        onMapInfoEstablished( MapInfo(map) );
    }

    // populate the terrain with whatever data is in the map to begin with:
    if ( _terrain )
    {
        updateTextureCombining();
    }

    // install a layer callback for processing further map actions:
    map->addMapCallback( new QuadTreeTerrainEngineNodeMapCallbackProxy(this) );

    // Attach to all of the existing elevation layers
    ElevationLayerVector elevationLayers;
    map->getElevationLayers( elevationLayers );
    for( ElevationLayerVector::const_iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i )
    {
        i->get()->addCallback( _elevationCallback.get() );
    }

    // register this instance to the osgDB plugin can find it.
    registerEngine( this );

    // now that we have a map, set up to recompute the bounds
    dirtyBound();


}


osg::BoundingSphere
QuadTreeTerrainEngineNode::computeBound() const
{
    if ( _terrain && _terrain->getNumChildren() > 0 )
    {
        return _terrain->getBound();
    }
    else
    {
        return TerrainEngineNode::computeBound();
    }
}


void
QuadTreeTerrainEngineNode::refresh()
{

    //Clear out the hf cache
    if (_tileModelFactory)
    {
        _tileModelFactory->getHeightFieldCache()->clear();
    }

    // rebuilds the terrain graph entirely.

    this->removeChild( _terrain );

    _terrain = new TerrainNode( _deadTiles.get() );

    const MapInfo& mapInfo = _update_mapf->getMapInfo();

    KeyNodeFactory* factory = getKeyNodeFactory();

    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    std::vector< TileKey > keys;
    _update_mapf->getProfile()->getAllKeysAtLOD( *_terrainOptions.firstLOD(), keys );

    if (_terrainOptions.enableBlending().value())
    {
        _terrain->getOrCreateStateSet()->setMode(GL_BLEND , osg::StateAttribute::ON); 
    }

    this->addChild( _terrain );

    // create a root node for each root tile key.
    for( unsigned i=0; i<keys.size(); ++i )
    {
        osg::Node* node = factory->createRootNode( keys[i] );
        if ( node )
            _terrain->addChild( node );
        else
            OE_WARN << LC << "Couldn't make tile for root key: " << keys[i].str() << std::endl;
    }

    updateTextureCombining();
}

void
QuadTreeTerrainEngineNode::onMapInfoEstablished( const MapInfo& mapInfo )
{
    // create the root terrai node.
    _terrain = new TerrainNode( _deadTiles.get() );

    this->addChild( _terrain );

    //// set the initial properties from the options structure:
    //_terrain->setVerticalScale( _terrainOptions.verticalScale().value() );
    //_terrain->setSampleRatio  ( _terrainOptions.heightFieldSampleRatio().value() );

    if (_terrainOptions.enableBlending().value())
    {
        _terrain->getOrCreateStateSet()->setMode(GL_BLEND , osg::StateAttribute::ON);    
    }

    OE_INFO << LC << "Sample ratio = " << _terrainOptions.heightFieldSampleRatio().value() << std::endl;

    // install the shader program, if applicable:
    installShaders();

    KeyNodeFactory* factory = getKeyNodeFactory();

    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    std::vector< TileKey > keys;
    _update_mapf->getProfile()->getAllKeysAtLOD( *_terrainOptions.firstLOD(), keys );

    for( unsigned i=0; i<keys.size(); ++i )
    {
        osg::Node* node = factory->createRootNode( keys[i] );
        if ( node )
            _terrain->addChild( node );
        else
            OE_WARN << LC << "Couldn't make tile for root key: " << keys[i].str() << std::endl;
    }

    // we just added the root tiles, so mark the bound in need of recomputation.
    this->dirtyBound();
}


KeyNodeFactory*
QuadTreeTerrainEngineNode::getKeyNodeFactory()
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
            _texCompositor.get(),
            optimizeTriangleOrientation,
            _terrainOptions );

        // initialize a key node factory.
        knf = new SerialKeyNodeFactory( 
            _tileModelFactory.get(),
            compiler,
            _liveTiles.get(),
            _deadTiles.get(),
            _terrainOptions, 
            MapInfo( getMap() ),
            _terrain, 
            _uid );
    }

    return knf.get();
}


osg::Node*
QuadTreeTerrainEngineNode::createNode( const TileKey& key )
{
    // if the engine has been disconnected from the scene graph, bail out and don't
    // create any more tiles
    if ( getNumParents() == 0 ) //|| !_keyNodeFactory.valid() )
        return 0L;

    OE_DEBUG << LC << "Create node for \"" << key.str() << "\"" << std::endl;

    osg::Node* result =  getKeyNodeFactory()->createNode( key );
    return result;
}

osg::Node*
QuadTreeTerrainEngineNode::createTile( const TileKey& key )
{
    return getKeyNodeFactory()->createNode( key );
}


void
QuadTreeTerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
    // update the thread-safe map model copy:
    _update_mapf->sync();

    // dispatch the change handler
    if ( change.getLayer() )
    {
        // first inform the texture compositor with the new model changes:
        if ( _texCompositor.valid() && change.getImageLayer() )
        {
            _texCompositor->applyMapModelChange( change );
        }

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
        case MapModelChange::ADD_MODEL_LAYER:
        case MapModelChange::REMOVE_MODEL_LAYER:
        case MapModelChange::MOVE_MODEL_LAYER:
        default: 
            break;
        }
    }
}


void
QuadTreeTerrainEngineNode::addImageLayer( ImageLayer* layerAdded )
{
    refresh();
}


void
QuadTreeTerrainEngineNode::removeImageLayer( ImageLayer* layerRemoved )
{
    refresh();
}


void
QuadTreeTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
#if 0
    // take a thread-safe copy of the tile table
    TileNodeVector tiles;
    _terrain->getTiles( tiles );

    for (TileNodeVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        TileNode* tile = itr->get();
        //tile->applyImmediateTileUpdate( TileUpdate::MOVE_IMAGE_LAYER );
        OE_WARN << LC << "moveImageLayer under review" << std::endl;
    }
#endif

    updateTextureCombining();
}


#if 0
void
QuadTreeTerrainEngineNode::updateElevation( TileNode* tile )
{
    Threading::ScopedWriteLock exclusiveLock( tile->getTileLayersMutex() );

    const TileKey& key = tile->getKey();

    bool hasElevation = _update_mapf->elevationLayers().size() > 0;

    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    if (heightFieldLayer)
    {
        osg::ref_ptr<osg::HeightField> hf;

        if ( hasElevation )
        {
            _update_mapf->getHeightField( key, true, hf, 0L);
        }

        if ( !hf.valid() )
        {
            hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), 8, 8 );
        }

        // update the heightfield:
        heightFieldLayer->setHeightField( hf.get() );
        hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );

        // TODO: review this in favor of a tile update...
        tile->setDirty( true );
    }
}

#endif

void
QuadTreeTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    if ( !layer )
        return;

    layer->addCallback( _elevationCallback.get() );

    refresh();
}

void
QuadTreeTerrainEngineNode::removeElevationLayer( ElevationLayer* layerRemoved )
{
    layerRemoved->removeCallback( _elevationCallback.get() );

    refresh();
}

void
QuadTreeTerrainEngineNode::moveElevationLayer( unsigned int oldIndex, unsigned int newIndex )
{
    refresh();
}

void
QuadTreeTerrainEngineNode::validateTerrainOptions( TerrainOptions& options )
{
    TerrainEngineNode::validateTerrainOptions( options );
    
    //nop for now.
    //note: to validate plugin-specific features, we would create an QuadTreeTerrainEngineOptions
    // and do the validation on that. You would then re-integrate it by calling
    // options.mergeConfig( osgTerrainOptions ).
}

void
QuadTreeTerrainEngineNode::installShaders()
{
    // This method installs a default shader setup on the engine node itself. The texture compositor
    // can then override parts of the program by using a VirtualProgram on the _terrain node. We do
    // it this way so that the developer has the option of removing this top-level shader program,
    // replacing it, or migrating it higher up the scene graph if necessary.

    //if ( _texCompositor.valid() && _texCompositor->usesShaderComposition() )
    //{
    //    const ShaderFactory* sf = Registry::instance()->getShaderFactory();

    //    int numLayers = osg::maximum( 1, (int)_update_mapf->imageLayers().size() );

    //    VirtualProgram* vp = new VirtualProgram();
    //    vp->setName( "engine_quadtree:EngineNode" );
    //    vp->installDefaultColoringAndLightingShaders( numLayers );

    //    getOrCreateStateSet()->setAttributeAndModes( vp, osg::StateAttribute::ON );
    //}
}

void
QuadTreeTerrainEngineNode::updateTextureCombining()
{
    if ( _texCompositor.valid() )
    {
        int numImageLayers = _update_mapf->imageLayers().size();
        osg::StateSet* terrainStateSet = _terrain->getOrCreateStateSet();

        if ( _texCompositor->usesShaderComposition() )
        {
            // Creates or updates the shader components that are generated by the texture compositor.
            // These components reside in the CustomTerrain's stateset, and override the components
            // installed in the VP on the engine-node's stateset in installShaders().
            VirtualProgram* vp = new VirtualProgram();
            vp->setName( "engine_quadtree:TerrainNode" );

            terrainStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );

            // first, update the default shader components based on the new layer count:
            const ShaderFactory* sf = Registry::instance()->getShaderFactory();
            
            // second, install the per-layer color filter functions.
            for( int i=0; i<numImageLayers; ++i )
            {
                std::string layerFilterFunc = Stringify() << "osgearth_runColorFilters_" << i;
                const ColorFilterChain& chain = _update_mapf->getImageLayerAt(i)->getColorFilters();

                // install the wrapper function that calls all the color filters in turn:
                vp->setShader( layerFilterFunc, sf->createColorFilterChainFragmentShader(layerFilterFunc, chain) );

                // install each of the filter entry points:
                for( ColorFilterChain::const_iterator j = chain.begin(); j != chain.end(); ++j )
                {
                    const ColorFilter* filter = j->get();
                    filter->install( terrainStateSet );
                }
            }
        }

        // next, inform the compositor that it needs to update based on a new layer count:
        _texCompositor->updateMasterStateSet( terrainStateSet );
    }
}


namespace
{
    class UpdateElevationVisitor : public osg::NodeVisitor
    {
    public:
        UpdateElevationVisitor( TileModelCompiler* compiler ):
          osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
          _compiler(compiler)
          {}

          void apply(osg::Node& node)
          {
              TileNode* tile = dynamic_cast<TileNode*>(&node);
              if (tile)
              {
                  tile->compile( _compiler );
                  //tile->applyImmediateTileUpdate(TileUpdate::UPDATE_ELEVATION);
              }

              traverse(node);
          }

          TileModelCompiler* _compiler;
    };
}


void
QuadTreeTerrainEngineNode::onVerticalScaleChanged()
{
//    _terrain->setVerticalScale(getVerticalScale());
    _terrainOptions.verticalScale() = getVerticalScale();
    UpdateElevationVisitor visitor( getKeyNodeFactory()->getCompiler() );
    this->accept(visitor);
}
