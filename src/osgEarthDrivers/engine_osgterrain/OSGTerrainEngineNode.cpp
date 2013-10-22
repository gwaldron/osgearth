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
#include "OSGTerrainEngineNode"
#include "MultiPassTerrainTechnique"
#include "ParallelKeyNodeFactory"
#include "SinglePassTerrainTechnique"
#include "TerrainNode"
#include "StreamingTerrainNode"
#include "TileBuilder"
#include "TransparentLayer"

#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderFactory>
#include <osgEarth/MapModelChange>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/PagedLOD>
#include <osg/Timer>

#define LC "[OSGTerrainEngine] "

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;

//------------------------------------------------------------------------

// adapter that lets OSGTerrainEngineNode listen to Map events
struct OSGTerrainEngineNodeMapCallbackProxy : public MapCallback
{
    OSGTerrainEngineNodeMapCallbackProxy(OSGTerrainEngineNode* node) : _node(node) { }
    osg::observer_ptr<OSGTerrainEngineNode> _node;

    void onMapInfoEstablished( const MapInfo& mapInfo ) {
        _node->onMapInfoEstablished( mapInfo );
    }

    void onMapModelChanged( const MapModelChange& change ) {
        _node->onMapModelChanged( change );
    }
};

//---------------------------------------------------------------------------

//static
//static OpenThreads::ReentrantMutex s_engineNodeCacheMutex;
static Threading::ReadWriteMutex s_engineNodeCacheMutex;
//Caches the MapNodes that have been created
typedef std::map<UID, osg::observer_ptr<OSGTerrainEngineNode> > EngineNodeCache;

static
EngineNodeCache& getEngineNodeCache()
{
    static EngineNodeCache s_cache;
    return s_cache;
}

void
OSGTerrainEngineNode::registerEngine(OSGTerrainEngineNode* engineNode)
{
    Threading::ScopedWriteLock exclusiveLock( s_engineNodeCacheMutex );
    getEngineNodeCache()[engineNode->_uid] = engineNode;
    OE_DEBUG << LC << "Registered engine " << engineNode->_uid << std::endl;
}

void
OSGTerrainEngineNode::unregisterEngine( UID uid )
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
OSGTerrainEngineNode::getEngineByUID( UID uid, osg::ref_ptr<OSGTerrainEngineNode>& output )
{
    Threading::ScopedReadLock sharedLock( s_engineNodeCacheMutex );
    EngineNodeCache::const_iterator k = getEngineNodeCache().find( uid );
    if (k != getEngineNodeCache().end())
        output = k->second.get();
}

UID
OSGTerrainEngineNode::getUID() const
{
    return _uid;
}

//------------------------------------------------------------------------

OSGTerrainEngineNode::ElevationChangedCallback::ElevationChangedCallback( OSGTerrainEngineNode* terrain ):
_terrain( terrain )
{
}

void
OSGTerrainEngineNode::ElevationChangedCallback::onVisibleChanged( TerrainLayer* layer )
{    
    osgEarth::Registry::instance()->clearBlacklist();
    _terrain->refresh();
}

//------------------------------------------------------------------------

OSGTerrainEngineNode::OSGTerrainEngineNode() :
TerrainEngineNode(),
_terrain         ( 0L ),
_update_mapf     ( 0L ),
_cull_mapf       ( 0L ),
_tileCount       ( 0 ),
_tileCreationTime( 0.0 )
{
    _uid = Registry::instance()->createUID();
    _taskServiceMgr = Registry::instance()->getTaskServiceManager();

    _elevationCallback = new ElevationChangedCallback( this );
}

OSGTerrainEngineNode::~OSGTerrainEngineNode()
{
    unregisterEngine( _uid );

    if ( _update_mapf )
    {
        delete _update_mapf;
    }

    if ( _cull_mapf )
    {
        delete _cull_mapf;
    }
}

void
OSGTerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::preInitialize( map, options );

    _isStreaming =
        options.loadingPolicy()->mode() == LoadingPolicy::MODE_PREEMPTIVE ||
        options.loadingPolicy()->mode() == LoadingPolicy::MODE_SEQUENTIAL;

    // in standard mode, try to set the number of OSG DatabasePager threads to use.
    if ( options.loadingPolicy().isSet() && !_isStreaming )
    {
        int numThreads = -1;

        if ( options.loadingPolicy()->numLoadingThreads().isSet() )
        {
            numThreads = osg::maximum( 1, *options.loadingPolicy()->numLoadingThreads() );
        }
        else if ( options.loadingPolicy()->numLoadingThreadsPerCore().isSet() )
        {
            float numThreadsPerCore = *options.loadingPolicy()->numLoadingThreadsPerCore();
            numThreads = osg::maximum( (int)1, (int)osg::round( 
                numThreadsPerCore * (float)OpenThreads::GetNumberOfProcessors() ) );
        }

        if ( numThreads > 0 )
        {
            // NOTE: this doesn't work. the pager gets created before we ever get here.
            numThreads = osg::maximum(numThreads, 2);
            int numHttpThreads = osg::clampBetween( numThreads/2, 1, numThreads-1 );

            //OE_INFO << LC << "Requesting pager threads in STANDARD mode: local=" << numThreads << ", http=" << numHttpThreads << std::endl;
            osg::DisplaySettings::instance()->setNumOfDatabaseThreadsHint( numThreads );
            osg::DisplaySettings::instance()->setNumOfHttpDatabaseThreadsHint( numHttpThreads );
        }
    }
}

void
OSGTerrainEngineNode::postInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::postInitialize( map, options );

    // Initialize the map frames. We need one for the update thread and one for the
    // cull thread. Someday we can detect whether these are actually the same thread
    // (depends on the viewer's threading mode).
    _update_mapf = new MapFrame( map, Map::MASKED_TERRAIN_LAYERS, "osgterrain-update" );
    _cull_mapf   = new MapFrame( map, Map::TERRAIN_LAYERS, "osgterrain-cull" );

    // merge in the custom options:
    _terrainOptions.merge( options );

    // handle an already-established map profile:
    if ( _update_mapf->getProfile() )
    {
        // NOTE: this will initialize the map with the startup layers
        onMapInfoEstablished( MapInfo(map) );
    }

    // populate the terrain with whatever data is in the map to begin with:
    if ( _terrain )
    {
        // update the terrain revision in threaded mode
        if ( _isStreaming )
        {
            static_cast<StreamingTerrainNode*>(_terrain)->updateTaskServiceThreads( *_update_mapf );
        }

        updateTextureCombining();
    }

    // install a layer callback for processing further map actions:
    map->addMapCallback( new OSGTerrainEngineNodeMapCallbackProxy(this) );

    //Attach to all of the existing elevation layers
    ElevationLayerVector elevationLayers;
    map->getElevationLayers( elevationLayers );
    for( ElevationLayerVector::const_iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i )
    {
        i->get()->addCallback( _elevationCallback.get() );
    }

    //Attach a callback to all of the 

    // register me.
    registerEngine( this );

    // now that we have a map, set up to recompute the bounds
    dirtyBound();
}

osg::BoundingSphere
OSGTerrainEngineNode::computeBound() const
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
OSGTerrainEngineNode::refresh()
{
    {
        removeChild( _terrain );
    }    


    _terrain = new TerrainNode(*_update_mapf, *_cull_mapf, _tileFactory.get(), *_terrainOptions.quickReleaseGLObjects() );    
    installTerrainTechnique();

   
    const MapInfo& mapInfo = _update_mapf->getMapInfo();
    _keyNodeFactory = new SerialKeyNodeFactory( _tileBuilder.get(), _terrainOptions, mapInfo, _terrain, _uid );

    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    std::vector< TileKey > keys;
    _update_mapf->getProfile()->getRootKeys( keys );

    if (_terrainOptions.enableBlending().value())
    {
        _terrain->getOrCreateStateSet()->setMode(GL_BLEND , osg::StateAttribute::ON);    
    }

    addChild( _terrain );

    for( unsigned i=0; i<keys.size(); ++i )
    {
        osg::Node* node;
        if ( _keyNodeFactory.valid() )
            node = _keyNodeFactory->createRootNode( keys[i] );
        else
            node = _tileFactory->createSubTiles( *_update_mapf, _terrain, keys[i], true );

        if ( node )
            _terrain->addChild( node );
        else
            OE_WARN << LC << "Couldn't make tile for root key: " << keys[i].str() << std::endl;
    }

    updateTextureCombining();
}

void
OSGTerrainEngineNode::onMapInfoEstablished( const MapInfo& mapInfo )
{
    LoadingPolicy::Mode mode = *_terrainOptions.loadingPolicy()->mode();
    OE_INFO << LC << "Loading policy mode = " <<
        ( mode == LoadingPolicy::MODE_PREEMPTIVE ? "PREEMPTIVE" :
          mode == LoadingPolicy::MODE_SEQUENTIAL ? "SEQUENTIAL" :
          mode == LoadingPolicy::MODE_PARALLEL   ? "PARALLEL" :
          "SERIAL/STANDARD" )
        << std::endl;

    // create a factory for creating actual tile data
    _tileFactory = new OSGTileFactory( _uid, *_cull_mapf, _terrainOptions );

    // go through and build the root nodesets.
    if ( !_isStreaming )
    {
        _terrain = new TerrainNode(
            *_update_mapf, *_cull_mapf, _tileFactory.get(), *_terrainOptions.quickReleaseGLObjects() );
    }
    else
    {
        _terrain = new StreamingTerrainNode(
            *_update_mapf, *_cull_mapf, _tileFactory.get(), *_terrainOptions.quickReleaseGLObjects() );
    }

    this->addChild( _terrain );

    // set the initial properties from the options structure:
    _terrain->setVerticalScale( _terrainOptions.verticalScale().value() );
    _terrain->setSampleRatio  ( _terrainOptions.heightFieldSampleRatio().value() );

    if (_terrainOptions.enableBlending().value())
    {
        _terrain->getOrCreateStateSet()->setMode(GL_BLEND , osg::StateAttribute::ON);    
    }

    OE_INFO << LC << "Sample ratio = " << _terrainOptions.heightFieldSampleRatio().value() << std::endl;

    // install the proper layer composition technique:

    installTerrainTechnique();    

    // install the shader program, if applicable:
    installShaders();

    // calculate a good thread pool size for non-streaming parallel processing
    if ( !_isStreaming )
    {
        unsigned num = 2 * OpenThreads::GetNumberOfProcessors();
        if ( _terrainOptions.loadingPolicy().isSet() )
        {
            if ( _terrainOptions.loadingPolicy()->numLoadingThreads().isSet() )
            {
                num = *_terrainOptions.loadingPolicy()->numLoadingThreads();
            }
            else if ( _terrainOptions.loadingPolicy()->numLoadingThreadsPerCore().isSet() )
            {
                num = (unsigned)(*_terrainOptions.loadingPolicy()->numLoadingThreadsPerCore() * OpenThreads::GetNumberOfProcessors());
            }
        }

        if ( mode == LoadingPolicy::MODE_PARALLEL )
        {
            _tileService = new TaskService( "TileBuilder", num );
        }

        // initialize the tile builder
        _tileBuilder = new TileBuilder( getMap(), _terrainOptions, _tileService.get() );


        // initialize a key node factory.
        switch( mode )
        {
        case LoadingPolicy::MODE_SERIAL:
            _keyNodeFactory = new SerialKeyNodeFactory( _tileBuilder.get(), _terrainOptions, mapInfo, _terrain, _uid );
            break;

        case LoadingPolicy::MODE_PARALLEL:
            _keyNodeFactory = new ParallelKeyNodeFactory( _tileBuilder.get(), _terrainOptions, mapInfo, _terrain, _uid );
            break;

        default:
            break;
        }
    }

    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    std::vector< TileKey > keys;
    _update_mapf->getProfile()->getRootKeys( keys );

    for( unsigned i=0; i<keys.size(); ++i )
    {
        osg::Node* node;
        if ( _keyNodeFactory.valid() )
            node = _keyNodeFactory->createRootNode( keys[i] );
        else
            node = _tileFactory->createSubTiles( *_update_mapf, _terrain, keys[i], true );

        if ( node )
            _terrain->addChild( node );
        else
            OE_WARN << LC << "Couldn't make tile for root key: " << keys[i].str() << std::endl;
    }

    // we just added the root tiles, so mark the bound in need of recomputation.
    dirtyBound();
}

osg::Node*
OSGTerrainEngineNode::createNode( const TileKey& key )
{
    // if the engine has been disconnected from the scene graph, bail out and don't
    // create any more tiles
    if ( getNumParents() == 0 )
        return 0L;

    OE_DEBUG << LC << "Create node for \"" << key.str() << "\"" << std::endl;

#ifdef PROFILING
    osg::Timer_t start = _timer.tick();
#endif

    osg::Node* result = 0L;

    osg::ref_ptr< TerrainNode > terrain = _terrain;

    osg::ref_ptr< KeyNodeFactory > keyNodeFactory = _keyNodeFactory;

    if ( _isStreaming )
    {
        // sequential or preemptive mode only.
        // create a map frame so we can safely create tiles from this dbpager thread
        MapFrame mapf( getMap(), Map::TERRAIN_LAYERS, "dbpager::earth plugin" );
        result = getTileFactory()->createSubTiles( mapf, terrain.get(), key, false );
    }
    else
    {
        if (keyNodeFactory.valid() && terrain.valid())
        {
            result = keyNodeFactory->createNode( key );
        }
    }

#ifdef PROFILING
    osg::Timer_t end = osg::Timer::instance()->tick();
    if ( result )
    {
        _tileCount++;
        _tileCreationTime += _timer.delta_s(start,_timer.tick());
        if ( _tileCount % 60 == 0 )
        {
            OE_INFO << LC << "Avg tile = " << 1000.0*(_tileCreationTime/(double)_tileCount)
                << " ms, tiles per sec = " << (double)_tileCount/_timer.time_s() << std::endl;
        }
    }
#endif

    return result;
}

osg::Node*
OSGTerrainEngineNode::createTile( const TileKey& key )
{
    if ( !_tileBuilder.valid() )
        return 0L;

    osg::ref_ptr<Tile> tile;
    bool hasRealData, hasLodBlendedLayers;

    _tileBuilder->createTile(
        key,
        false,
        tile,
        hasRealData,
        hasLodBlendedLayers );

    if ( !tile.valid() )
        return 0L;

    // code block required in order to properly manage the ref count of the transform
    SinglePassTerrainTechnique* tech = new SinglePassTerrainTechnique( _texCompositor.get() );
    // prepare the interpolation technique for generating triangles:
    if ( getMap()->getMapOptions().elevationInterpolation() == INTERP_TRIANGULATE )
        tech->setOptimizeTriangleOrientation( false ); 

    tile->setTerrainTechnique( tech );
    tile->init();
    
    return tech->takeTransform();
}

void
OSGTerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
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
        default: break;
        }
    }

    // update the terrain revision in threaded mode
    if ( _isStreaming )
    {
        //getTerrain()->incrementRevision();
        static_cast<StreamingTerrainNode*>(_terrain)->updateTaskServiceThreads( *_update_mapf );
    }
}

void
OSGTerrainEngineNode::addImageLayer( ImageLayer* layerAdded )
{
    if ( !layerAdded )
        return;

    if (!_isStreaming)
    {
        refresh();
    }
    else
    {
        // visit all existing terrain tiles and inform each one of the new image layer:
        TileVector tiles;
        _terrain->getTiles( tiles );

        for( TileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr )
        {
            Tile* tile = itr->get();

            StreamingTile* streamingTile = 0L;

            GeoImage geoImage;
            bool needToUpdateImagery = false;
            int imageLOD = -1;

            if ( !_isStreaming || tile->getKey().getLevelOfDetail() == 1 )
            {
                // in standard mode, or at the first LOD in seq/pre mode, fetch the image immediately.
                TileKey geoImageKey = tile->getKey();
                _tileFactory->createValidGeoImage( layerAdded, tile->getKey(), geoImage, geoImageKey );
                imageLOD = tile->getKey().getLevelOfDetail();
            }
            else
            {
                // in seq/pre mode, set up a placeholder and mark the tile as dirty.
                geoImage = GeoImage(ImageUtils::createEmptyImage(), tile->getKey().getExtent() );
                needToUpdateImagery = true;
                streamingTile = static_cast<StreamingTile*>(tile);
            }

            if (geoImage.valid())
            {
                const MapInfo& mapInfo = _update_mapf->getMapInfo();

                double img_min_lon, img_min_lat, img_max_lon, img_max_lat;
                geoImage.getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);

                //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
                osg::ref_ptr<GeoLocator> img_locator = tile->getKey().getProfile()->getSRS()->createLocator( 
                    img_min_lon, img_min_lat, img_max_lon, img_max_lat, 
                    !mapInfo.isGeocentric() );

                //Set the CS to geocentric if we are dealing with a geocentric map
                if ( mapInfo.isGeocentric() )
                {
                    img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
                }

                tile->setCustomColorLayer( CustomColorLayer(
                    layerAdded,
                    geoImage.getImage(),
                    img_locator.get(), imageLOD,  tile->getKey() ) );

                // if necessary, tell the tile to queue up a new imagery request (since we
                // just installed a placeholder)
                if ( needToUpdateImagery )
                {
                    streamingTile->updateImagery( layerAdded, *_update_mapf, _tileFactory.get() );
                }
            }
            else
            {
                // this can happen if there's no data in the new layer for the given tile.
                // we will rely on the driver to dump out a warning if this is an error.
            }

            tile->applyImmediateTileUpdate( TileUpdate::ADD_IMAGE_LAYER, layerAdded->getUID() );
        }

        updateTextureCombining();
    }
}

void
OSGTerrainEngineNode::removeImageLayer( ImageLayer* layerRemoved )
{
    if (!_isStreaming)
    {
        refresh();
    }
    else
    {
        // make a thread-safe copy of the tile table
        TileVector tiles;
        _terrain->getTiles( tiles );

        for (TileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            Tile* tile = itr->get();

            // critical section
            tile->removeCustomColorLayer( layerRemoved->getUID() );
        }

        updateTextureCombining();
    }
}

void
OSGTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
    // take a thread-safe copy of the tile table
    TileVector tiles;
    _terrain->getTiles( tiles );

    for (TileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        Tile* tile = itr->get();
        tile->applyImmediateTileUpdate( TileUpdate::MOVE_IMAGE_LAYER );
    }     

    updateTextureCombining();
}

void
OSGTerrainEngineNode::updateElevation( Tile* tile )
{
    Threading::ScopedWriteLock exclusiveLock( tile->getTileLayersMutex() );

    const TileKey& key = tile->getKey();

    bool hasElevation = _update_mapf->elevationLayers().size() > 0;

    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    if (heightFieldLayer)
    {
        // In standard mode, just load the elevation data and dirty the tile.
        if ( !_isStreaming )
        {
            osg::ref_ptr<osg::HeightField> hf;

            if (hasElevation)
                _update_mapf->getHeightField( key, true, hf, 0L);

            if (!hf.valid()) 
                hf = OSGTileFactory::createEmptyHeightField( key );

            heightFieldLayer->setHeightField( hf.get() );
            hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );

            //TODO: review this in favor of a tile update...
            tile->setDirty( true );
        }

        else // if ( isStreaming )
        {
            StreamingTile* stile = static_cast<StreamingTile*>(tile);

            //Update the elevation hint
            stile->setHasElevationHint( hasElevation );

            //In seq/pre mode, if there is no elevation, just clear out all the elevation on the tiles
            if ( !hasElevation )
            {
                osg::ref_ptr<osg::HeightField> hf = OSGTileFactory::createEmptyHeightField( key );
                heightFieldLayer->setHeightField( hf.get() );
                hf->setSkirtHeight( stile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
                stile->setElevationLOD( key.getLevelOfDetail() );
                stile->resetElevationRequests( *_update_mapf );
                stile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
            }
            else
            {
                //Always load the first LOD so the children tiles can have something to use for placeholders
                if (stile->getKey().getLevelOfDetail() == 1)
                {
                    osg::ref_ptr<osg::HeightField> hf;
                    _update_mapf->getHeightField( key, true, hf, 0L);
                    if (!hf.valid()) 
                        hf = OSGTileFactory::createEmptyHeightField( key );
                    heightFieldLayer->setHeightField( hf.get() );
                    hf->setSkirtHeight( stile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
                    stile->setElevationLOD(tile->getKey().getLevelOfDetail());
                    stile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                }
                else
                {
                    //Set the elevation LOD to -1
                    stile->setElevationLOD(-1);
                    stile->resetElevationRequests( *_update_mapf );
                }
            }
        }
    }
}


void
OSGTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    if ( !layer )
        return;

    layer->addCallback( _elevationCallback.get() );


    if (!_isStreaming)
    {
        refresh();
    }
    else
    {    
        TileVector tiles;
        _terrain->getTiles( tiles );

        OE_DEBUG << LC << "Found " << tiles.size() << std::endl;

        for (TileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            updateElevation( itr->get() );
        }
    }
}

void
OSGTerrainEngineNode::removeElevationLayer( ElevationLayer* layerRemoved )
{
    layerRemoved->removeCallback( _elevationCallback.get() );

    if (!_isStreaming)
    {
        refresh();
    }
    else
    {
        TileVector tiles;
        _terrain->getTiles( tiles );

        for (TileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            updateElevation( itr->get() );
        }
    }
}

void
OSGTerrainEngineNode::moveElevationLayer( unsigned int oldIndex, unsigned int newIndex )
{
    if (!_isStreaming)
    {
        refresh();
    }
    else
    {
        TileVector tiles;
        _terrain->getTiles( tiles );

        OE_DEBUG << "Found " << tiles.size() << std::endl;

        for (TileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            updateElevation( itr->get() );
        }
    }
}

void
OSGTerrainEngineNode::validateTerrainOptions( TerrainOptions& options )
{
    TerrainEngineNode::validateTerrainOptions( options );
    
    //nop for now.
    //note: to validate plugin-specific features, we would create an OSGTerrainOptions
    // and do the validation on that. You would then re-integrate it by calling
    // options.mergeConfig( osgTerrainOptions ).
}

void
OSGTerrainEngineNode::traverse( osg::NodeVisitor& nv )
{
    if ( _cull_mapf ) // ensures initialize() has been called
    {
        if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            // update the cull-thread map frame if necessary. (We don't need to sync the
            // update_mapf because that happens in response to a map callback.)

            // TODO: address the fact that this can happen from multiple threads.
            // Really we need a _cull_mapf PER view. -gw
            _cull_mapf->sync();
        }
    }

    TerrainEngineNode::traverse( nv );
}

void
OSGTerrainEngineNode::installShaders()
{
    // This method installs a default shader setup on the engine node itself. The texture compositor
    // can then override parts of the program by using a VirtualProgram on the _terrain node. We do
    // it this way so that the developer has the option of removing this top-level shader program,
    // replacing it, or migrating it higher up the scene graph if necessary.

    if ( _texCompositor.valid() && _texCompositor->usesShaderComposition() )
    {
        const ShaderFactory* sf = Registry::instance()->getShaderFactory();

        int numLayers = osg::maximum( 1, (int)_update_mapf->imageLayers().size() );
        //int numLayers = osg::maximum( 0, (int)_update_mapf->imageLayers().size() );

        VirtualProgram* vp = new VirtualProgram();
        vp->setName( "engine_osgterrain:EngineNode" );
        //vp->installDefaultColoringAndLightingShaders(numLayers);

        getOrCreateStateSet()->setAttributeAndModes( vp, osg::StateAttribute::ON );
    }
}

void
OSGTerrainEngineNode::updateTextureCombining()
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

            VirtualProgram* vp = new VirtualProgram() ;
            vp->setName( "engine_osgterrain:TerrainNode" );
            //vp->installDefaultColoringShaders(numImageLayers);

            terrainStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );

            // first, update the default shader components based on the new layer count:
            const ShaderFactory* sf = Registry::instance()->getShaderFactory();
            
            // second, install the per-layer color filter functions.
            for( int i=0; i<numImageLayers; ++i )
            {
                std::string layerFilterFunc = Stringify() << "osgearth_runColorFilters_" << i;
                const ColorFilterChain& chain = _update_mapf->getImageLayerAt(i)->getColorFilters();

                // install the wrapper function that calls all the filters in turn:
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
        _texCompositor->updateMasterStateSet( terrainStateSet ); //, numImageLayers );
    }
}

namespace
{
    class UpdateElevationVisitor : public osg::NodeVisitor
    {
    public:
        UpdateElevationVisitor():
          osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
          {}

          void apply(osg::Node& node)
          {
              Tile* tile = dynamic_cast<Tile*>(&node);
              if (tile)
              {
                  tile->applyImmediateTileUpdate(TileUpdate::UPDATE_ELEVATION);
              }

              traverse(node);
          }
    };
}

void
OSGTerrainEngineNode::onVerticalScaleChanged()
{
    _terrain->setVerticalScale(getVerticalScale());

    UpdateElevationVisitor visitor;
    this->accept(visitor);
}

void
OSGTerrainEngineNode::installTerrainTechnique()
{
    if ( _texCompositor->getTechnique() == TerrainOptions::COMPOSITING_MULTIPASS )
    {
        //If we are using multipass mode, disable GLSL on it, it is using straight FFP
        _terrain->getOrCreateStateSet()->setAttributeAndModes( new osg::Program(), osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
        _terrain->setTechniquePrototype( new MultiPassTerrainTechnique( _texCompositor.get() ) );
        OE_INFO << LC << "Compositing technique = MULTIPASS" << std::endl;
    }

    else 
    {
        SinglePassTerrainTechnique* tech = new SinglePassTerrainTechnique( _texCompositor.get() );
        tech->setClearDataAfterCompile( !_isStreaming );
        

        if ( getMap()->getMapOptions().elevationInterpolation() == INTERP_TRIANGULATE )
            tech->setOptimizeTriangleOrientation( false );   
        
        _terrain->setTechniquePrototype( tech );
    }
}
