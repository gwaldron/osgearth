/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include "SinglePassTerrainTechnique"
#include "CustomTerrain"
#include "MultiPassTerrainTechnique"
#include "TransparentLayer"

#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>

#define LC "[OSGTerrainEngine] "

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
static OpenThreads::ReentrantMutex s_engineNodeCacheMutex;
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
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_engineNodeCacheMutex);
    getEngineNodeCache()[engineNode->_uid] = engineNode;
    OE_DEBUG << LC << "Registered engine " << engineNode->_uid << std::endl;
}

void
OSGTerrainEngineNode::unregisterEngine( UID uid )
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_engineNodeCacheMutex);
    EngineNodeCache::iterator k = getEngineNodeCache().find( uid );
    if (k != getEngineNodeCache().end())
    {
        getEngineNodeCache().erase(k);
        OE_DEBUG << LC << "Unregistered engine " << uid << std::endl;
    }
}

OSGTerrainEngineNode*
OSGTerrainEngineNode::getEngineByUID( UID uid )
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_engineNodeCacheMutex);
    EngineNodeCache::const_iterator k = getEngineNodeCache().find( uid );
    if (k != getEngineNodeCache().end()) return k->second.get();
    return 0;
}

UID
OSGTerrainEngineNode::getUID() const
{
    return _uid;
}

//------------------------------------------------------------------------

OSGTerrainEngineNode::OSGTerrainEngineNode() :
TerrainEngineNode(),
_terrain( 0L ),
_update_mapf( 0L ),
_cull_mapf( 0L )
{
    _uid = Registry::instance()->createUID();
    _taskServiceMgr = Registry::instance()->getTaskServiceManager();
}

OSGTerrainEngineNode::OSGTerrainEngineNode( const OSGTerrainEngineNode& rhs, const osg::CopyOp& op ) :
TerrainEngineNode( rhs, op )
{
    //nop - this copy ctor will never get called since this is a plugin instance.
    OE_WARN << LC << "ILLEGAL STATE in OSGTerrainEngineNode Copy CTOR" << std::endl;
}

OSGTerrainEngineNode::~OSGTerrainEngineNode()
{
    unregisterEngine( _uid );

    if ( _update_mapf )
        delete _update_mapf;

    if ( _cull_mapf )
        delete _cull_mapf;
}

void
OSGTerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::preInitialize( map, options );

    // in standard mode, try to set the number of OSG DatabasePager threads to use.
    if (options.loadingPolicy().isSet() &&
        options.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
    {
        int numThreads = -1;

        if ( options.loadingPolicy()->numLoadingThreads().isSet() )
        {
            numThreads = osg::maximum( 1, *options.loadingPolicy()->numLoadingThreads() );
        }
        else if ( options.loadingPolicy()->numLoadingThreadsPerCore().isSet() )
        {
            int numThreadsPerCore = *options.loadingPolicy()->numLoadingThreadsPerCore();
            numThreads = osg::maximum( (int)1, (int)osg::round( 
                numThreadsPerCore * (float)OpenThreads::GetNumberOfProcessors() ) );
        }

        if ( numThreads > 0 )
        {
            OE_INFO << LC << "Requesting " << numThreads << " database pager threads in STANDARD mode" << std::endl;
            osg::DisplaySettings::instance()->setNumOfDatabaseThreadsHint( numThreads );
            //osg::DisplaySettings::instance()->setNumOfHttpDatabaseThreadsHint( numThreads );
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
    _update_mapf = new MapFrame( map, Map::TERRAIN_LAYERS, "osgterrain-update" );
    _cull_mapf   = new MapFrame( map, Map::TERRAIN_LAYERS, "osgterrain-cull" );

    // merge in the custom options:
    _terrainOptions.merge( options );

    // handle an already-established map profile:
    if ( _update_mapf->getProfile() )
    {
        onMapInfoEstablished( MapInfo(map) );
        //onMapProfileEstablished( _update_mapf->getProfile() );
    }

    // populate the terrain with whatever data is in the map to begin with:
    if ( _terrain )
    {
        _update_mapf->sync();

        unsigned int index = 0;
        for( ElevationLayerVector::const_iterator i = _update_mapf->elevationLayers().begin(); i != _update_mapf->elevationLayers().end(); i++ )
        {
            addElevationLayer( i->get() );
        }

        index = 0;
        for( ImageLayerVector::const_iterator j = _update_mapf->imageLayers().begin(); j != _update_mapf->imageLayers().end(); j++ )
        {
            addImageLayer( j->get() );
        }

        // update the terrain revision in threaded mode
        if ( _terrainOptions.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
        {
            _terrain->updateTaskServiceThreads( *_update_mapf );
        }
    }

    // install a layer callback for processing further map actions:
    map->addMapCallback( new OSGTerrainEngineNodeMapCallbackProxy(this) );

    // register me.
    registerEngine( this );

    // now that we have a map, set up to recompute the bounds
    dirtyBound();
}

osg::BoundingSphere
OSGTerrainEngineNode::computeBound() const
{
    if ( _terrain )
        return _terrain->getBound();
    else
        return TerrainEngineNode::computeBound();
}

void
OSGTerrainEngineNode::onMapInfoEstablished( const MapInfo& mapInfo )
{
    OE_INFO << LC << "Map profile established" << std::endl;

    // create a factory for creating actual tile data
    _tileFactory = new OSGTileFactory( _uid, *_cull_mapf, _terrainOptions );

    // go through and build the root nodesets.
    _terrain = new CustomTerrain(
        *_update_mapf, *_cull_mapf, _tileFactory.get(), *_terrainOptions.quickReleaseGLObjects() );

    this->addChild( _terrain );

    // set the initial properties from the options structure:
    _terrain->setVerticalScale( _terrainOptions.verticalScale().value() );
    _terrain->setSampleRatio( _terrainOptions.heightFieldSampleRatio().value() );

    OE_INFO << LC << "Sample ratio = " << _terrainOptions.heightFieldSampleRatio().value() << std::endl;

    //// install the proper layer composition technique:
    //_texCompositor = new TextureCompositor( _terrainOptions.compositingTechnique().value() );

    if ( _texCompositor->getTechnique() == TerrainOptions::COMPOSITING_MULTIPASS )
    {
        _terrain->setTerrainTechniquePrototype( new MultiPassTerrainTechnique( _texCompositor.get() ) );
        OE_INFO << LC << "Compositing technique = MULTIPASS" << std::endl;
    }

    else 
    {
        CustomTerrainTechnique* tech = new SinglePassTerrainTechnique( _texCompositor.get() );

        // prepare the interpolation technique for generating triangles:
        if ( _terrainOptions.elevationInterpolation() == INTERP_TRIANGULATE )
            tech->setOptimizeTriangleOrientation( false );

        _terrain->setTerrainTechniquePrototype( tech );
    }
    
    // prime the texture compositor with any existing layers:
    for( unsigned int i=0; i<_update_mapf->imageLayers().size(); ++i )
    {
        _texCompositor->applyMapModelChange( MapModelChange(
            MapModelChange::ADD_IMAGE_LAYER,
            _update_mapf->getRevision(),
            _update_mapf->getImageLayerAt(i),
            i ) );
    }

    // install the shader program, if applicable:
    installShaders();

    // apply any pending callbacks:
#if 0
    for( TerrainCallbackList::iterator c = _pendingTerrainCallbacks.begin(); c != _pendingTerrainCallbacks.end(); ++c )
    {
        terrain->addTerrainCallback( c->get() );
    }
    _pendingTerrainCallbacks.clear();
#endif

    // collect the tile keys comprising the root tiles of the terrain.
    std::vector< TileKey > keys;
    _update_mapf->getProfile()->getRootKeys( keys );

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        // always load the root tiles completely; no deferring. -gw
        bool loadNow = true; //!_terrainOptions.getPreemptiveLOD();

        osg::Node* node = _tileFactory->createSubTiles( *_update_mapf, _terrain, keys[i], loadNow );
        if (node)
        {
            _terrain->addChild(node);
        }
        else
        {
            OE_WARN << LC << "Couldn't make tile for root key: " << keys[i].str() << std::endl;
        }
    }

    // we just added the root tiles, so mark the bound in need of recomputation.
    dirtyBound();
}

void
OSGTerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
    _update_mapf->sync();

    // dispatch the change handler
    if ( change.getLayer() )
    {
        // first inform the texture compositor with the new model changes:
        if ( _texCompositor.valid() )
            _texCompositor->applyMapModelChange( change );

        // then apply the actual change:
        switch( change.getAction() )
        {
        case MapModelChange::ADD_IMAGE_LAYER:
            addImageLayer( change.getImageLayer() );
            break;
        case MapModelChange::REMOVE_IMAGE_LAYER:
            removeImageLayer( change.getImageLayer(), change.getFirstIndex() );
            break;
        case MapModelChange::ADD_ELEVATION_LAYER:
            addElevationLayer( change.getElevationLayer() );
            break;
        case MapModelChange::REMOVE_ELEVATION_LAYER:
            removeElevationLayer( change.getElevationLayer(), change.getFirstIndex() );
            break;
        case MapModelChange::MOVE_IMAGE_LAYER:
            moveImageLayer( change.getFirstIndex(), change.getSecondIndex() );
            break;
        case MapModelChange::MOVE_ELEVATION_LAYER:
            moveElevationLayer( change.getFirstIndex(), change.getSecondIndex() );
            break;
		default:
			break;
        }
    }

    // update the terrain revision in threaded mode
    if ( _terrainOptions.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
    {
        getTerrain()->incrementRevision();
        getTerrain()->updateTaskServiceThreads( *_update_mapf );
    }
}

void
OSGTerrainEngineNode::addImageLayer( ImageLayer* layerAdded )
{
    if ( !layerAdded || !layerAdded->getTileSource() )
        return;

    // visit all existing terrain tiles and inform each one of the new image layer:
    CustomTileVector tiles;
    _terrain->getCustomTiles( tiles );

    for( CustomTileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr )
    {
        CustomTile* tile = itr->get();

        GeoImage geoImage;
        bool needToUpdateImagery = false;
        int imageLOD = -1;

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD ||
            tile->getKey().getLevelOfDetail() == 1)
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
                img_locator.get(), imageLOD ) );

            // if necessary, tell the tile to queue up a new imagery request (since we
            // just installed a placeholder)
            if ( needToUpdateImagery )
                tile->updateImagery( layerAdded, *_update_mapf, _tileFactory.get() );
        }
        else
        {
            // this can happen if there's no data in the new layer for the given tile.
            // we will rely on the driver to dump out a warning if this is an error.
        }

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
        {
            tile->applyImmediateTileUpdate( TileUpdate::ADD_IMAGE_LAYER, layerAdded->getUID() );
        }
        else
        {
            tile->applyImmediateTileUpdate( TileUpdate::ADD_IMAGE_LAYER, layerAdded->getUID() );
        }
    }

    updateTextureCombining();
}

void
OSGTerrainEngineNode::removeImageLayer( ImageLayer* layerRemoved, unsigned int index )
{
    // make a thread-safe copy of the tile table
    CustomTileVector tiles;
    _terrain->getCustomTiles( tiles );

    for (CustomTileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = itr->get();

        // critical section
        tile->removeCustomColorLayer( index );

        //if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
        //    tile->applyImmediateTileUpdate( TileUpdate::REMOVE_IMAGE_LAYER, layerRemoved->getUID() );
        //else
        //    tile->applyImmediateTileUpdate( TileUpdate::REMOVE_IMAGE_LAYER, layerRemoved->getUID() );
    }
    
    updateTextureCombining();

    OE_DEBUG << "[osgEarth::Map::removeImageSource] end " << std::endl;  
}

void
OSGTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
    // take a thread-safe copy of the tile table
    CustomTileVector tiles;
    _terrain->getCustomTiles( tiles );

    for (CustomTileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = itr->get();

        //tile->moveColorLayer( oldIndex, newIndex );

        tile->applyImmediateTileUpdate( TileUpdate::MOVE_IMAGE_LAYER );
    }     

    updateTextureCombining();
}

void
OSGTerrainEngineNode::updateElevation(CustomTile* tile)
{
    Threading::ScopedWriteLock tileLock( tile->getTileLayersMutex() );

    const TileKey& key = tile->getKey();

    bool hasElevation;
    {
        hasElevation = _update_mapf->elevationLayers().size() > 0;
    }    

    //Update the elevation hint
    tile->setHasElevationHint( hasElevation );

    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    if (heightFieldLayer)
    {
        //In standard mode, just load the elevation data and dirty the tile.

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
        {
            osg::ref_ptr<osg::HeightField> hf;

            if (hasElevation)
                _update_mapf->getHeightField( key, true, hf, _terrainOptions.elevationInterpolation().value());

            if (!hf.valid()) 
                hf = OSGTileFactory::createEmptyHeightField( key );

            heightFieldLayer->setHeightField( hf.get() );
            hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );

            //TODO: review this in favor of a tile update...
            tile->setDirty(true);
        }
        else
        {
            //In preemptive mode, if there is no elevation, just clear out all the elevation on the tiles
            if (!hasElevation)
            {
                osg::ref_ptr<osg::HeightField> hf = OSGTileFactory::createEmptyHeightField( key );
                heightFieldLayer->setHeightField( hf.get() );
                hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
                tile->setElevationLOD( key.getLevelOfDetail() );
                tile->resetElevationRequests( *_update_mapf );
                tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
            }
            else
            {
                //Always load the first LOD so the children tiles can have something to use for placeholders
                if (tile->getKey().getLevelOfDetail() == 1)
                {
                    osg::ref_ptr<osg::HeightField> hf;
                    _update_mapf->getHeightField( key, true, hf, _terrainOptions.elevationInterpolation().value());
                    if (!hf.valid()) 
                        hf = OSGTileFactory::createEmptyHeightField( key );
                    heightFieldLayer->setHeightField( hf.get() );
                    hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
                    tile->setElevationLOD(tile->getKey().getLevelOfDetail());
                    tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                }
                else
                {
                    //Set the elevation LOD to -1
                    tile->setElevationLOD(-1);
                    tile->resetElevationRequests( *_update_mapf );
                }
            }
        }
    }
}


void
OSGTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    if ( !layer || !layer->getTileSource() )
        return;
    
    CustomTileVector tiles;
    _terrain->getCustomTiles( tiles );

    OE_DEBUG << LC << "Found " << tiles.size() << std::endl;

    for (CustomTileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        updateElevation( itr->get() );
    }
}

void
OSGTerrainEngineNode::removeElevationLayer( ElevationLayer* layerRemoved, unsigned int index )
{
    CustomTileVector tiles;
    _terrain->getCustomTiles( tiles );

    for (CustomTileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        updateElevation( itr->get() );
    }
}

void
OSGTerrainEngineNode::moveElevationLayer( unsigned int oldIndex, unsigned int newIndex )
{
    CustomTileVector tiles;
    _terrain->getCustomTiles( tiles );

    OE_DEBUG << "Found " << tiles.size() << std::endl;

    for (CustomTileVector::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        updateElevation( itr->get() );
    }
}

void
OSGTerrainEngineNode::validateTerrainOptions( TerrainOptions& options )
{
    TerrainEngineNode::validateTerrainOptions( options );

    // LOD blending is currently only compatible with STANDARD loading policy
    if ( options.lodBlending() == true && options.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
    {
        //options.lodBlending() = false;
    }
    
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
            // update_mapf becuase that happens in response to a map callback.)
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

        VirtualProgram* vp = new VirtualProgram();

        // note. this stuff should probably happen automatically in VirtualProgram. gw

        //vp->setShader( "osgearth_vert_main",     sf->createVertexShaderMain() ); // happens in VirtualProgram now
        vp->setShader( "osgearth_vert_setupLighting", sf->createDefaultLightingVertexShader() );
        vp->setShader( "osgearth_vert_setupTexturing",  sf->createDefaultTextureVertexShader( numLayers ) );

        //vp->setShader( "osgearth_frag_main",     sf->createFragmentShaderMain() ); // happend in VirtualProgram now
        vp->setShader( "osgearth_frag_applyLighting", sf->createDefaultLightingFragmentShader() );
        vp->setShader( "osgearth_frag_applyTexturing",  sf->createDefaultTextureFragmentShader( numLayers ) );

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

            VirtualProgram* vp = dynamic_cast<VirtualProgram*>( terrainStateSet->getAttribute(osg::StateAttribute::PROGRAM) );
            if ( !vp )
            {
                // create and add it the first time around..
                vp = new VirtualProgram();
                terrainStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );
            }

            // first, update the default shader components based on the new layer count:
            const ShaderFactory* sf = Registry::instance()->getShaderFactory();
            vp->setShader( "osgearth_vert_setupTexturing",  sf->createDefaultTextureVertexShader( numImageLayers ) );

            // not this one, because the compositor always generates a new one.
            //vp->setShader( "osgearth_frag_applyTexturing",  lib.createDefaultTextureFragmentShader( numImageLayers ) );
        }

        // next, inform the compositor that it needs to update based on a new layer count:
        _texCompositor->updateMasterStateSet( terrainStateSet ); //, numImageLayers );
    }
}
