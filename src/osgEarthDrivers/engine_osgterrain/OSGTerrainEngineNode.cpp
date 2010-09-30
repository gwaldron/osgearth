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
#include "CompositingTerrainTechnique"
#include "CustomTerrain"
#include "MultiPassTerrainTechnique"
#include "TransparentLayer"

#include <osgEarth/ImageUtils>
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

    void onMapProfileEstablished( const Profile* profile ) {
        _node->onMapProfileEstablished(profile);
    }
    void onImageLayerAdded( ImageLayer* layer, unsigned int index ) {
        _node->onImageLayerAdded(layer, index);
    }
    void onImageLayerRemoved( ImageLayer* layer, unsigned int index ) {
        _node->onImageLayerRemoved(layer, index);
    }
    void onImageLayerMoved( ImageLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
        _node->onImageLayerMoved(layer,oldIndex,newIndex);
    }
    void onElevationLayerAdded( ElevationLayer* layer, unsigned int index ) {
        _node->onElevationLayerAdded(layer, index);
    }
    void onElevationLayerRemoved( ElevationLayer* layer, unsigned int index ) {
        _node->onElevationLayerRemoved(layer, index);
    }
    void onElevationLayerMoved( ElevationLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
        _node->onElevationLayerMoved(layer,oldIndex,newIndex);
    }
};

//---------------------------------------------------------------------------

//static
static OpenThreads::ReentrantMutex s_mapNodeCacheMutex;
static unsigned int s_mapNodeID = 0;
//Caches the MapNodes that have been created
typedef std::map<unsigned int, osg::observer_ptr<OSGTerrainEngineNode> > MapNodeCache;

static
MapNodeCache& getMapNodeCache()
{
    static MapNodeCache s_cache;
    return s_cache;
}

void
OSGTerrainEngineNode::registerEngine(OSGTerrainEngineNode* mapNode)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    getMapNodeCache()[mapNode->_id] = mapNode;
    OE_INFO << LC << "Registered engine " << mapNode->_id << std::endl;
}

void
OSGTerrainEngineNode::unregisterEngine(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    MapNodeCache::iterator k = getMapNodeCache().find( id);
    if (k != getMapNodeCache().end())
    {
        getMapNodeCache().erase(k);
        OE_INFO << LC << "Unregistered engine " << id << std::endl;
    }
}

OSGTerrainEngineNode*
OSGTerrainEngineNode::getEngineById(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    MapNodeCache::const_iterator k = getMapNodeCache().find( id);
    if (k != getMapNodeCache().end()) return k->second.get();
    return 0;
}

unsigned int
OSGTerrainEngineNode::getId() const
{
    return _id;
}

//------------------------------------------------------------------------

OSGTerrainEngineNode::OSGTerrainEngineNode( const OSGTerrainEngineNode& rhs, const osg::CopyOp& op ) :
TerrainEngineNode( rhs, op ),
_terrain( 0L )
{
    //nop
}

OSGTerrainEngineNode::~OSGTerrainEngineNode()
{
    unregisterEngine( _id );
}

void
OSGTerrainEngineNode::initialize( Map* map, const TerrainOptions& terrainOptions )
{
    TerrainEngineNode::initialize( map, terrainOptions );

    // merge in the custom options:
    _terrainOptions.merge( terrainOptions );

    // genearte a new unique mapnode ID
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock( s_mapNodeCacheMutex );
        _id = s_mapNodeID++;
    }
    //_map->setId( _id );

    // handle an already-established map profile:
    if ( _map->getProfile() )
    {
        onMapProfileEstablished( _map->getProfile() );
    }

    if ( _terrain )
    {
        // go through the map and process any already-installed layers:

        ElevationLayerVector elevLayers;
        _map->getElevationLayers( elevLayers );
        unsigned int index = 0;
        for( ElevationLayerVector::const_iterator i = elevLayers.begin(); i != elevLayers.end(); i++ )
        {
            onElevationLayerAdded( i->get(), index++ );
        }

        ImageLayerVector imageLayers;
        _map->getImageLayers( imageLayers );
        index = 0;
        for( ImageLayerVector::const_iterator j = imageLayers.begin(); j != imageLayers.end(); j++ )
        {
            onImageLayerAdded( j->get(), index++ );
        }
    }

    // install a layer callback for processing further map actions:
    _map->addMapCallback( new OSGTerrainEngineNodeMapCallbackProxy(this) );

    // register me.
    registerEngine( this );
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
OSGTerrainEngineNode::onMapProfileEstablished( const Profile* mapProfile )
{
    OE_INFO << LC << "Map profile established" << std::endl;

    // set up the ellipsoid
    this->setCoordinateSystem( mapProfile->getSRS()->getInitString() );
    this->setFormat( mapProfile->getSRS()->getInitType() );
    if ( !mapProfile->getSRS()->isProjected() )
        this->setEllipsoidModel( new osg::EllipsoidModel( *mapProfile->getSRS()->getEllipsoid() ) );

    // create a factory for creating actual tile data
    _tileFactory = new OSGTileFactory( _id, _terrainOptions );

    // go through and build the root nodesets.
    _terrain = new CustomTerrain( _map.get(), _tileFactory.get() );
    this->addChild( _terrain );

    // set the initial properties from the options structure:
    _terrain->setVerticalScale( _terrainOptions.verticalScale().value() );
    _terrain->setSampleRatio( _terrainOptions.heightFieldSampleRatio().value() );

    // install the proper layering technique:
    if ( _terrainOptions.layeringTechnique() == TerrainOptions::LAYERING_COMPOSITE )
    {
        _texCompositor = new TextureCompositor();
        CustomTerrainTechnique* tech = new CompositingTerrainTechnique( _texCompositor.get() );

        // prepare the interpolation technique for generating triangles:
        if ( _terrainOptions.elevationInterpolation() == INTERP_TRIANGULATE )
            tech->setOptimizeTriangleOrientation( false );

        _terrain->setTerrainTechniquePrototype( tech );

        OE_INFO << LC << "Layering technique = COMPOSITE" << std::endl;
    }
    else // MULTIPASS ... probably to be deprecated
    {
        _terrain->setTerrainTechniquePrototype( new MultiPassTerrainTechnique() );
        OE_INFO << LC << "Layering technique = MULTIPASS" << std::endl;
    }

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
    _map->getProfile()->getRootKeys( keys );

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        // always load the root tiles completely; no deferring. -gw
        bool loadNow = true; //!_terrainOptions.getPreemptiveLOD();

        osg::Node* node = _tileFactory->createSubTiles( _map.get(), _terrain, keys[i], loadNow );
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
OSGTerrainEngineNode::onTerrainLayerAdded( TerrainLayer* layer )
{
    if ( _terrainOptions.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
    {
        if ( layer->getTileSource() )
        {
            getTerrain()->incrementRevision();
            getTerrain()->updateTaskServiceThreads();
        }
    }
}

void
OSGTerrainEngineNode::onImageLayerAdded( ImageLayer* layer, unsigned int index )
{
    if ( layer )
    {
        onTerrainLayerAdded( layer );
        if ( layer->getTileSource() )
            addImageLayer( layer );
    }
}

void
OSGTerrainEngineNode::onElevationLayerAdded( ElevationLayer* layer, unsigned int index )
{
    if ( layer )
    {
        onTerrainLayerAdded( layer );
        if ( layer->getTileSource() )
            addElevationLayer( layer );
    }
}

void
OSGTerrainEngineNode::addImageLayer( ImageLayer* layer )
{
    //TODO: review the scope of this mapdata mutex lock within this method. We can 
    // probably optimize it some
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    // visit all existing terrain tiles and inform each one of the new image layer:
    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        Threading::ScopedWriteLock tileLock(tile->getTileLayersMutex());

        //Create a TileKey from the TileID
        osgTerrain::TileID tileId = tile->getTileID();
        TileKey key( TileKey::getLOD(tileId), tileId.x, tileId.y, _map->getProfile() );

        GeoImage geoImage;

        bool needToUpdateImagery = false;
        int imageLOD = -1;

        // establish the initial image for this tile.
        //if (( _options.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD ) ||
        //   (( _options.loadingPolicy()->mode() == LoadingPolicy::MODE_SEQUENTIAL) && key.getLevelOfDetail() == 1))

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD ||
            key.getLevelOfDetail() == 1)
        {
            // in standard mode, or at the first LOD in seq/pre mode, fetch the image immediately.
            geoImage = _tileFactory->createValidGeoImage( layer, key );
            imageLOD = key.getLevelOfDetail();
        }
        else
        {
            // in seq/pre mode, set up a placeholder and mark the tile as dirty.
            geoImage = GeoImage(ImageUtils::createEmptyImage(), key.getExtent() );
            needToUpdateImagery = true;
        }

        if (geoImage.valid())
        {
            double img_min_lon, img_min_lat, img_max_lon, img_max_lat;
            geoImage.getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<GeoLocator> img_locator = key.getProfile()->getSRS()->createLocator( 
                img_min_lon, img_min_lat, img_max_lon, img_max_lat, 
                !_map->isGeocentric() );
            
            //Set the CS to geocentric if we are dealing with a geocentric map
            if ( _map->isGeocentric() )
            {
                img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
            }

            // Create a layer wrapper that supports opacity.
            // TODO: review this; the Transparent layer holds a back-reference to the actual MapLayer
            TransparentLayer* img_layer = new TransparentLayer( geoImage.getImage(), _map->getImageLayers()[_map->getImageLayers().size()-1] );
            img_layer->setLevelOfDetail(imageLOD);
            img_layer->setLocator( img_locator.get());
            img_layer->setMinFilter( layer->getImageLayerOptions().minFilter().value() );
            img_layer->setMagFilter( layer->getImageLayerOptions().magFilter().value() );

            unsigned int newLayer = _map->getImageLayers().size() - 1;
            tile->setColorLayer( newLayer, img_layer );

            if (needToUpdateImagery)
            {
                tile->updateImagery( layer->getId(), _map.get(), _tileFactory.get());
            }
        }
        else
        {
            // this can happen if there's no data in the new layer for the given tile.
            // we will rely on the driver to dump out a warning if this is an error.

            //OE_INFO << LC << 
            //    "Adding layer " << layer->getName()
            //    << ": Could not create geoimage for tile " << key.str() << std::endl;
        }

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty(true);
        else
            tile->markTileForRegeneration();
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
        Threading::ScopedReadLock mapDataLock(_map->getMapDataMutex());
        hasElevation = _map->getElevationLayers().size() > 0;
    }    

    //Update the elevation hint
    tile->setHasElevationHint( hasElevation );

    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    if (heightFieldLayer)
    {
        //In standard mode, just load the elevation data and dirty the tile.

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            //if (!_options.getPreemptiveLOD())
        {
            osg::ref_ptr<osg::HeightField> hf;
            if (hasElevation)
            {
                hf = _map->createHeightField( key, true, _terrainOptions.elevationInterpolation().value());
            }
            if (!hf.valid()) hf = OSGTileFactory::createEmptyHeightField( key );
            heightFieldLayer->setHeightField( hf.get() );
            hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
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
                tile->resetElevationRequests();
                tile->markTileForRegeneration();
            }
            else
            {
                //Always load the first LOD so the children tiles can have something to use for placeholders
                if (tile->getKey().getLevelOfDetail() == 1)
                {
                    osg::ref_ptr<osg::HeightField> hf = _map->createHeightField( key, true, _terrainOptions.elevationInterpolation().value());
                    if (!hf.valid()) hf = OSGTileFactory::createEmptyHeightField( key );
                    heightFieldLayer->setHeightField( hf.get() );
                    hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
                    tile->setElevationLOD(tile->getKey().getLevelOfDetail());
                    tile->markTileForRegeneration();
                }
                else
                {
                    //Set the elevation LOD to -1
                    tile->setElevationLOD(-1);
                    tile->resetElevationRequests();
                }
            }
        }
    }
}


void
OSGTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    //TODO: review the use of this read-mutex here. do we need it??
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    OE_DEBUG << LC << "Found " << tiles.size() << std::endl;

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        updateElevation(tile);
    }
}

void
OSGTerrainEngineNode::onImageLayerRemoved( ImageLayer* layer, unsigned int index )
{
    if ( layer )
        removeImageLayer( index );
}

void
OSGTerrainEngineNode::onElevationLayerRemoved( ElevationLayer* layer, unsigned int index )
{
    if ( layer )
        removeElevationLayer( index );
}

void
OSGTerrainEngineNode::removeImageLayer( unsigned int index )
{
    //TODO: need this mutex??
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        Threading::ScopedWriteLock tileLock(tile->getTileLayersMutex());

        //OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());
        //An image layer was removed, so reorganize the color layers in the tiles to account for it's removal
        std::vector< osg::ref_ptr< osgTerrain::Layer > > layers;
        for (unsigned int i = 0; i < itr->get()->getNumColorLayers(); ++i)
        {   
            //Skip the layer that is being removed
            if (i != index)
            {
                osgTerrain::Layer* imageLayer = itr->get()->getColorLayer(i);
                if (imageLayer)
                {
                    layers.push_back(imageLayer);
                }
            }
            //Set the current value to NULL
            itr->get()->setColorLayer( i, NULL);
        }

        //Reset the color layers to the correct order
        for (unsigned int i = 0; i < layers.size(); ++i)
        {
            itr->get()->setColorLayer( i, layers[i].get() );
        }

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty( true );
        else
            tile->markTileForRegeneration();
    }
    
    updateTextureCombining();

    OE_DEBUG << "[osgEarth::Map::removeImageSource] end " << std::endl;  
}

void
OSGTerrainEngineNode::removeElevationLayer( unsigned int index )
{
    //TODO: need this mutex??
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        updateElevation( tile );
    }
}

void
OSGTerrainEngineNode::onImageLayerMoved( ImageLayer* layer, unsigned int oldIndex, unsigned int newIndex )
{
    if ( layer )
        moveImageLayer( oldIndex, newIndex );
}

void 
OSGTerrainEngineNode::onElevationLayerMoved( ElevationLayer* layer, unsigned int oldIndex, unsigned int newIndex )
{
    if ( layer )
        moveElevationLayer( oldIndex, newIndex );
}

void
OSGTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
    //TODO: need thi mutex???
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        Threading::ScopedWriteLock tileLock(tile->getTileLayersMutex());

        //Collect the current color layers
        std::vector< osg::ref_ptr< osgTerrain::Layer > > layers;

        for (unsigned int i = 0; i < itr->get()->getNumColorLayers(); ++i)
        {              
            layers.push_back(itr->get()->getColorLayer(i));
        }

        //Swap the original position
        osg::ref_ptr< osgTerrain::Layer > layer = layers[oldIndex];
        layers.erase(layers.begin() + oldIndex);
        layers.insert(layers.begin() + newIndex, layer.get());

        for (unsigned int i = 0; i < layers.size(); ++i)
        {
            itr->get()->setColorLayer( i, layers[i].get() );
        }

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty( true );
        else
            tile->markTileForRegeneration();
    }     

    updateTextureCombining();
}

void
OSGTerrainEngineNode::moveElevationLayer( unsigned int oldIndex, unsigned int newIndex )
{
    //TODO: need this mutex??
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );
    OE_DEBUG << "Found " << tiles.size() << std::endl;

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        updateElevation(tile);
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

typedef std::list<const osg::StateSet*> StateSetStack;
static osg::StateAttribute::GLModeValue 
getModeValue(const StateSetStack& statesetStack, osg::StateAttribute::GLMode mode)
{
    osg::StateAttribute::GLModeValue base_val = osg::StateAttribute::ON;
    for(StateSetStack::const_iterator itr = statesetStack.begin();
        itr != statesetStack.end();
        ++itr)
    {
        osg::StateAttribute::GLModeValue val = (*itr)->getMode(mode);
        if ((val & ~osg::StateAttribute::INHERIT)!=0)
        {
            if ((val & osg::StateAttribute::PROTECTED)!=0 ||
                (base_val & osg::StateAttribute::OVERRIDE)==0)
            {
                base_val = val;
            }
        }
    }
    return base_val;
}

void
OSGTerrainEngineNode::traverse( osg::NodeVisitor& nv )
{
    //TODO: this should not change the uniforms during the cull traversal...
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
	{
        //Update the lighting uniforms
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if (cv)
        {
            StateSetStack statesetStack;

            osgUtil::StateGraph* sg = cv->getCurrentStateGraph();
            while(sg)
            {
                const osg::StateSet* stateset = sg->getStateSet();
                if (stateset)
                {
                    statesetStack.push_front(stateset);
                }                
                sg = sg->_parent;
            }

            //Update the lighting uniforms
            osg::StateAttribute::GLModeValue lightingEnabled = getModeValue(statesetStack, GL_LIGHTING);     
            osg::Uniform* lightingEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("osgEarth_lightingEnabled", osg::Uniform::BOOL);
            lightingEnabledUniform->set((lightingEnabled & osg::StateAttribute::ON)!=0);

            const unsigned int numLights = 8;
            osg::Uniform* lightsEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("osgEarth_lightsEnabled", osg::Uniform::BOOL, numLights);
            for (unsigned int i = 0; i < numLights; ++i)
            {
                osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT0 + i);     
                lightsEnabledUniform->setElement(i, (lightEnabled & osg::StateAttribute::ON)!=0);
            }				
        }
    }

    TerrainEngineNode::traverse( nv );
}

void
OSGTerrainEngineNode::updateTextureCombining()
{
    if ( _texCompositor.valid() )
    {
        // ASSUMPTION: map data mutex is held
        _texCompositor->updateGlobalStateSet( getOrCreateStateSet(), _map->getImageLayers().size() );
    }

    //TODO: reintroduce support for _terrainOptions.combineLayers() ????
}


