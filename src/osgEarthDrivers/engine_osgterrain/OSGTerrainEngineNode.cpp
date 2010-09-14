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
#include "MultiTextureTerrainTechnique"
#include "MultiPassTerrainTechnique"
#include "CompositingTerrainTechnique"
#include "CustomTerrain"
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
    void onMapLayerAdded( MapLayer* layer, unsigned int index ) {
        _node->onMapLayerAdded(layer, index);
    }
    void onMapLayerRemoved( MapLayer* layer, unsigned int index ) {
        _node->onMapLayerRemoved(layer, index);
    }
    void onMapLayerMoved( MapLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
        _node->onMapLayerMoved(layer,oldIndex,newIndex);
    }
#if 0
    void onModelLayerAdded( ModelLayer* layer ) {
        _node->onModelLayerAdded( layer );
    }
    void onModelLayerRemoved( ModelLayer* layer ) {
        _node->onModelLayerRemoved( layer );
    }
    void onMaskLayerAdded( MaskLayer* layer ) {
        _node->onMaskLayerAdded( layer );
    }
    void onMaskLayerRemoved( MaskLayer* layer ) {
        _node->onMaskLayerRemoved( layer );
    }
#endif
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
        unsigned int index = 0;
        for( MapLayerList::const_iterator i = _map->getHeightFieldMapLayers().begin(); i != _map->getHeightFieldMapLayers().end(); i++ )
        {
            onMapLayerAdded( i->get(), index++ );
        }

        index = 0;
        for( MapLayerList::const_iterator j = _map->getImageMapLayers().begin(); j != _map->getImageMapLayers().end(); j++ )
        {
            onMapLayerAdded( j->get(), index++ );
        }
#if 0
        for( ModelLayerList::const_iterator k = _map->getModelLayers().begin(); k != _map->getModelLayers().end(); k++ )
        {
            onModelLayerAdded( k->get() );
        }
        if ( _map->getTerrainMaskLayer() )
        {
            onMaskLayerAdded( _map->getTerrainMaskLayer() );
        }
#endif
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
    if ( _terrainOptions.layeringTechnique() == TerrainOptions::LAYERING_MULTIPASS )
    {
        _terrain->setTerrainTechniquePrototype( new MultiPassTerrainTechnique());
        OE_INFO << LC << "Layering technique = MULTIPASS" << std::endl;
    }
    else
    {
        CustomTerrainTechnique* tech = 0;

        if ( _terrainOptions.layeringTechnique() == TerrainOptions::LAYERING_MULTITEXTURE )
        {
            tech = new MultiTextureTerrainTechnique();
            OE_INFO << LC << "Layering technique = MULTITEXTURE" << std::endl;
        }

        else if ( _terrainOptions.layeringTechnique() == TerrainOptions::LAYERING_COMPOSITE )
        {
            tech = new CompositingTerrainTechnique();
            OE_INFO << LC << "Layering technique = COMPOSITE" << std::endl;
        }

        if ( tech )
        {
            //If we are using triangulate interpolation, tell the terrain technique to just create simple triangles with
            //consistent orientation rather than trying to optimize the orientation
            if ( _terrainOptions.elevationInterpolation() == INTERP_TRIANGULATE )
            {
                tech->setOptimizeTriangleOrientation( false );
            }

            _terrain->setTerrainTechniquePrototype( tech );
        }
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

#if 0
void
OSGTerrainEngineNode::onModelLayerAdded( ModelLayer* layer )
{
    osg::Node* node = layer->getOrCreateNode();

    if ( node )
    {
        if ( _modelLayerNodes.find( layer ) != _modelLayerNodes.end() )
        {
            OE_WARN
                << "Illegal: tried to add the name model layer more than once: " 
                << layer->getName()
                << std::endl;
        }
        else
        {
            // treat overlay node as a special case
            if ( dynamic_cast<osgSim::OverlayNode*>( node ) )
            {
                osgSim::OverlayNode* overlay = static_cast<osgSim::OverlayNode*>( node );
                bool autoTextureUnit = overlay->getOverlayTextureUnit() == 0; // indicates AUTO mode
                installOverlayNode( overlay, autoTextureUnit );
            }
            else
            {
                _models->addChild( node );
            }

            ModelSource* ms = layer->getModelSource();
            if ( ms && ms->getOptions()->renderOrder().isSet() )
            {
                node->getOrCreateStateSet()->setRenderBinDetails(
                    ms->getOptions()->renderOrder().value(), "RenderBin" );
            }

            _modelLayerNodes[ layer ] = node;
        }
    }
}

void
OSGTerrainEngineNode::onModelLayerRemoved( ModelLayer* layer )
{
    if ( layer )
    {
        // look up the node associated with this model layer.
        ModelLayerNodeMap::iterator i = _modelLayerNodes.find( layer );
        if ( i != _modelLayerNodes.end() )
        {
            osg::Node* node = i->second;

            if ( dynamic_cast<osgSim::OverlayNode*>( node ) )
            {
                // handle the special-case overlay node
                uninstallOverlayNode( static_cast<osgSim::OverlayNode*>(node) );
            }
            else
            {
                _models->removeChild( node );
            }

            _modelLayerNodes.erase( i );
        }
    }
}

struct MaskNodeFinder : public osg::NodeVisitor {
    MaskNodeFinder() : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ) { }
    void apply( osg::Group& group ) {
        if ( dynamic_cast<MaskNode*>( &group ) ) {
            _groups.push_back( &group );
        }
        traverse(group);
    }
    std::list< osg::Group* > _groups;
};

void
OSGTerrainEngineNode::onMaskLayerAdded( MaskLayer* layer )
{
    osg::Node* node = layer->getOrCreateNode();

    if ( node && node->asGroup() )
    {
        int count = 0;
        MaskNodeFinder f;
        node->accept( f );
        for( std::list<osg::Group*>::iterator i = f._groups.begin(); i != f._groups.end(); ++i )
        {
            (*i)->addChild( _terrainContainer.get() );
            count++;
        }
        this->replaceChild( _terrainContainer.get(), node );

        OE_NOTICE<<"Installed terrain mask ("
            <<count<< " mask nodes found)" << std::endl;

        _maskLayerNode = node->asGroup();
    }
}

void
OSGTerrainEngineNode::onMaskLayerRemoved( MaskLayer* layer )
{
    if ( layer && _maskLayerNode )
    {
        osg::ref_ptr<osg::Node> child = _maskLayerNode->getChild( 0 );
        this->replaceChild( _maskLayerNode, child.get() );
        _maskLayerNode = 0L;
    }
}
#endif

void
OSGTerrainEngineNode::onMapLayerAdded( MapLayer* layer, unsigned int index )
{
    if ( layer )
    {
#if 0
        if ( _terrainOptions.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
        {
            if ( layer->getTileSource() )
            {
                getTerrain()->incrementRevision();
                getTerrain()->updateTaskServiceThreads();
            }
            updateStateSet();
        }
#endif

        if ( layer->getTileSource() )
        {        
            if ( layer->getType() == MapLayer::TYPE_IMAGE )
            {
                addImageLayer( layer );
            }
            else if ( layer->getType() == MapLayer::TYPE_HEIGHTFIELD )
            {
                addHeightFieldLayer( layer );
            }
        }
    }
}

void
OSGTerrainEngineNode::addImageLayer( MapLayer* layer )
{
    //TODO: review the scope of this mapdata mutex lock within this method. We can 
    // probably optimize it some
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    // apply a controller to the layer so we can process runtime property updates:
    //layer->setController( _mapLayerController.get() );

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

        osg::ref_ptr< GeoImage > geoImage;

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
            geoImage = new GeoImage(ImageUtils::createEmptyImage(), key.getGeoExtent() );
            needToUpdateImagery = true;
        }

        if (geoImage.valid())
        {
            double img_min_lon, img_min_lat, img_max_lon, img_max_lat;

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<GeoLocator> img_locator;

            // Use a special locator for mercator images (instead of reprojecting).
            // We do this under 2 conditions when we have mercator tiles:
            // a) The map is geocentric; or
            // b) The map is projected but is also "geographic" (i.e., plate carre)
            bool isGeocentric = _map->getCoordinateSystemType() != Map::CSTYPE_PROJECTED;
            bool isGeographic = _map->getProfile()->getSRS()->isGeographic();
            bool canUseMercatorLocator = geoImage->getSRS()->isMercator() && (isGeocentric || isGeographic);

            if ( canUseMercatorLocator && layer->useMercatorFastPath() == true )
            {
                GeoExtent geog_ext = geoImage->getExtent().transform(geoImage->getExtent().getSRS()->getGeographicSRS());
                geog_ext.getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                img_locator = key.getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat, !isGeocentric );
                img_locator = new MercatorLocator( *img_locator.get(), geoImage->getExtent() );
            }
            else
            {
                geoImage->getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                img_locator = key.getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat, !isGeocentric );
            }

            //Set the CS to geocentric if we are dealing with a geocentric map
            if ( _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC || _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC_CUBE)
            {
                img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
            }

            // Create a layer wrapper that supports opacity.
            // TODO: review this; the Transparent layer holds a back-reference to the actual MapLayer
            TransparentLayer* img_layer = new TransparentLayer( geoImage->getImage(), _map->getImageMapLayers()[_map->getImageMapLayers().size()-1] );
            img_layer->setLevelOfDetail(imageLOD);
            img_layer->setLocator( img_locator.get());
            img_layer->setMinFilter( layer->getMinFilter().value());
            img_layer->setMagFilter( layer->getMagFilter().value());

            unsigned int newLayer = _map->getImageMapLayers().size() - 1;
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
        hasElevation = _map->getHeightFieldMapLayers().size() > 0;
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
OSGTerrainEngineNode::addHeightFieldLayer( MapLayer* layer )
{
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
OSGTerrainEngineNode::onMapLayerRemoved( MapLayer* layer, unsigned int index )
{
    if ( layer )
    {
        if ( layer->getType() == MapLayer::TYPE_IMAGE )
        {
            removeImageLayer( index );
        }
        else if ( layer->getType() == MapLayer::TYPE_HEIGHTFIELD )
        {
            removeHeightFieldLayer( index );
        }
    }
}

void
OSGTerrainEngineNode::removeImageLayer( unsigned int index )
{
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
OSGTerrainEngineNode::removeHeightFieldLayer( unsigned int index )
{
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
OSGTerrainEngineNode::onMapLayerMoved( MapLayer* layer, unsigned int oldIndex, unsigned int newIndex )
{
    if ( layer )
    {
        if ( layer->getType() == MapLayer::TYPE_IMAGE )
        {
            moveImageLayer( oldIndex, newIndex );
        }
        else if ( layer->getType() == MapLayer::TYPE_HEIGHTFIELD )
        {
            moveHeightFieldLayer( oldIndex, newIndex );
        }
    }
}

void
OSGTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
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
OSGTerrainEngineNode::moveHeightFieldLayer( unsigned int oldIndex, unsigned int newIndex )
{
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
    //TODO...
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
    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR &&
        _terrainOptions.layeringTechnique() == TerrainOptions::LAYERING_MULTITEXTURE)
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
    // ASSUMPTION: map data mutex is held

    if (_terrainOptions.layeringTechnique() == TerrainOptions::LAYERING_MULTITEXTURE &&
        _terrainOptions.combineLayers() == true )
    {
        int numLayers = _map->getImageMapLayers().size();

        osg::StateSet* stateset = getOrCreateStateSet();

        if (numLayers == 1)
        {
            osg::TexEnv* texenv = new osg::TexEnv(osg::TexEnv::MODULATE);
            stateset->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
        }
        else if (numLayers >= 2)
        {
            //Blend together the colors and accumulate the alpha values of textures 0 and 1 on unit 0
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::ADD);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_Alpha(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateset->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
            }


            //For textures 2 and beyond, blend them together with the previous
            //Add the alpha values of this unit and the previous unit
            for (int unit = 1; unit < numLayers-1; ++unit)
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::ADD);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_Alpha(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateset->setTextureAttributeAndModes(unit, texenv, osg::StateAttribute::ON);
            }

            //Modulate the colors to get proper lighting on the last unit
            //Keep the alpha results from the previous stage
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::MODULATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::REPLACE);

                texenv->setSource0_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::PRIMARY_COLOR);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                stateset->setTextureAttributeAndModes(numLayers-1, texenv, osg::StateAttribute::ON);
            }
        }
    }
}
