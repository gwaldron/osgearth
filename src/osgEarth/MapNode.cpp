/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/MapNode>
#include <osgEarth/Locators>
#include <osgEarth/FindNode>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/MultiPassTerrainTechnique>
#include <osgEarth/TileSourceFactory>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/MaskNode>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/Notify>
#include <osg/CullFace>
#include <osg/NodeVisitor>
#include <osg/FragmentProgram>
#include <osg/PolygonOffset>

using namespace osgEarth;
using namespace OpenThreads;

//static
OpenThreads::ReentrantMutex MapNode::s_mapNodeCacheMutex;
static unsigned int s_mapNodeID = 0;
//Caches the MapNodes that have been created
typedef std::map<unsigned int, osg::observer_ptr<MapNode> > MapNodeCache;


#define CHILD_TERRAINS 0
#define CHILD_MODELS   1

static
MapNodeCache& getMapNodeCache()
{
    static MapNodeCache s_cache;
    return s_cache;
}

// adapter that lets MapNode listen to Map events
struct MapNodeMapCallbackProxy : public MapCallback
{
    MapNodeMapCallbackProxy(MapNode* node) : _node(node) { }
    osg::observer_ptr<MapNode> _node;

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
    void onModelLayerAdded( ModelLayer* layer ) {
        _node->onModelLayerAdded( layer );
    }
    void onModelLayerRemoved( ModelLayer* layer ) {
        _node->onModelLayerRemoved( layer );
    }
    void onTerrainMaskLayerAdded( ModelLayer* layer ) {
        _node->onTerrainMaskLayerAdded( layer );
    }
    void onTerrainMaskLayerRemoved( ModelLayer* layer ) {
        _node->onTerrainMaskLayerRemoved( layer );
    }
};


void
MapNode::registerMapNode(MapNode* mapNode)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    getMapNodeCache()[mapNode->_id] = mapNode;
    osg::notify(osg::INFO) << "[osgEarth::MapNode] Registered " << mapNode->_id << std::endl;
}

void
MapNode::unregisterMapNode(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    MapNodeCache::iterator k = getMapNodeCache().find( id);
    if (k != getMapNodeCache().end())
    {
        getMapNodeCache().erase(k);
        osg::notify(osg::INFO) << "[osgEarth::MapNode] Unregistered " << id << std::endl;
    }
}

MapNode*
MapNode::getMapNodeById(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    MapNodeCache::const_iterator k = getMapNodeCache().find( id);
    if (k != getMapNodeCache().end()) return k->second.get();
    return 0;
}

unsigned int
MapNode::getId() const
{
    return _id;
}


MapNode::MapNode() :
_map( new Map() )
{
    init();
}

MapNode::MapNode( Map* map ) :
_map( map )
{
    init();
}

MapNode::MapNode( const MapEngineProperties& engineProps ) :
_map( new Map() ),
_engineProps( engineProps )
{
    init();
}

MapNode::MapNode( Map* map, const MapEngineProperties& engineProps ) :
_map( map? map : new Map() ),
_engineProps( engineProps )
{
    init();
}

void
MapNode::init()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( s_mapNodeCacheMutex );
    _id = s_mapNodeID++;

    _map->setId( _id );

    const osgDB::ReaderWriter::Options* global_options = _map->getGlobalOptions();
    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ? 
        new osgDB::ReaderWriter::Options( *global_options ) :
        NULL;

    // transcribe proxy settings:
    if ( !_engineProps.proxySettings().isSet() )
    {
        if ( !local_options.valid() )
            local_options = new osgDB::ReaderWriter::Options();

        std::stringstream buf;
        buf << local_options->getOptionString() << " "
            << "OSG_CURL_PROXY=" << _engineProps.proxySettings()->hostName() << " "
            << "OSG_CURL_PROXYPORT=" << _engineProps.proxySettings()->port();
		std::string bufStr;
		bufStr = buf.str();
        local_options->setOptionString( bufStr );
    }

    osg::notify(osg::INFO) 
        << "[osgEarth] Map: options string = " 
        << (local_options.valid()? local_options->getOptionString() : "<empty>")
        << std::endl;

    _map->setGlobalOptions( local_options.get() );

    // create the map engine that wil geneate tiles for this node:
    _engine = new MapEngine( _engineProps ); //_map->createMapEngine( _engineProps );

    // make a group for terrains:
    _terrains = new osg::Group();
    //Give the terrain a stateset to protect it from being optimized away by the REMOVE_REDUNDANT_NODES optimization
    _terrains->getOrCreateStateSet();
    this->addChild( _terrains.get() );

    // handle an already-established map profile:
    if ( _map->getProfile() )
    {
        onMapProfileEstablished( _map->getProfile() );
    }

    // make a group for the model layers:
    _models = new osg::Group();
    addChild( _models.get() );

    // overlays:
    _pendingOverlayAutoSetTextureUnit = true;

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
    for( ModelLayerList::const_iterator k = _map->getModelLayers().begin(); k != _map->getModelLayers().end(); k++ )
    {
        onModelLayerAdded( k->get() );
    }
    if ( _map->getTerrainMaskLayer() )
    {
        onTerrainMaskLayerAdded( _map->getTerrainMaskLayer() );
    }

    updateStateSet();

    // install a layer callback for processing further map actions:
    _map->addMapCallback( new MapNodeMapCallbackProxy(this) );

    osg::StateSet* ss = getOrCreateStateSet();
	//ss->setAttributeAndModes( new osg::CullFace() ); //, osg::StateAttribute::ON);
    //ss->setAttributeAndModes( new osg::PolygonOffset( -1, -1 ) );

    if ( _engineProps.enableLighting().isSet() )
    {
        ss->setMode( GL_LIGHTING, _engineProps.enableLighting().value() ? 
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
            osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    }

    registerMapNode(this);
}

MapNode::~MapNode()
{
    unregisterMapNode(_id);

    removeChildren( 0, getNumChildren() );
}

osg::BoundingSphere
MapNode::computeBound() const
{
    if ( isGeocentric() )
    {
        return osg::BoundingSphere( osg::Vec3(0,0,0), getEllipsoidModel()->getRadiusEquator()+25000 );
    }
    else
    {
        return osg::CoordinateSystemNode::computeBound();
    }
}

Map*
MapNode::getMap()
{
    return _map.get();
}

MapEngine*
MapNode::getEngine() const
{
    return _engine.get();
}

const MapEngineProperties&
MapNode::getMapEngineProperties() const
{
    return _engineProps;
}

MapNode*
MapNode::findMapNode( osg::Node* graph )
{
    return findTopMostNodeOfType<MapNode>( graph );
}

bool
MapNode::isGeocentric() const
{
    return _map->getCoordinateSystemType() != Map::CSTYPE_PROJECTED;
}

unsigned int
MapNode::getNumTerrains() const
{
    return _terrainVec.size();
}

osgEarth::VersionedTerrain*
MapNode::getTerrain( unsigned int i ) const
{
    return _terrainVec[i].get();
}

osg::Group*
MapNode::getModelLayerGroup()
{
    return _models.get();
}

void
MapNode::addTerrainCallback( TerrainCallback* cb )
{
    if ( _terrainVec.size() > 0 )
    {
        for( int i=0; i < _terrainVec.size(); i++ )
        {
            _terrainVec[i]->addTerrainCallback( cb );
        }
    }
    else
    {
        _pendingTerrainCallbacks.push_back( cb );
    }
}

void
MapNode::installOverlayNode( osgSim::OverlayNode* overlay, bool autoSetTextureUnit )
{
    if ( _terrainVec.empty() )
    {
        _pendingOverlayNode = overlay;
        _pendingOverlayAutoSetTextureUnit = autoSetTextureUnit;
    }
    else
    {
        overlay->addChild( this->getChild(CHILD_TERRAINS) );
        this->replaceChild( this->getChild(CHILD_TERRAINS), overlay );

        _pendingOverlayNode = 0L;

        if ( autoSetTextureUnit )
        {
            int nextTextureUnit =
                _engineProps.layeringTechnique() == MapEngineProperties::LAYERING_MULTIPASS ? 1 :
                _map->getImageMapLayers().size();

            overlay->setOverlayTextureUnit( nextTextureUnit );
        }
    }
}

void
MapNode::uninstallOverlayNode( osgSim::OverlayNode* overlay )
{
    if ( _terrainVec.empty() )
    {
        _pendingOverlayNode = 0L;
    }
    else
    {
        osg::ref_ptr<osg::Node> overlayChild = overlay->getChild( 0 );
        this->replaceChild( overlay, overlayChild.get() );
    }
}

osg::Group*
MapNode::getTerrainsGroup() {
    return _terrains.get();
}

void
MapNode::onMapProfileEstablished( const Profile* mapProfile )
{
    // set up the CSN values
    _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( _map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED )
        this->setEllipsoidModel( NULL );


    // go through and build the root nodesets.
    int faces_ok = 0;
    for( int face = 0; face < _map->getProfile()->getNumFaces(); face++ )
    {
        VersionedTerrain* terrain = new VersionedTerrain( _map.get(), _engine.get() );

        // install the proper layering technique:

        if ( _engineProps.layeringTechnique() == MapEngineProperties::LAYERING_MULTIPASS )
        {
			terrain->setTerrainTechniquePrototype( new osgEarth::MultiPassTerrainTechnique());
        }
        else // LAYERING_MULTITEXTURE (default)
        {
			terrain->setTerrainTechniquePrototype( new osgEarth::EarthTerrainTechnique() );
        }

        // apply any pending callbacks:
        for( TerrainCallbackList::iterator c = _pendingTerrainCallbacks.begin(); c != _pendingTerrainCallbacks.end(); ++c )
        {
            terrain->addTerrainCallback( c->get() );
        }
        _pendingTerrainCallbacks.clear();


        terrain->setVerticalScale( _engineProps.verticalScale().value() );
        terrain->setSampleRatio( _engineProps.heightFieldSampleRatio().value() );
        _terrains->addChild( terrain );
        _terrainVec.push_back( terrain );

        std::vector< osg::ref_ptr<TileKey> > keys;
        _map->getProfile()->getFaceProfile( face )->getRootKeys( keys, face );

        int numAdded = 0;
        for (unsigned int i = 0; i < keys.size(); ++i)
        {
            // always load the root tiles completely; no deferring. -gw
            bool loadNow = true; //!_engineProps.getPreemptiveLOD();

            osg::Node* node = _engine->createSubTiles( _map.get(), terrain, keys[i].get(), loadNow );
            if (node)
            {
                terrain->addChild(node);
                numAdded++;
            }
            else
            {
                osg::notify(osg::NOTICE) << "[osgEarth::MapNode] Couldn't get tile for " << keys[i]->str() << std::endl;
            }
        }
        if ( numAdded == keys.size() )
        {
            faces_ok++;
        }
    }

    if ( _pendingOverlayNode.valid() )
    {
        installOverlayNode( _pendingOverlayNode.get(), _pendingOverlayAutoSetTextureUnit );
    }
}

void
MapNode::onModelLayerAdded( ModelLayer* layer )
{
    osg::Node* node = layer->createNode();

    if ( node )
    {
        if ( _modelLayerNodes.find( layer ) != _modelLayerNodes.end() )
        {
            osg::notify(osg::WARN)
                << "[osgEarth] Illegal: tried to add the name model layer more than once: " 
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
MapNode::onModelLayerRemoved( ModelLayer* layer )
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
MapNode::onTerrainMaskLayerAdded( ModelLayer* layer )
{
    osg::Node* node = layer->createNode();

    if ( node && node->asGroup() )
    {
        int count = 0;
        MaskNodeFinder f;
        node->accept( f );
        for( std::list<osg::Group*>::iterator i = f._groups.begin(); i != f._groups.end(); ++i )
        {
            (*i)->addChild( _terrains.get() );
            count++;
        }
        this->replaceChild( _terrains.get(), node );
        
        osg::notify(osg::NOTICE)<<"[osgEarth] Installed terrain mask ("
            <<count<< " mask nodes found)" << std::endl;
    }
}

void
MapNode::onTerrainMaskLayerRemoved( ModelLayer* layer )
{
    if ( layer )
    {
        ModelLayerNodeMap::iterator i = _modelLayerNodes.find( layer );
        if ( i != _modelLayerNodes.end() )
        {
            osg::Group* maskNode = i->second->asGroup();
            osg::ref_ptr<osg::Node> child = maskNode->getChild( 0 );
            this->replaceChild( maskNode, child.get() );
        }
    }
}

void
MapNode::onMapLayerAdded( MapLayer* layer, unsigned int index )
{
    if ( _engineProps.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
    {
        if ( layer && layer->getTileSource() )
        {
            for( unsigned int i=0; i<_terrainVec.size(); i++ )
            {
                _terrainVec[i]->incrementRevision();
                _terrainVec[i]->updateTaskServiceThreads();
            }
        }
        updateStateSet();
    }

    if ( layer && layer->getTileSource() )
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

void
MapNode::addImageLayer( MapLayer* layer )
{
    OpenThreads::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    for( unsigned int i=0; i<_terrainVec.size(); i++ )
    {            
        VersionedTerrain* terrain = _terrainVec[i].get();
        TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

        for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
            OpenThreads::ScopedWriteLock tileLock(tile->getTileLayersMutex());

            //Create a TileKey from the TileID
            osgTerrain::TileID tileId = tile->getTileID();
		    osg::ref_ptr< TileKey > key = new TileKey( i, TileKey::getLOD(tileId), tileId.x, tileId.y, _map->getProfile()->getFaceProfile( i ) );

            osg::ref_ptr< GeoImage > geoImage;

            bool needToUpdateImagery = false;
            int imageLOD = -1;

            //If we are in preemptiveLOD mode, just add an empty placeholder image for the new layer.  Otherwise, go ahead and get the image
            if (( _engineProps.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD ) ||
               ((_engineProps.loadingPolicy()->mode() == LoadingPolicy::MODE_SEQUENTIAL) && key->getLevelOfDetail() == 1))
            {
                geoImage = _engine->createValidGeoImage( layer, key.get() );
                imageLOD = key->getLevelOfDetail();
            }
            else
            {
                geoImage = new GeoImage(ImageUtils::getEmptyImage(), key->getGeoExtent() );
                needToUpdateImagery = true;
            }

            if (geoImage.valid())
            {
                double img_min_lon, img_min_lat, img_max_lon, img_max_lat;

                //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
                osg::ref_ptr<GeoLocator> img_locator; // = key->getProfile()->getSRS()->createLocator();

                
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
                    img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat, !isGeocentric );
                    img_locator = new MercatorLocator( *img_locator.get(), geoImage->getExtent() );
                }
                else
                {
                    geoImage->getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                    img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat, !isGeocentric );
                }

                //Set the CS to geocentric is we are dealing with a geocentric map
                if ( _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC || _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC_CUBE)
                {
                    img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
                }

                //osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( geoImage->getImage() );
				TransparentLayer* img_layer = new TransparentLayer( geoImage->getImage(), _map->getImageMapLayers()[_map->getImageMapLayers().size()-1] );
                img_layer->setLevelOfDetail(imageLOD);
                img_layer->setLocator( img_locator.get());

                unsigned int newLayer = _map->getImageMapLayers().size() - 1;
                tile->setColorLayer( newLayer, img_layer );

                if (needToUpdateImagery)
                {
                    tile->updateImagery( layer->getId(), _map.get(), _engine.get());
                }
            }
            else
            {
                osg::notify(osg::NOTICE) << "Could not create geoimage for " << layer->getName() << " " << key->str() << std::endl;
            }

            
            if ( _engineProps.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
                tile->setDirty(true);
            else
                tile->markTileForRegeneration();
        }
    }
    updateStateSet();       
}

void
MapNode::updateElevation(VersionedTile* tile)
{
    OpenThreads::ScopedWriteLock tileLock( tile->getTileLayersMutex() );

    osg::ref_ptr< const TileKey > key = tile->getKey();

    bool hasElevation;
    {
        OpenThreads::ScopedReadLock mapDataLock(_map->getMapDataMutex());
        hasElevation = _map->getHeightFieldMapLayers().size() > 0;
    }    

    //Update the elevation hint
    tile->setHasElevationHint( hasElevation );

    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    if (heightFieldLayer)
    {
        //In standard mode, just load the elevation data and dirty the tile.
        
        if ( _engineProps.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
        //if (!_engineProps.getPreemptiveLOD())
        {
            osg::ref_ptr<osg::HeightField> hf;
            if (hasElevation)
            {
                hf = _map->createHeightField( key.get(), true );
            }
            if (!hf.valid()) hf = MapEngine::createEmptyHeightField( key.get() );
            heightFieldLayer->setHeightField( hf.get() );
            hf->setSkirtHeight( tile->getBound().radius() * _engineProps.heightFieldSkirtRatio().value() );
            tile->setDirty(true);
        }
        else
        {
            //In preemptive mode, if there is no elevation, just clear out all the elevation on the tiles
            if (!hasElevation)
            {
                osg::ref_ptr<osg::HeightField> hf = MapEngine::createEmptyHeightField( key.get() );
                heightFieldLayer->setHeightField( hf.get() );
                hf->setSkirtHeight( tile->getBound().radius() * _engineProps.heightFieldSkirtRatio().value() );
                tile->setElevationLOD( key->getLevelOfDetail() );
                tile->resetElevationRequests();
                tile->markTileForRegeneration();
            }
            else
            {
                //Always load the first LOD so the children tiles can have something to use for placeholders
                if (tile->getKey()->getLevelOfDetail() == 1)
                {
                    osg::ref_ptr<osg::HeightField> hf = _map->createHeightField( key.get(), true );
                    if (!hf.valid()) hf = MapEngine::createEmptyHeightField( key.get() );
                    heightFieldLayer->setHeightField( hf.get() );
                    hf->setSkirtHeight( tile->getBound().radius() * _engineProps.heightFieldSkirtRatio().value() );
                    tile->setElevationLOD(tile->getKey()->getLevelOfDetail());
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
MapNode::addHeightFieldLayer( MapLayer* layer )
{
    osg::notify(osg::INFO) << "[osgEarth::MapEngine::addHeightFieldLayer] Begin " << std::endl;
    OpenThreads::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrainVec.size(); ++i)
    {            
        VersionedTerrain* terrain = _terrainVec[i].get();
        TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

        for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
            updateElevation(tile);
        }
    }
}

void
MapNode::onMapLayerRemoved( MapLayer* layer, unsigned int index )
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
MapNode::removeImageLayer( unsigned int index )
{
    OpenThreads::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrainVec.size(); ++i)
    {            
        VersionedTerrain* terrain = _terrainVec[i].get();
        TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );

        for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
            OpenThreads::ScopedWriteLock tileLock(tile->getTileLayersMutex());

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

            
            if ( _engineProps.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
                tile->setDirty( true );
            else
                tile->markTileForRegeneration();
        }
    }

    updateStateSet();

    osg::notify(osg::INFO) << "[osgEarth::Map::removeImageSource] end " << std::endl;  
}

void
MapNode::removeHeightFieldLayer( unsigned int index )
{
    OpenThreads::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrainVec.size(); ++i)
    {            
        VersionedTerrain* terrain = _terrainVec[i].get();
        TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        //osg::notify(osg::NOTICE) << "Found " << tiles.size() << std::endl;

        for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
            updateElevation( tile );
        }
    }
}

void
MapNode::onMapLayerMoved( MapLayer* layer, unsigned int oldIndex, unsigned int newIndex )
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
MapNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
    OpenThreads::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrainVec.size(); ++i)
    {            
        VersionedTerrain* terrain = _terrainVec[i].get();
        TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

        for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
            OpenThreads::ScopedWriteLock tileLock(tile->getTileLayersMutex());

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

            if ( _engineProps.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
                tile->setDirty( true );
            else
                tile->markTileForRegeneration();
        }
    } 

    updateStateSet();
}

void
MapNode::moveHeightFieldLayer( unsigned int oldIndex, unsigned int newIndex )
{
    OpenThreads::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrainVec.size(); ++i)
    {            
        VersionedTerrain* terrain = _terrainVec[i].get();
        TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

        for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
            updateElevation(tile);
        }
    }
}

void MapNode::updateStateSet()
{
	if (_engineProps.layeringTechnique() == MapEngineProperties::LAYERING_MULTIPASS)
        return;

    if ( _engineProps.combineLayers() == true )
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


