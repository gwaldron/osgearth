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
#if 0

#include <osgEarth/MapNode>
#include <osgEarth/Locators>
#include <osgEarth/FindNode>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/MultiPassTerrainTechnique>
#include <osgEarth/CompositingTerrainTechnique>
#include <osgEarth/TileSourceFactory>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/MaskNode>
#include <osgEarth/EarthFile>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/Notify>
#include <osg/CullFace>
#include <osg/NodeVisitor>
#include <osg/FragmentProgram>
#include <osg/PolygonOffset>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[MapNode] "

#define CHILD_TERRAINS 0
#define CHILD_MODELS   1

//---------------------------------------------------------------------------

//static
OpenThreads::ReentrantMutex MapNode::s_mapNodeCacheMutex;
static unsigned int s_mapNodeID = 0;
//Caches the MapNodes that have been created
typedef std::map<unsigned int, osg::observer_ptr<MapNode> > MapNodeCache;

static
MapNodeCache& getMapNodeCache()
{
    static MapNodeCache s_cache;
    return s_cache;
}

//---------------------------------------------------------------------------

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
    void onMaskLayerAdded( MaskLayer* layer ) {
        _node->onMaskLayerAdded( layer );
    }
    void onMaskLayerRemoved( MaskLayer* layer ) {
        _node->onMaskLayerRemoved( layer );
    }
};

//---------------------------------------------------------------------------

namespace osgEarth
{
    class MapNodeMapLayerController : public MapLayerController
    {
    public:
        MapNodeMapLayerController( MapNode* mapNode ) : _node(mapNode) { }

        void updateOpacity( MapLayer* layer ) 
        {
            _node->updateLayerOpacity( layer );
        }

        void updateEnabled( MapLayer* layer)
        {
            _node->updateLayerEnabled( layer );
        }

    private:
        MapNode* _node;
    };
}

//---------------------------------------------------------------------------

class RemoveBlacklistedFilenamesVisitor : public osg::NodeVisitor
{
public:
    RemoveBlacklistedFilenamesVisitor():
      osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
          _numRemoved(0)
      {
      }

      virtual void apply(osg::PagedLOD& node)
      {
          for (unsigned int i = 0; i < node.getNumFileNames();)
          {
              const std::string &filename = node.getFileName(i);
              if (osgEarth::Registry::instance()->isBlacklisted(filename))
              {
                  OE_DEBUG << "Removing blacklisted child" << i << "  " << filename <<  std::endl;
                  //Adjust the previous LOD's range so that it becomes the last child
                  if (i > 0)
                  {
                      node.setRange(i-1, 0, FLT_MAX);
                  }
                  node.removeChildren(i, 1);
                  _numRemoved++;
              }
              else
              {
                  i++;
              }
          }

          traverse(node);
      }

      unsigned int _numRemoved;
};

//---------------------------------------------------------------------------

void
MapNode::registerMapNode(MapNode* mapNode)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    getMapNodeCache()[mapNode->_id] = mapNode;
    OE_INFO << LC << "Registered map " << mapNode->_id << std::endl;
}

void
MapNode::unregisterMapNode(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    MapNodeCache::iterator k = getMapNodeCache().find( id);
    if (k != getMapNodeCache().end())
    {
        getMapNodeCache().erase(k);
        OE_INFO << LC << "Unregistered map " << id << std::endl;
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
_mapOptions( engineProps )
{
    init();
}

MapNode::MapNode( Map* map, const MapEngineProperties& engineProps ) :
_map( map? map : new Map() ),
_mapOptions( engineProps )
{
    init();
}

void
MapNode::init()
{
	//Protect the MapNode from the Optimizer
	setDataVariance(osg::Object::DYNAMIC);


    setName( "osgEarth::MapNode" );

    setNumChildrenRequiringUpdateTraversal(1);

    // Since we have global uniforms in the stateset, mark it dynamic so it is immune to
    // multi-threaded overlap
    getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);

    //Set the layer unit uniforms
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer0_unit", osg::Uniform::INT)->set(0);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer1_unit", osg::Uniform::INT)->set(1);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer2_unit", osg::Uniform::INT)->set(2);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer3_unit", osg::Uniform::INT)->set(3);


    _maskLayerNode = 0L;

    _lastNumBlacklistedFilenames = 0;

    // genearte a new unique mapnode ID
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock( s_mapNodeCacheMutex );
        _id = s_mapNodeID++;
    }

    _map->setId( _id );

    // establish global driver options
    const osgDB::ReaderWriter::Options* global_options = _map->getGlobalOptions();
    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ? 
        new osgDB::ReaderWriter::Options( *global_options ) :
        NULL;

    //Set the global proxy settings
    if ( _mapOptions.proxySettings().isSet() )
    {
		HTTPClient::setProxySettings( _mapOptions.proxySettings().get() );
    }

    if ( local_options.valid() )
    {
        OE_INFO << LC
            << "Options string = " 
            << (local_options.valid()? local_options->getOptionString() : "<empty>")
            << std::endl;
    }

    _map->setGlobalOptions( local_options.get() );

    // validate and adjust the engine properties as necessary:
    validateEngineProps( _mapOptions );

    // create the map engine that wil geneate tiles for this node:
    _engine = new MapEngine( _mapOptions );

    // make a group for terrain nodes:
    _terrainContainer = new osg::CoordinateSystemNode();
    _terrainContainer->setName( "osgEarth::MapNode.terrainContainer" );

    //Give the terrain a stateset to protect it from being optimized away by the REMOVE_REDUNDANT_NODES optimization
    _terrainContainer->getOrCreateStateSet();
    this->addChild( _terrainContainer.get() );

    // handle an already-established map profile:
    if ( _map->getProfile() )
    {
        onMapProfileEstablished( _map->getProfile() );
    }

    // make a group for the model layers:
    _models = new osg::Group();
    _models->setName( "osgEarth::MapNode.modelsGroup" );
    addChild( _models.get() );

    // overlays:
    _pendingOverlayAutoSetTextureUnit = true;

    // the layer controller allows you to use the MapLayer API to affect top-level changes in MapNode:
    _mapLayerController = new MapNodeMapLayerController( this );

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
        onMaskLayerAdded( _map->getTerrainMaskLayer() );
    }

    updateStateSet();

    // install a layer callback for processing further map actions:
    _map->addMapCallback( new MapNodeMapCallbackProxy(this) );

    osg::StateSet* ss = getOrCreateStateSet();
	//ss->setAttributeAndModes( new osg::CullFace() ); //, osg::StateAttribute::ON);
    //ss->setAttributeAndModes( new osg::PolygonOffset( -1, -1 ) );

    if ( _mapOptions.enableLighting().isSet() )
    {
        ss->setMode( GL_LIGHTING, _mapOptions.enableLighting().value() ? 
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
    return _mapOptions;
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

osgEarth::VersionedTerrain*
MapNode::getTerrain() const
{
    if ( _terrainContainer->getNumChildren() > 0 )
        return static_cast<osgEarth::VersionedTerrain*>( _terrainContainer->getChild(0) );
    else
        return 0L;
}

osg::Group*
MapNode::getModelLayerGroup()
{
    return _models.get();
}

void
MapNode::addTerrainCallback( TerrainCallback* cb )
{
    if ( getTerrain() )
        getTerrain()->addTerrainCallback( cb );
    else
        _pendingTerrainCallbacks.push_back( cb );
}

void
MapNode::installOverlayNode( osgSim::OverlayNode* overlay, bool autoSetTextureUnit )
{
    if ( !getTerrain() )
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
                _mapOptions.layeringTechnique() == MapEngineProperties::LAYERING_MULTIPASS ? 1 :
                _map->getImageMapLayers().size();

            overlay->setOverlayTextureUnit( nextTextureUnit );
        }
    }
}

void
MapNode::uninstallOverlayNode( osgSim::OverlayNode* overlay )
{
    if ( !getTerrain() )
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
MapNode::getTerrainContainer()
{
    return _terrainContainer.get();
}

void
MapNode::addTerrainDecorator(osg::Group* decorator)
{    
    decorator->addChild( _terrainContainer.get() );
    _terrainContainer->getParent(0)->replaceChild( _terrainContainer.get(), decorator );
}

void
MapNode::removeTerrainDecorator(osg::Group* decorator)
{
    osg::Node* child = _terrainContainer.get();
    for( osg::Group* g = child->getParent(0); g != this; child = g, g = g->getParent(0) )
    {
        if ( g == decorator )
        {
            g->getParent(0)->replaceChild( g, child );
            break;
        }
    }
}

void
MapNode::onMapProfileEstablished( const Profile* mapProfile )
{
    // set up the CSN values
    _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );
    _map->getProfile()->getSRS()->populateCoordinateSystemNode( _terrainContainer.get() );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( _map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED )
        this->setEllipsoidModel( NULL );

    // go through and build the root nodesets.
    VersionedTerrain* terrain = new VersionedTerrain( _map.get(), _engine.get() );

    // install the proper layering technique:

    if ( _mapOptions.layeringTechnique() == MapEngineProperties::LAYERING_MULTIPASS )
    {
		terrain->setTerrainTechniquePrototype( new osgEarth::MultiPassTerrainTechnique());
        OE_INFO << LC << "Layering technique = MULTIPASS" << std::endl;
    }
    else
    {
        ExtendedTerrainTechnique* tech = 0;

        if ( _mapOptions.layeringTechnique() == MapEngineProperties::LAYERING_MULTITEXTURE )
        {
            tech = new EarthTerrainTechnique();
            OE_INFO << LC << "Layering technique = MULTITEXTURE" << std::endl;
        }

        else if ( _mapOptions.layeringTechnique() == MapEngineProperties::LAYERING_COMPOSITE )
        {
            tech = new CompositingTerrainTechnique();
            OE_INFO << LC << "Layering technique = COMPOSITE" << std::endl;
        }

        if ( tech )
        {
            //If we are using triangulate interpolation, tell the terrain technique to just create simple triangles with
            //consistent orientation rather than trying to optimize the orientation
            if ( _mapOptions.elevationInterpolation() == INTERP_TRIANGULATE )
            {
                tech->setOptimizeTriangleOrientation( false );
            }
            
            terrain->setTerrainTechniquePrototype( tech );
        }
    }

    // apply any pending callbacks:
    for( TerrainCallbackList::iterator c = _pendingTerrainCallbacks.begin(); c != _pendingTerrainCallbacks.end(); ++c )
    {
        terrain->addTerrainCallback( c->get() );
    }
    _pendingTerrainCallbacks.clear();

    terrain->setVerticalScale( _mapOptions.verticalScale().value() );
    terrain->setSampleRatio( _mapOptions.heightFieldSampleRatio().value() );
    
    // put the terrain in its container. TODO: later, this may be the attach point for terrain engine plugins.
    _terrainContainer->addChild( terrain );

    // collect the tile keys comprising the root tiles of the terrain.
    std::vector< osg::ref_ptr<TileKey> > keys;
    _map->getProfile()->getRootKeys( keys );

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        // always load the root tiles completely; no deferring. -gw
        bool loadNow = true; //!_mapOptions.getPreemptiveLOD();

        osg::Node* node = _engine->createSubTiles( _map.get(), terrain, keys[i].get(), loadNow );
        if (node)
        {
            terrain->addChild(node);
        }
        else
        {
            OE_WARN << LC << "Couldn't make tile for root key: " << keys[i]->str() << std::endl;
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
MapNode::onMaskLayerAdded( MaskLayer* layer )
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
MapNode::onMaskLayerRemoved( MaskLayer* layer )
{
    if ( layer && _maskLayerNode )
    {
        osg::ref_ptr<osg::Node> child = _maskLayerNode->getChild( 0 );
        this->replaceChild( _maskLayerNode, child.get() );
        _maskLayerNode = 0L;
    }
}

void
MapNode::onMapLayerAdded( MapLayer* layer, unsigned int index )
{
    if ( layer )
    {
        if ( _mapOptions.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
        {
            if ( layer->getTileSource() )
            {
                getTerrain()->incrementRevision();
                getTerrain()->updateTaskServiceThreads();
            }
            updateStateSet();
        }

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
MapNode::addImageLayer( MapLayer* layer )
{
    //TODO: review the scope of this mapdata mutex lock within this method. We can 
    // probably optimize it some
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );
    
    // apply a controller to the layer so we can process runtime property updates:
    layer->setController( _mapLayerController.get() );

    // visit all existing terrain tiles and inform each one of the new image layer:
    TerrainTileList tiles;
    getTerrain()->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
        Threading::ScopedWriteLock tileLock(tile->getTileLayersMutex());

        //Create a TileKey from the TileID
        osgTerrain::TileID tileId = tile->getTileID();
	    osg::ref_ptr< TileKey > key = new TileKey( TileKey::getLOD(tileId), tileId.x, tileId.y, _map->getProfile() );

        osg::ref_ptr< GeoImage > geoImage;

        bool needToUpdateImagery = false;
        int imageLOD = -1;

        // establish the initial image for this tile.
        //if (( _mapOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD ) ||
        //   ((_mapOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_SEQUENTIAL) && key->getLevelOfDetail() == 1))

        if (_mapOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD ||
            key->getLevelOfDetail() == 1)
        {
            // in standard mode, or at the first LOD in seq/pre mode, fetch the image immediately.
            geoImage = _engine->createValidGeoImage( layer, key.get() );
            imageLOD = key->getLevelOfDetail();
        }
        else
        {
            // in seq/pre mode, set up a placeholder and mark the tile as dirty.
            geoImage = new GeoImage(ImageUtils::createEmptyImage(), key->getGeoExtent() );
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
                img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat, !isGeocentric );
                img_locator = new MercatorLocator( *img_locator.get(), geoImage->getExtent() );
            }
            else
            {
                geoImage->getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat, !isGeocentric );
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
                tile->updateImagery( layer->getId(), _map.get(), _engine.get());
            }
        }
        else
        {
            // this can happen if there's no data in the new layer for the given tile.
            // we will rely on the driver to dump out a warning if this is an error.

            //OE_INFO << LC << 
            //    "Adding layer " << layer->getName()
            //    << ": Could not create geoimage for tile " << key->str() << std::endl;
        }
        
        if ( _mapOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty(true);
        else
            tile->markTileForRegeneration();
    }

    updateStateSet();       
}

void
MapNode::updateElevation(VersionedTile* tile)
{
    Threading::ScopedWriteLock tileLock( tile->getTileLayersMutex() );

    osg::ref_ptr< const TileKey > key = tile->getKey();

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
        
        if ( _mapOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
        //if (!_mapOptions.getPreemptiveLOD())
        {
            osg::ref_ptr<osg::HeightField> hf;
            if (hasElevation)
            {
                hf = _map->createHeightField( key.get(), true, _mapOptions.elevationInterpolation().value());
            }
            if (!hf.valid()) hf = MapEngine::createEmptyHeightField( key.get() );
            heightFieldLayer->setHeightField( hf.get() );
            hf->setSkirtHeight( tile->getBound().radius() * _mapOptions.heightFieldSkirtRatio().value() );
            tile->setDirty(true);
        }
        else
        {
            //In preemptive mode, if there is no elevation, just clear out all the elevation on the tiles
            if (!hasElevation)
            {
                osg::ref_ptr<osg::HeightField> hf = MapEngine::createEmptyHeightField( key.get() );
                heightFieldLayer->setHeightField( hf.get() );
                hf->setSkirtHeight( tile->getBound().radius() * _mapOptions.heightFieldSkirtRatio().value() );
                tile->setElevationLOD( key->getLevelOfDetail() );
                tile->resetElevationRequests();
                tile->markTileForRegeneration();
            }
            else
            {
                //Always load the first LOD so the children tiles can have something to use for placeholders
                if (tile->getKey()->getLevelOfDetail() == 1)
                {
                    osg::ref_ptr<osg::HeightField> hf = _map->createHeightField( key.get(), true, _mapOptions.elevationInterpolation().value());
                    if (!hf.valid()) hf = MapEngine::createEmptyHeightField( key.get() );
                    heightFieldLayer->setHeightField( hf.get() );
                    hf->setSkirtHeight( tile->getBound().radius() * _mapOptions.heightFieldSkirtRatio().value() );
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
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    getTerrain()->getTerrainTiles( tiles );

    OE_DEBUG << LC << "Found " << tiles.size() << std::endl;

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
        updateElevation(tile);
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
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    getTerrain()->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
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

        
        if ( _mapOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty( true );
        else
            tile->markTileForRegeneration();
    }

    updateStateSet();

    OE_DEBUG << "[osgEarth::Map::removeImageSource] end " << std::endl;  
}

void
MapNode::removeHeightFieldLayer( unsigned int index )
{
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    getTerrain()->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
        updateElevation( tile );
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
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    getTerrain()->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
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

        if ( _mapOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty( true );
        else
            tile->markTileForRegeneration();
    }     

    updateStateSet();
}

void
MapNode::moveHeightFieldLayer( unsigned int oldIndex, unsigned int newIndex )
{
    Threading::ScopedReadLock mapDataLock( _map->getMapDataMutex() );

    TerrainTileList tiles;
    getTerrain()->getTerrainTiles( tiles );
    OE_DEBUG << "Found " << tiles.size() << std::endl;

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        VersionedTile* tile = static_cast< VersionedTile* >( itr->get() );
        updateElevation(tile);
    }
}

void MapNode::updateStateSet()
{
    // ASSUMPTION: map data mutex is held

    if ( _mapOptions.layeringTechnique() == MapEngineProperties::LAYERING_MULTITEXTURE )
    {
        if ( _mapOptions.combineLayers() == true )
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

    // update the layer uniform arrays:
    osg::StateSet* stateSet = this->getOrCreateStateSet();

    MapLayerList imageLayers;
    _map->getImageMapLayers( imageLayers );

    stateSet->removeUniform( "osgearth_imagelayer_opacity" );
    stateSet->removeUniform( "osgearth_imagelayer_enabled" );
    
    if ( imageLayers.size() > 0 )
    {
        //Update the layer opacity uniform
        _layerOpacityUniform = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_imagelayer_opacity", imageLayers.size() );
        for( MapLayerList::const_iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
            _layerOpacityUniform->setElement( (int)(i-imageLayers.begin()), i->get()->opacity().value() );
        stateSet->addUniform( _layerOpacityUniform.get() );

        //Update the layer enabled uniform
        _layerEnabledUniform = new osg::Uniform( osg::Uniform::BOOL, "osgearth_imagelayer_enabled", imageLayers.size() );
        for( MapLayerList::const_iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
        {
            _layerEnabledUniform->setElement( (int)(i-imageLayers.begin()), i->get()->enabled().value() );
        }
        stateSet->addUniform( _layerEnabledUniform.get() );
    }
    stateSet->getOrCreateUniform( "osgearth_imagelayer_count", osg::Uniform::INT )->set( (int)imageLayers.size() );
}

void
MapNode::updateLayerOpacity( MapLayer* layer )
{
    MapLayerList imageLayers;
    {
        Threading::ScopedReadLock lock( _map->getMapDataMutex() );
        _map->getImageMapLayers( imageLayers );
    }

    MapLayerList::const_iterator i = std::find( imageLayers.begin(), imageLayers.end(), layer );
    if ( i != imageLayers.end() )
    {
        int layerNum = i - imageLayers.begin();
        _layerOpacityUniform->setElement( layerNum, layer->opacity().value() );
        //OE_INFO << LC << "Updating layer " << layerNum << " opacity to " << layer->opacity().value() << std::endl;
    }
    else
    {
        OE_WARN << LC << "Odd, updateLayerOpacity did not find layer" << std::endl;
    }
}

void
MapNode::updateLayerEnabled( MapLayer* layer )
{
    MapLayerList imageLayers;
    {
        Threading::ScopedReadLock lock( _map->getMapDataMutex() );
        _map->getImageMapLayers( imageLayers );
    }

    MapLayerList::const_iterator i = std::find( imageLayers.begin(), imageLayers.end(), layer );
    if ( i != imageLayers.end() )
    {
        int layerNum = i - imageLayers.begin();
        _layerEnabledUniform->setElement( layerNum, layer->enabled().value() );
        //OE_INFO << LC << "Updating layer " << layerNum << " opacity to " << layer->opacity().value() << std::endl;
    }
    else
    {
        OE_WARN << LC << "Odd, updateLayerOpacity did not find layer" << std::endl;
    }
}

typedef std::list<const osg::StateSet*> StateSetStack;
static osg::StateAttribute::GLModeValue getModeValue(const StateSetStack& statesetStack, osg::StateAttribute::GLMode mode)
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
MapNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        unsigned int numBlacklist = osgEarth::Registry::instance()->getNumBlacklistedFilenames();
        if (numBlacklist != _lastNumBlacklistedFilenames)
        {
            //Only remove the blacklisted filenames if new filenames have been added since last time.
            _lastNumBlacklistedFilenames = numBlacklist;
            RemoveBlacklistedFilenamesVisitor v;
            accept(v);
        }
    }

    if (_mapOptions.layeringTechnique() == MapEngineProperties::LAYERING_MULTITEXTURE)
	{
        //Update the lighting uniforms
		if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
		{
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
    }
    osg::CoordinateSystemNode::traverse(nv);
}

void
MapNode::validateEngineProps( MapEngineProperties& props )
{
    // make sure all the requested properties are compatible, and fall back as necessary.
    const Capabilities& caps = Registry::instance()->getCapabilities();

    // check that the layering technique is supported by the hardware.
    if (props.layeringTechnique() == MapEngineProperties::LAYERING_COMPOSITE &&
        !caps.supportsTextureArrays() )
    {
        OE_WARN << LC << "COMPOSITE layering requires EXT_texture_array; falling back to MULTIPASS" << std::endl;
        props.layeringTechnique() = MapEngineProperties::LAYERING_MULTIPASS;
    }

    if (props.layeringTechnique() == MapEngineProperties::LAYERING_MULTITEXTURE &&
        !caps.supportsMultiTexture() )
    {
        OE_WARN << LC << "MULTITEXTURE layering requires EXT_multitexture; falling back to MULTIPASS" << std::endl;
        props.layeringTechnique() = MapEngineProperties::LAYERING_MULTIPASS;
    }

    // warn against mixing multipass technique with preemptive/sequential mode:
    if (props.layeringTechnique() == MapEngineProperties::LAYERING_MULTIPASS &&
        props.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
    {
        OE_WARN << LC << "MULTIPASS layering is incompatible with preemptive/sequential loading policy; "
            << "falling back on STANDARD mode" << std::endl;
        props.loadingPolicy()->mode() = LoadingPolicy::MODE_STANDARD;
    }
}

#else // OSGEARTH2

#include <osgEarth/MapNode>
#include <osgEarth/MaskNode>
#include <osgEarth/FindNode>
#include <osgEarth/Registry>

using namespace osgEarth;

#define LC "[MapNode] "

//---------------------------------------------------------------------------

// adapter that lets MapNode listen to Map events
struct MapNodeMapCallbackProxy : public MapCallback
{
    MapNodeMapCallbackProxy(MapNode* node) : _node(node) { }
    osg::observer_ptr<MapNode> _node;

#if 0
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
#endif
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
};

//---------------------------------------------------------------------------

namespace osgEarth
{
    class MapNodeMapLayerController : public MapLayerController
    {
    public:
        MapNodeMapLayerController( MapNode* mapNode ) : _node(mapNode) { }

        void updateOpacity( MapLayer* layer ) 
        {
            _node->updateLayerOpacity( layer );
        }

        void updateEnabled( MapLayer* layer)
        {
            _node->updateLayerEnabled( layer );
        }

    private:
        MapNode* _node;
    };
}

//---------------------------------------------------------------------------

class RemoveBlacklistedFilenamesVisitor : public osg::NodeVisitor
{
public:
    RemoveBlacklistedFilenamesVisitor():
      osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
          _numRemoved(0)
      {
      }

      virtual void apply(osg::PagedLOD& node)
      {
          for (unsigned int i = 0; i < node.getNumFileNames();)
          {
              const std::string &filename = node.getFileName(i);
              if (osgEarth::Registry::instance()->isBlacklisted(filename))
              {
                  OE_DEBUG << "Removing blacklisted child" << i << "  " << filename <<  std::endl;
                  //Adjust the previous LOD's range so that it becomes the last child
                  if (i > 0)
                  {
                      node.setRange(i-1, 0, FLT_MAX);
                  }
                  node.removeChildren(i, 1);
                  _numRemoved++;
              }
              else
              {
                  i++;
              }
          }

          traverse(node);
      }

      unsigned int _numRemoved;
};

//---------------------------------------------------------------------------

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

MapNode::MapNode( const MapOptions& options ) :
_map( new Map() ),
_mapOptions( options )
{
    init();
}

MapNode::MapNode( Map* map, const MapOptions& options ) :
_map( map? map : new Map() ),
_mapOptions( options )
{
    init();
}

void
MapNode::init()
{
	// Protect the MapNode from the Optimizer
	setDataVariance(osg::Object::DYNAMIC);

    setName( "osgEarth::MapNode" );

    setNumChildrenRequiringUpdateTraversal( 1 );

    // Since we have global uniforms in the stateset, mark it dynamic so it is immune to
    // multi-threaded overlap
    getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);

    // Set the layer unit uniforms
    // TODO: depcrecate in favor of the opacity uniform array?
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer0_unit", osg::Uniform::INT)->set(0);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer1_unit", osg::Uniform::INT)->set(1);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer2_unit", osg::Uniform::INT)->set(2);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer3_unit", osg::Uniform::INT)->set(3);

    _maskLayerNode = 0L;
    _lastNumBlacklistedFilenames = 0;

    // Set the global proxy settings
    // TODO: this should probably happen elsewhere, like in the registry?
    if ( _mapOptions.proxySettings().isSet() )
    {
		HTTPClient::setProxySettings( _mapOptions.proxySettings().get() );
    }

    // establish global driver options. These are OSG reader-writer options that
    // will make their way to any read* calls down the pipe
    const osgDB::ReaderWriter::Options* global_options = _map->getGlobalOptions();
    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ? 
        new osgDB::ReaderWriter::Options( *global_options ) :
        NULL;

    if ( local_options.valid() )
    {
        OE_INFO << LC
            << "Options string = " 
            << (local_options.valid()? local_options->getOptionString() : "<empty>")
            << std::endl;
    }

    // TODO: not sure why we call this here
    _map->setGlobalOptions( local_options.get() );

    // validate and adjust the engine properties as necessary:
    // TODO: this will move to Engine
    //validateEngineProps( _mapOptions );

    // create the map engine that wil geneate tiles for this node:
//    _engine = new MapEngine( _mapOptions );

    // make a group for the model layers:
    _models = new osg::Group();
    _models->setName( "osgEarth::MapNode.modelsGroup" );
    addChild( _models.get() );

    // overlays:
    _pendingOverlayAutoSetTextureUnit = true;

    // the layer controller allows you to use the MapLayer API to affect top-level changes in MapNode:
    _mapLayerController = new MapNodeMapLayerController( this );

    // go through the map and process any already-installed layers:
    // TODO: non-hard-code
    TerrainOptions terrainOpt;
    terrainOpt.setDriver( "osgterrain" );
    _terrainEngine = TerrainEngineNodeFactory::create( terrainOpt );
    if ( _terrainEngine.valid() )
        this->addChild( _terrainEngine.get() );
    else
        OE_WARN << "FAILED to create a terrain engine for this map" << std::endl;

#if 0
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
#endif

    // install any pre-existing model layers:
    for( ModelLayerList::const_iterator k = _map->getModelLayers().begin(); k != _map->getModelLayers().end(); k++ )
    {
        onModelLayerAdded( k->get() );
    }

    // install any pre-existing mask layer:
    if ( _map->getTerrainMaskLayer() )
    {
        onMaskLayerAdded( _map->getTerrainMaskLayer() );
    }

    updateStateSet();

    // install a layer callback for processing further map actions:
    _map->addMapCallback( new MapNodeMapCallbackProxy(this) );

    osg::StateSet* ss = getOrCreateStateSet();
	//ss->setAttributeAndModes( new osg::CullFace() ); //, osg::StateAttribute::ON);
    //ss->setAttributeAndModes( new osg::PolygonOffset( -1, -1 ) );

    if ( _mapOptions.enableLighting().isSet() )
    {
        ss->setMode( GL_LIGHTING, _mapOptions.enableLighting().value() ? 
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
            osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    }
}

MapNode::~MapNode()
{
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

TerrainEngineNode*
MapNode::getTerrainEngine() const
{
    return _terrainEngine.get();
}

const MapOptions&
MapNode::getMapOptions() const
{
    return _mapOptions;
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

void
MapNode::onModelLayerAdded( ModelLayer* layer )
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
MapNode::onMaskLayerAdded( MaskLayer* layer )
{
    osg::Node* node = layer->getOrCreateNode();

    if ( node && node->asGroup() )
    {
        int count = 0;
        MaskNodeFinder f;
        node->accept( f );
        for( std::list<osg::Group*>::iterator i = f._groups.begin(); i != f._groups.end(); ++i )
        {
            (*i)->addChild( _terrainEngine );
            count++;
        }
        this->replaceChild( _terrainEngine, node );
        
        OE_NOTICE<<"Installed terrain mask ("
            <<count<< " mask nodes found)" << std::endl;

        _maskLayerNode = node->asGroup();
    }
}

void
MapNode::onMaskLayerRemoved( MaskLayer* layer )
{
    if ( layer && _maskLayerNode )
    {
        osg::ref_ptr<osg::Node> child = _maskLayerNode->getChild( 0 );
        this->replaceChild( _maskLayerNode, child.get() );
        _maskLayerNode = 0L;
    }
}

#endif // OSGEARTH2
