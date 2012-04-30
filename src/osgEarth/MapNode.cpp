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
#include <osgEarth/MapNode>
#include <osgEarth/MaskNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TextureCompositor>
#include <osgEarth/URI>
#include <osgEarth/DrapeableNode>
#include <osg/ArgumentParser>
#include <osg/PagedLOD>
#include <osgSim/OverlayNode>

using namespace osgEarth;

#define LC "[MapNode] "

//---------------------------------------------------------------------------

namespace 
{
    // adapter that lets MapNode listen to Map events
    struct MapNodeMapCallbackProxy : public MapCallback
    {
        MapNodeMapCallbackProxy(MapNode* node) : _node(node) { }

        void onModelLayerAdded( ModelLayer* layer, unsigned int index ) {
            _node->onModelLayerAdded( layer, index );
        }
        void onModelLayerRemoved( ModelLayer* layer ) {
            _node->onModelLayerRemoved( layer );
        }
        void onModelLayerMoved( ModelLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
            _node->onModelLayerMoved( layer, oldIndex, newIndex);
        }
#if 0
        void onMaskLayerAdded( MaskLayer* layer ) {
            _node->onMaskLayerAdded( layer );
        }
        void onMaskLayerRemoved( MaskLayer* layer ) {
            _node->onMaskLayerRemoved( layer );
        }
#endif

        osg::observer_ptr<MapNode> _node;
    };

    // converys overlay property changes to the OverlayDecorator in MapNode.
    class MapModelLayerCallback : public ModelLayerCallback
    {
    public:
        MapModelLayerCallback(MapNode* mapNode) : _node(mapNode) { }

        virtual void onOverlayChanged(ModelLayer* layer)
        {
            _node->onModelLayerOverlayChanged( layer );
        }

        osg::observer_ptr<MapNode> _node;
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
          //The PagedLOD node will contain two filenames, the first is empty and is the actual geometry of the          
          //tile and the second is the filename of the next tile.
          if (node.getNumFileNames() > 1)
          {
              //Get the child filename
              const std::string &filename = node.getFileName(1);              
              if (osgEarth::Registry::instance()->isBlacklisted(filename))
              {
                  //If the tile is blacklisted, we set the actual geometry, child 0, to always display
                  //and the second child to never display
                  node.setRange(0, 0, FLT_MAX);
                  node.setRange(1, FLT_MAX, FLT_MAX);
              }
              else
              {
                  //If the child is not blacklisted, it is possible that it could have been blacklisted previously so reset the
                  //ranges of both the first and second children.  This gives the second child another
                  //chance to be traversed in case a layer was added that might have data.
                  osg::ref_ptr< MapNode::TileRangeData > ranges = static_cast< MapNode::TileRangeData* >(node.getUserData());
                  if (ranges)
                  {
                      node.setRange(0, ranges->_minRange, ranges->_maxRange);
                      node.setRange(1, 0, ranges->_minRange);
                  }                  
              }
              
          }
          traverse(node);
      }

      unsigned int _numRemoved;
};

//---------------------------------------------------------------------------

MapNode*
MapNode::load(osg::ArgumentParser& args)
{
    for( int i=1; i<args.argc(); ++i )
    {
        if ( args[i] && endsWith(args[i], ".earth") )
        {
            ReadResult r = URI(args[i]).readNode();
            if ( r.succeeded() )
            {
                return r.release<MapNode>();
            }
        }
    }    
    return 0L;
}

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

MapNode::MapNode( const MapNodeOptions& options ) :
_map( new Map() ),
_mapNodeOptions( options )
{
    init();
}

MapNode::MapNode( Map* map, const MapNodeOptions& options ) :
_map( map? map : new Map() ),
_mapNodeOptions( options )
{
    init();
}

void
MapNode::init()
{
    // Protect the MapNode from the Optimizer
    setDataVariance(osg::Object::DYNAMIC);

    // initialize 0Ls
    _terrainEngine          = 0L;
    _terrainEngineContainer = 0L;
    _overlayDecorator       = 0L;

    setName( "osgEarth::MapNode" );

    // Since we have global uniforms in the stateset, mark it dynamic so it is immune to
    // multi-threaded overlap
    // TODO: do we need this anymore? there are no more global uniforms in here.. gw
    getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);

    _modelLayerCallback = new MapModelLayerCallback(this);

    _maskLayerNode = 0L;
    _lastNumBlacklistedFilenames = 0;

    // Set the global proxy settings
    // TODO: this should probably happen elsewhere, like in the registry?
    if ( _mapNodeOptions.proxySettings().isSet() )
    {
        HTTPClient::setProxySettings( _mapNodeOptions.proxySettings().get() );
    }

    // establish global driver options. These are OSG reader-writer options that
    // will make their way to any read* calls down the pipe
    const osgDB::Options* global_options = _map->getGlobalOptions();
    osg::ref_ptr<osgDB::Options> local_options = global_options ? 
        Registry::instance()->cloneOrCreateOptions( global_options ) :
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

    // load and attach the terrain engine, but don't initialize it until we need it
    const TerrainOptions& terrainOptions = _mapNodeOptions.getTerrainOptions();

    _terrainEngine = TerrainEngineNodeFactory::create( _map.get(), terrainOptions );
    _terrainEngineInitialized = false;

    // the engine needs a container so we can set lighting state on the container and
    // not on the terrain engine itself. Setting the dynamic variance will prevent
    // an optimizer from collapsing the empty group node.
    _terrainEngineContainer = new osg::Group();
    _terrainEngineContainer->setDataVariance( osg::Object::DYNAMIC );
    this->addChild( _terrainEngineContainer );

    // initialize terrain-level lighting:
    if ( terrainOptions.enableLighting().isSet() )
    {
        _terrainEngineContainer->getOrCreateStateSet()->setMode( GL_LIGHTING, terrainOptions.enableLighting().value() ? 
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
            osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    }

    if ( _terrainEngine )
    {
        // inform the terrain engine of the map information now so that it can properly
        // initialize it's CoordinateSystemNode. This is necessary in order to support
        // manipulators and to set up the texture compositor prior to frame-loop 
        // initialization.
        _terrainEngine->preInitialize( _map.get(), terrainOptions );
        _terrainEngineContainer->addChild( _terrainEngine );
    }
    else
    {
        OE_WARN << "FAILED to create a terrain engine for this map" << std::endl;
    }

    // make a group for the model layers.
    // NOTE: for now, we are going to nullify any shader programs that occur above the model
    // group, since it does not YET support shader composition. Programs defined INSIDE a
    // model layer will still work OK though.
    _models = new osg::Group();
    _models->setName( "osgEarth::MapNode.modelsGroup" );
    _models->getOrCreateStateSet()->setAttributeAndModes( new osg::Program(), osg::StateAttribute::OFF );
    addChild( _models.get() );

    // make a group for overlay model layers:
    _overlayModels = new osg::Group();
    _overlayModels->setName( "osgEarth::MapNode.overlayModelsGroup" );

    // a decorator for overlay models:
    _overlayDecorator = new OverlayDecorator();
    if ( _mapNodeOptions.overlayBlending().isSet() )
        _overlayDecorator->setOverlayBlending( *_mapNodeOptions.overlayBlending() );
    if ( _mapNodeOptions.overlayTextureSize().isSet() )
        _overlayDecorator->setTextureSize( *_mapNodeOptions.overlayTextureSize() );
    if ( _mapNodeOptions.overlayMipMapping().isSet() )
        _overlayDecorator->setMipMapping( *_mapNodeOptions.overlayMipMapping() );

    addTerrainDecorator( _overlayDecorator );

    // install any pre-existing model layers:
    ModelLayerVector modelLayers;
    _map->getModelLayers( modelLayers );
    int modelLayerIndex = 0;
    for( ModelLayerVector::const_iterator k = modelLayers.begin(); k != modelLayers.end(); k++, modelLayerIndex++ )
    {
        onModelLayerAdded( k->get(), modelLayerIndex );
    }

    _mapCallback = new MapNodeMapCallbackProxy(this);
    // install a layer callback for processing further map actions:
    _map->addMapCallback( _mapCallback.get()  );

    osg::StateSet* ss = getOrCreateStateSet();

    if ( _mapNodeOptions.enableLighting().isSet() )
    {
        ss->setMode( GL_LIGHTING, _mapNodeOptions.enableLighting().value() ? 
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
            osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    }

    dirtyBound();

    // register for event traversals so we can deal with blacklisted filenames
    ADJUST_EVENT_TRAV_COUNT( this, 1 );
}

MapNode::~MapNode()
{
    _map->removeMapCallback( _mapCallback.get() );

    ModelLayerVector modelLayers;
    _map->getModelLayers( modelLayers );
    //Remove our model callback from any of the model layers in the map    
    for (osgEarth::ModelLayerVector::iterator itr = modelLayers.begin(); itr != modelLayers.end(); ++itr)
    {
        this->onModelLayerRemoved( itr->get() );        
    }
}

osg::BoundingSphere
MapNode::computeBound() const
{
    osg::BoundingSphere bs;
    if ( getTerrainEngine() )
    {
        bs.expandBy(  getTerrainEngine()->getBound() );
    }
    if ( _models.valid() )
        bs.expandBy( _models->getBound() );
    return bs;
}

Map*
MapNode::getMap()
{
    return _map.get();
}

const Map*
MapNode::getMap() const
{
    return _map.get();
}

const SpatialReference*
MapNode::getMapSRS() const
{
    return getMap()->getProfile()->getSRS();
}

Terrain*
MapNode::getTerrain()
{
    return getTerrainEngine()->getTerrain();
}

TerrainEngineNode*
MapNode::getTerrainEngine() const
{
    if ( !_terrainEngineInitialized && _terrainEngine )
    {
        _terrainEngine->postInitialize( _map.get(), getMapNodeOptions().getTerrainOptions() );
        MapNode* me = const_cast< MapNode* >(this);
        me->_terrainEngineInitialized = true;
        me->dirtyBound();
    }
    return _terrainEngine;
}

void
MapNode::setCompositorTechnique( TextureCompositorTechnique* tech )
{
    if ( _terrainEngine )
    {
        _terrainEngine->getTextureCompositor()->setTechnique( tech );
    }
}

osg::Group*
MapNode::getModelLayerGroup() const
{
    return _models.get();
}

const MapNodeOptions&
MapNode::getMapNodeOptions() const
{
    return _mapNodeOptions;
}

MapNode*
MapNode::findMapNode( osg::Node* graph, unsigned travmask )
{
    return findRelativeNodeOfType<MapNode>( graph, travmask );
}

bool
MapNode::isGeocentric() const
{
    return _map->isGeocentric();
}

void
MapNode::onModelLayerAdded( ModelLayer* layer, unsigned int index )
{
    if ( !layer->getEnabled() )
        return;

    osg::Node* node = layer->getOrCreateNode();

    layer->addCallback(_modelLayerCallback.get() );

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
            if ( dynamic_cast<TerrainDecorator*>(node) || dynamic_cast<osgSim::OverlayNode*>(node) )
            {
                OE_INFO << LC << "Installing overlay node" << std::endl;
                addTerrainDecorator( node->asGroup() );
            }
            else
            {
                if ( layer->getOverlay() )
                {
#if 0
                    _overlayModels->addChild( node );
                    updateOverlayGraph();
#else
                    DrapeableNode* draper = new DrapeableNode( this );
                    draper->addChild( node );
                    _models->insertChild( index, draper );
#endif
                }
                else
                {
                    _models->insertChild( index, node );
                }
            }

            ModelSource* ms = layer->getModelSource();
            if ( ms && ms->getOptions().renderOrder().isSet() )
            {
                node->getOrCreateStateSet()->setRenderBinDetails(
                    ms->getOptions().renderOrder().value(), "RenderBin" );
            }

            _modelLayerNodes[ layer ] = node;
        }

        dirtyBound();
    }
}

void
MapNode::onModelLayerRemoved( ModelLayer* layer )
{
    if ( layer )
    {
        layer->removeCallback( _modelLayerCallback.get() );

        // look up the node associated with this model layer.
        ModelLayerNodeMap::iterator i = _modelLayerNodes.find( layer );
        if ( i != _modelLayerNodes.end() )
        {
            osg::Node* node = i->second;

            if ( dynamic_cast<TerrainDecorator*>( node ) || dynamic_cast<osgSim::OverlayNode*>( node ) )
            {
                removeTerrainDecorator( node->asGroup() );
            }
            else
            {
                if ( layer->getModelLayerOptions().overlay() == true )
                {
                    _overlayModels->removeChild( node );
                    updateOverlayGraph();
                }
                else
                {
                    _models->removeChild( node );
                }
            }
            
            _modelLayerNodes.erase( i );
        }
        
        dirtyBound();
    }
}

void
MapNode::onModelLayerMoved( ModelLayer* layer, unsigned int oldIndex, unsigned int newIndex )
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
                // treat overlay node as a special case
            }
            else
            {
                _models->removeChild( node );
                _models->insertChild( newIndex, node );
            }
        }
        
        dirtyBound();
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
MapNode::addTerrainDecorator(osg::Group* decorator)
{    
    if ( _terrainEngine )
    {
        decorator->addChild( _terrainEngine );
        _terrainEngine->getParent(0)->replaceChild( _terrainEngine, decorator );
        dirtyBound();

        TerrainDecorator* td = dynamic_cast<TerrainDecorator*>( decorator );
        if ( td )
            td->onInstall( _terrainEngine );
    }
}

void
MapNode::removeTerrainDecorator(osg::Group* decorator)
{
    if ( _terrainEngine )
    {
        TerrainDecorator* td = dynamic_cast<TerrainDecorator*>( decorator );
        if ( td )
            td->onUninstall( _terrainEngine );

        osg::ref_ptr<osg::Node> child = _terrainEngine;
        for( osg::Group* g = child->getParent(0); g != _terrainEngineContainer; )
        {
            if ( g == decorator )
            {
                g->getParent(0)->replaceChild( g, child );
                g->removeChild( child );
                break;
            }
            child = g;
            g = g->getParent(0);
        }
        dirtyBound();
    }
}

void
MapNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::EVENT_VISITOR )
    {
        unsigned int numBlacklist = Registry::instance()->getNumBlacklistedFilenames();
        if (numBlacklist != _lastNumBlacklistedFilenames)
        {
            //Only remove the blacklisted filenames if new filenames have been added since last time.
            _lastNumBlacklistedFilenames = numBlacklist;
            RemoveBlacklistedFilenamesVisitor v;
            //accept( v );
            _terrainEngine->accept( v );
        }
    }

    osg::Group::traverse( nv );
}

void
MapNode::onModelLayerOverlayChanged( ModelLayer* layer )
{
    OE_NOTICE << "Overlay changed to "  << layer->getOverlay() << std::endl;
    osg::ref_ptr< osg::Group > origParent = layer->getOverlay() ? _models.get() : _overlayModels.get();
    osg::ref_ptr< osg::Group > newParent  = layer->getOverlay() ? _overlayModels.get() : _models.get();

    osg::ref_ptr< osg::Node > node = layer->getOrCreateNode();
    if (node.valid())
    {
        //Remove it from the original parent and add it to the new parent
        origParent->removeChild( node.get() );
        newParent->addChild( node.get() );
    }

    updateOverlayGraph();
}

void
MapNode::updateOverlayGraph()
{
    if ( _overlayModels->getNumChildren() > 0 && _overlayDecorator->getOverlayGraph() != _overlayModels.get() )
    {
        _overlayDecorator->setOverlayGraph( _overlayModels.get() );
    }
    else if ( _overlayModels->getNumChildren() == 0 && _overlayDecorator->getOverlayGraph() == _overlayModels.get() )
    {
        _overlayDecorator->setOverlayGraph( 0L );
    }
}

