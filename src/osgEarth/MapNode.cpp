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
#include <osgEarth/MapNode>
#include <osgEarth/Capabilities>
#include <osgEarth/ClampableNode>
#include <osgEarth/ClampingTechnique>
#include <osgEarth/CullingUtils>
#include <osgEarth/DrapeableNode>
#include <osgEarth/DrapingTechnique>
#include <osgEarth/MapNodeObserver>
#include <osgEarth/MaskNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderFactory>
#include <osgEarth/TraversalData>
#include <osgEarth/VirtualProgram>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TextureCompositor>
#include <osgEarth/URI>
#include <osg/ArgumentParser>
#include <osg/PagedLOD>

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

        osg::observer_ptr<MapNode> _node;
    };

    // callback that will run the MapNode installer on model layers so that
    // MapNodeObservers can have MapNode access
    struct MapNodeObserverInstaller : public NodeOperation
    {
        MapNodeObserverInstaller( MapNode* mapNode ) : _mapNode( mapNode ) { }

        void operator()( osg::Node* node )
        {
            if ( _mapNode.valid() && node )
            {
                MapNodeReplacer replacer( _mapNode.get() );
                node->accept( replacer );
            }
        }

        osg::observer_ptr<MapNode> _mapNode;
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
                      if (node.getRangeMode() == osg::LOD::PIXEL_SIZE_ON_SCREEN)
                      {
                          node.setRange( 0, ranges->_minRange, ranges->_maxRange );
                          node.setRange( 1, ranges->_maxRange, FLT_MAX );                          
                      }
                      else
                      {                          
                          node.setRange(0, ranges->_minRange, ranges->_maxRange);
                          node.setRange(1, 0, ranges->_minRange);
                      }
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

MapNode::MapNode( Map* map, const MapNodeOptions& options, bool autoInit ) :
_map( map? map : new Map() ),
_mapNodeOptions( options )
{
    if ( autoInit )
    {
        init();
    }
}


void
MapNode::init()
{
    // Take a reference to this object so that it doesn't get inadvertently
    // deleting during startup. It is possible that during startup, a driver
    // will load that will take a reference to the MapNode (like in a
    // ModelSource node operation) and we don't want that deleting the MapNode
    // out from under us. 
    // This is paired by an unref_nodelete() at the end of this method.
    this->ref();

    // Protect the MapNode from the Optimizer
    setDataVariance(osg::Object::DYNAMIC);

    // initialize 0Ls
    _terrainEngine          = 0L;
    _terrainEngineContainer = 0L;
    _overlayDecorator       = 0L;

    setName( "osgEarth::MapNode" );

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
        //_terrainEngineContainer->getOrCreateStateSet()->setMode( 
        //    GL_LIGHTING, 
        //    terrainOptions.enableLighting().value() ? 1 : 0 );

        _terrainEngineContainer->getOrCreateStateSet()->addUniform(
            Registry::shaderFactory()->createUniformForGLMode(GL_LIGHTING, *terrainOptions.enableLighting()) );
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
    addChild( _models.get() );

    // a decorator for overlay models:
    _overlayDecorator = new OverlayDecorator();
    _overlayDecorator->setOverlayGraphTraversalMask( terrainOptions.secondaryTraversalMask().value() );

    // install the Draping technique for overlays:
    {
        DrapingTechnique* draping = new DrapingTechnique();

        const char* envOverlayTextureSize = ::getenv("OSGEARTH_OVERLAY_TEXTURE_SIZE");

        if ( _mapNodeOptions.overlayBlending().isSet() )
            draping->setOverlayBlending( *_mapNodeOptions.overlayBlending() );
        if ( envOverlayTextureSize )
            draping->setTextureSize( as<int>(envOverlayTextureSize, 1024) );
        else if ( _mapNodeOptions.overlayTextureSize().isSet() )
            draping->setTextureSize( *_mapNodeOptions.overlayTextureSize() );
        if ( _mapNodeOptions.overlayMipMapping().isSet() )
            draping->setMipMapping( *_mapNodeOptions.overlayMipMapping() );
        if ( _mapNodeOptions.overlayAttachStencil().isSet() )
            draping->setAttachStencil( *_mapNodeOptions.overlayAttachStencil() );
        if ( _mapNodeOptions.overlayResolutionRatio().isSet() )
            draping->setResolutionRatio( *_mapNodeOptions.overlayResolutionRatio() );

        _overlayDecorator->addTechnique( draping );
    }

    // install the Clamping technique for overlays:
    {
        _overlayDecorator->addTechnique( new ClampingTechnique() );
    }


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

    osg::StateSet* stateset = getOrCreateStateSet();
    if ( _mapNodeOptions.enableLighting().isSet() )
    {
        stateset->setMode( 
            GL_LIGHTING, 
            _mapNodeOptions.enableLighting().value() ? 1 : 0 );
    }

    dirtyBound();

    // Install a default lighting shader program.
    if ( Registry::capabilities().supportsGLSL() )
    {
        VirtualProgram* vp = VirtualProgram::getOrCreate( stateset );
        vp->setName( "osgEarth::MapNode" );

        Registry::shaderFactory()->installLightingShaders( vp );
    }

    // register for event traversals so we can deal with blacklisted filenames
    ADJUST_EVENT_TRAV_COUNT( this, 1 );

    // remove the temporary reference.
    this->unref_nodelete();
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

const Terrain*
MapNode::getTerrain() const
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

osg::Node*
MapNode::getModelLayerNode( ModelLayer* layer ) const
{
    ModelLayerNodeMap::const_iterator i = _modelLayerNodes.find( layer );
    return i != _modelLayerNodes.end() ? i->second : 0L;
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
    
    // install a noe operation that will associate this mapnode with
    // any MapNodeObservers loaded by the model layer:
    ModelSource* modelSource = layer->getModelSource();
    if ( modelSource )
    {
        // install a post-processing callback on the ModelLayer's source 
        // so we can update the MapNode on new data that comes in:
        modelSource->addPostProcessor( new MapNodeObserverInstaller(this) );
    }

    // create the scene graph:
    osg::Node* node = layer->createSceneGraph( _map.get(), _map->getDBOptions(), 0L );

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
            if ( dynamic_cast<TerrainDecorator*>(node) )
            {
                OE_INFO << LC << "Installing overlay node" << std::endl;
                addTerrainDecorator( node->asGroup() );
            }
            else
            {
                _models->insertChild( index, node );
            }

            ModelSource* ms = layer->getModelSource();

            if ( ms )
            {
                // enfore a rendering bin if necessary:
                if ( ms->getOptions().renderOrder().isSet() )
                {
                    node->getOrCreateStateSet()->setRenderBinDetails(
                        ms->getOptions().renderOrder().value(), "RenderBin" );
                }
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
        // look up the node associated with this model layer.
        ModelLayerNodeMap::iterator i = _modelLayerNodes.find( layer );
        if ( i != _modelLayerNodes.end() )
        {
            osg::Node* node = i->second;

            if ( dynamic_cast<TerrainDecorator*>( node ) )
            {
                removeTerrainDecorator( node->asGroup() );
            }
            else
            {
                _models->removeChild( node );
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
            osg::ref_ptr<osg::Node> node = i->second;

            _models->removeChild( node.get() );
            _models->insertChild( newIndex, node.get() );
        }
        
        dirtyBound();
    }
}

namespace
{
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
}

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
    if ( nv.getVisitorType() == nv.EVENT_VISITOR )
    {
        unsigned int numBlacklist = Registry::instance()->getNumBlacklistedFilenames();
        if (numBlacklist != _lastNumBlacklistedFilenames)
        {
            //Only remove the blacklisted filenames if new filenames have been added since last time.
            _lastNumBlacklistedFilenames = numBlacklist;
            RemoveBlacklistedFilenamesVisitor v;
            _terrainEngine->accept( v );
        }

        // traverse:
        std::for_each( _children.begin(), _children.end(), osg::NodeAcceptOp(nv) );
    }

    else if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // update the light model uniforms.
        _updateLightingUniformsHelper.cullTraverse( this, &nv );

        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        if ( cv )
        {
            // insert traversal data for this camera:
            osg::ref_ptr<osg::Referenced> oldUserData = cv->getUserData();
            MapNodeCullData* cullData = getCullData( cv->getCurrentCamera() );
            cv->setUserData( cullData );

            cullData->_mapNode = this;

            // calculate altitude:
            osg::Vec3d eye = cv->getViewPoint();
            const SpatialReference* srs = getMapSRS();
            if ( srs && !srs->isProjected() )
            {
                GeoPoint ecef;
                ecef.fromWorld( srs, eye );
                cullData->_cameraAltitude = ecef.alt();
            }
            else
            {
                cullData->_cameraAltitude = eye.z();
            }

            // window scale matrix:
            osg::Matrix  m4 = cv->getWindowMatrix();
            osg::Matrix3 m3( m4(0,0), m4(1,0), m4(2,0),
                             m4(0,1), m4(1,1), m4(2,1),
                             m4(0,2), m4(1,2), m4(2,2) );
            cullData->_windowScaleMatrixUniform->set( m3 );

            // traverse:
            cv->pushStateSet( cullData->_stateSet.get() );
            std::for_each( _children.begin(), _children.end(), osg::NodeAcceptOp(nv) );
            cv->popStateSet();

            // restore:
            cv->setUserData( oldUserData.get() );
        }
    }

    else
    {
        osg::Group::traverse( nv );
    }
}

MapNodeCullData*
MapNode::getCullData(osg::Camera* key) const
{
    // first look it up:
    {
        Threading::ScopedReadLock shared( _cullDataMutex );
        CullDataMap::const_iterator i = _cullData.find( key );
        if ( i != _cullData.end() )
            return i->second.get();
    }
    // if it's not there, make it, but double-check.
    {
        Threading::ScopedWriteLock exclusive( _cullDataMutex );
        
        CullDataMap::const_iterator i = _cullData.find( key );
        if ( i != _cullData.end() )
            return i->second.get();

        MapNodeCullData* cullData = new MapNodeCullData();
        _cullData[key] = cullData;
        return cullData;
    }
}
