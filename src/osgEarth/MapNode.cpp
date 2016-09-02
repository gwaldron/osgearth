/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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
#include <osgEarth/ShaderGenerator>
#include <osgEarth/SpatialReference>
#include <osgEarth/MapModelChange>
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

    typedef std::vector< osg::ref_ptr<Extension> > Extensions;
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

MapNode*
MapNode::load(osg::ArgumentParser& args, const MapNodeOptions& defaults)
{
    for( int i=1; i<args.argc(); ++i )
    {
        if ( args[i] && endsWith(args[i], ".earth") )
        {
            osg::ref_ptr<osgDB::Options> dbo = new osgDB::Options();
            std::string optionsJSON = defaults.getConfig().toJSON();
            dbo->setPluginStringData( "osgEarth.defaultOptions", optionsJSON );
            ReadResult r = URI(args[i]).readNode( dbo.get() );
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

    // Protect the MapNode from the ShaderGenerator
    ShaderGenerator::setIgnoreHint(this, true);

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
        _terrainEngineContainer->getOrCreateStateSet()->addUniform(
            Registry::shaderFactory()->createUniformForGLMode(GL_LIGHTING, *terrainOptions.enableLighting()) );

        _terrainEngineContainer->getOrCreateStateSet()->setMode(
            GL_LIGHTING,
            terrainOptions.enableLighting().value() ? 1 : 0 );
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
    //_models->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
    addChild( _models.get() );

    // a decorator for overlay models:
    _overlayDecorator = new OverlayDecorator();

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

        draping->reestablish( getTerrainEngine() );
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
        stateset->addUniform(Registry::shaderFactory()->createUniformForGLMode(
            GL_LIGHTING,
            _mapNodeOptions.enableLighting().value() ? 1 : 0));

        stateset->setMode(
            GL_LIGHTING,
            _mapNodeOptions.enableLighting().value() ? 1 : 0);
    }

    // Add in some global uniforms
    stateset->addUniform( new osg::Uniform("oe_isGeocentric", _map->isGeocentric()) );
    if ( _map->isGeocentric() )
    {
        OE_INFO << LC << "Adding ellipsoid uniforms.\n";

        // for a geocentric map, use an ellipsoid unit-frame transform and its inverse:
        if (_map->getSRS() != NULL && _map->getSRS()->getEllipsoid() != NULL)
        {
            osg::Vec3d ellipFrameInverse(
                _map->getSRS()->getEllipsoid()->getRadiusEquator(),
                _map->getSRS()->getEllipsoid()->getRadiusEquator(),
                _map->getSRS()->getEllipsoid()->getRadiusPolar());
            stateset->addUniform( new osg::Uniform("oe_ellipsoidFrameInverse", osg::Vec3f(ellipFrameInverse)) );

            osg::Vec3d ellipFrame = osg::componentDivide(osg::Vec3d(1.0,1.0,1.0), ellipFrameInverse);
            stateset->addUniform( new osg::Uniform("oe_ellipsoidFrame", osg::Vec3f(ellipFrame)) );
        }
        else
        {
            stateset->addUniform( new osg::Uniform("oe_ellipsoidFrameInverse", osg::Vec3f()) );
            stateset->addUniform( new osg::Uniform("oe_ellipsoidFrame", osg::Vec3f()) );
        }
    }

    // install the default rendermode uniform:
    stateset->addUniform( new osg::Uniform("oe_isPickCamera", false) );
    stateset->addUniform( new osg::Uniform("oe_isShadowCamera", false) );

    dirtyBound();

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

    this->clearExtensions();
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
    if (getTerrainEngine() == NULL)
          return NULL;
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
MapNode::addExtension(Extension* extension, const osgDB::Options* options)
{
    if ( extension )
    {
        _extensions.push_back( extension );

        // set the IO options is they were provided:
        if ( options )
            extension->setDBOptions( options );

        else if ( getMap()->getReadOptions() )
            extension->setDBOptions( getMap()->getReadOptions() );

        // start it.
        ExtensionInterface<MapNode>* extensionIF = ExtensionInterface<MapNode>::get(extension);
        if ( extensionIF )
        {
            extensionIF->connect( this );
        }

        OE_INFO << LC << "Added extension [" << extension->getName() << "]\n";
    }
}

void
MapNode::removeExtension(Extension* extension)
{
    Extensions::iterator i = std::find(_extensions.begin(), _extensions.end(), extension);
    if ( i != _extensions.end() )
    {
        ExtensionInterface<MapNode>* extensionIF = ExtensionInterface<MapNode>::get( i->get() );
        if ( extensionIF )
        {
            extensionIF->disconnect( this );
        }
        _extensions.erase( i );
    }
}

void
MapNode::clearExtensions()
{
    for(Extensions::iterator i = _extensions.begin(); i != _extensions.end(); ++i)
    {
        ExtensionInterface<MapNode>* extensionIF = ExtensionInterface<MapNode>::get( i->get() );
        if ( extensionIF )
        {
            extensionIF->disconnect( this );
        }
    }

    _extensions.clear();
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
        modelSource->addPostMergeOperation( new MapNodeObserverInstaller(this) );
    }

    // create the scene graph:
    osg::Node* node = layer->getOrCreateSceneGraph( _map.get(), _map->getReadOptions(), 0L );

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
                    osg::StateSet* mss = node->getOrCreateStateSet();
                    mss->setRenderBinDetails(
                        ms->getOptions().renderOrder().value(),
                        mss->getBinName().empty() ? "DepthSortedBin" : mss->getBinName());
                }
                if ( ms->getOptions().renderBin().isSet() )
                {
                    osg::StateSet* mss = node->getOrCreateStateSet();
                    mss->setRenderBinDetails(
                        mss->getBinNumber(),
                        ms->getOptions().renderBin().get() );
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

namespace
{
    template<typename T> void tryOpenLayer(T* layer)
    {
        if (!layer->getStatus().isError())
        {
            const Status& status = layer->open();
            if (status.isError())
            {
                OE_WARN << LC << "Failed to open layer \"" << layer->getName() << "\" ... " << status.message() << std::endl;
            }
        }
    }
}

void
MapNode::openMapLayers()
{
    MapFrame frame(_map.get());

    for (unsigned i = 0; i < frame.imageLayers().size(); ++i)
    {
        tryOpenLayer(frame.getImageLayerAt(i));
    }

    for (unsigned i = 0; i < frame.elevationLayers().size(); ++i)
    {
        tryOpenLayer(frame.getElevationLayerAt(i));
    }

    for (unsigned i = 0; i < frame.modelLayers().size(); ++i)
    {
        tryOpenLayer(frame.getModelLayerAt(i));
    }

    for (unsigned i = 0; i < frame.terrainMaskLayers().size(); ++i)
    {
        tryOpenLayer(frame.terrainMaskLayers().at(i).get());
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

        // This is a placeholder for later, if we decide to delay the automatic opening of
        // layers until the first traversal.
#if 0
        // if the model has changed, we need to queue up an update so we can open new layers.
        if (_mapRevisionMonitor.outOfSyncWith(_map->getDataModelRevision()))
        {
            openMapLayers();
            _mapRevisionMonitor.sync(_map->getDataModelRevision());
        }
#endif

        // traverse:
        std::for_each( _children.begin(), _children.end(), osg::NodeAcceptOp(nv) );
    }

    else if (nv.getVisitorType() == nv.UPDATE_VISITOR || nv.getVisitorType() == nv.CULL_VISITOR)
    {
        // put the MapNode in the visitor data and traverse.
        VisitorData::store(nv, "osgEarth::MapNode", this);
        std::for_each( _children.begin(), _children.end(), osg::NodeAcceptOp(nv) );
    }

    else
    {
        osg::Group::traverse( nv );
    }
}
