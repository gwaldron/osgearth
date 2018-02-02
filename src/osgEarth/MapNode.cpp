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
#include <osgEarth/TerrainResources>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/SpatialReference>
#include <osgEarth/MapModelChange>
#include <osgEarth/Lighting>
#include <osgEarth/ResourceReleaser>
#include <osgEarth/URI>
#include <osg/ArgumentParser>
#include <osg/PagedLOD>
#include <osgUtil/Optimizer>
#include <typeinfo>

using namespace osgEarth;

#define LC "[MapNode] "

//---------------------------------------------------------------------------

namespace
{
    /**
     * A group that the osgUtil::Optimizer won't remove even when it's empty.
     */
    struct StickyGroup : public osg::Group { };

    // adapter that lets MapNode listen to Map events
    struct MapNodeMapCallbackProxy : public MapCallback
    {
        MapNodeMapCallbackProxy(MapNode* node) : _node(node) { }

        void onLayerAdded(Layer* layer, unsigned index) {
            _node->onLayerAdded(layer, index);
            // for backwards compat until we refactor ModelLayer to use Layer::getNode
            MapCallback::onLayerAdded(layer, index);
        }
        void onLayerRemoved(Layer* layer, unsigned index) {
            _node->onLayerRemoved(layer, index);
            // for backwards compat until we refactor ModelLayer to use Layer::getNode
            MapCallback::onLayerRemoved(layer, index);
        }
        void onLayerMoved(Layer* layer, unsigned oldIndex, unsigned newIndex) {
            _node->onLayerMoved(layer, oldIndex, newIndex);
            MapCallback::onLayerMoved(layer, oldIndex, newIndex);
        }

        osg::observer_ptr<MapNode> _node;
    };

    // callback that will run the MapNode installer on model layers so that
    // MapNodeObservers can have MapNode access
    struct MapNodeObserverInstaller : public SceneGraphCallback
    {
        MapNodeObserverInstaller( MapNode* mapNode ) : _mapNode( mapNode ) { }

        virtual void onPostMergeNode(osg::Node* node)
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
_map( new Map() ),
_layerNodes(0L),
_terrainEngine(0L)
{
    init();
}

MapNode::MapNode( Map* map ) :
_map( map ),
_layerNodes(0L),
_terrainEngine(0L)
{
    init();
}

MapNode::MapNode( const MapNodeOptions& options ) :
_map( new Map() ),
_mapNodeOptions( options ),
_layerNodes(0L),
_terrainEngine(0L)
{
    init();
}

MapNode::MapNode( Map* map, const MapNodeOptions& options ) :
_map( map? map : new Map() ),
_mapNodeOptions( options ),
_layerNodes(0L),
_terrainEngine(0L)
{
    init();
}

MapNode::MapNode( Map* map, const MapNodeOptions& options, bool autoInit ) :
_map( map? map : new Map() ),
_mapNodeOptions( options ),
_layerNodes(0L),
_terrainEngine(0L)
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

    // Create and install a GL resource releaser that this node and extensions can use.
    _resourceReleaser = new ResourceReleaser();
    this->addChild(_resourceReleaser);

    // TODO: not sure why we call this here
    _map->setGlobalOptions( local_options.get() );

    // load and attach the terrain engine, but don't initialize it until we need it
    const TerrainOptions& terrainOptions = _mapNodeOptions.getTerrainOptions();

    _terrainEngine = TerrainEngineNodeFactory::create( terrainOptions );
    _terrainEngineInitialized = false;

    if ( _terrainEngine )
    {
        _terrainEngine->setMap( _map.get(), terrainOptions );
    }
    else
    {
        OE_WARN << "FAILED to create a terrain engine for this map" << std::endl;
    }

    // the engine needs a container so we can set lighting state on the container and
    // not on the terrain engine itself. Makeing it a StickyGroup prevents the osgUtil
    // Optimizer from collapsing it if it's empty
    _terrainEngineContainer = new StickyGroup();
    _terrainEngineContainer->setDataVariance( osg::Object::DYNAMIC );
    this->addChild( _terrainEngineContainer );

    // initialize terrain-level lighting:
    if ( terrainOptions.enableLighting().isSet() )
    {
        _terrainEngineContainer->getOrCreateStateSet()->setDefine(OE_LIGHTING_DEFINE, terrainOptions.enableLighting().get());

        _terrainEngineContainer->getOrCreateStateSet()->setMode(
            GL_LIGHTING,
            terrainOptions.enableLighting().value() ? 1 : 0 );
    }

    // a decorator for overlay models:
    _overlayDecorator = new OverlayDecorator();
    _terrainEngineContainer->addChild(_overlayDecorator);

    // install the Draping technique for overlays:
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

    draping->reestablish( _terrainEngine );
    _overlayDecorator->addTechnique( draping );
    _drapingManager = &draping->getDrapingManager();

    // install the Clamping technique for overlays:
    ClampingTechnique* clamping = new ClampingTechnique();
    _overlayDecorator->addTechnique(clamping);
    _clampingManager = &clamping->getClampingManager();

    _overlayDecorator->setTerrainEngine(_terrainEngine);
    _overlayDecorator->addChild(_terrainEngine);

    // make a group for the model layers. (Sticky otherwise the osg optimizer will remove it)
    _layerNodes = new StickyGroup();
    _layerNodes->setName( "osgEarth::MapNode.layerNodes" );
    this->addChild( _layerNodes );

    // Callback listens for changes in the Map:
    _mapCallback = new MapNodeMapCallbackProxy(this);
    _map->addMapCallback( _mapCallback.get() );

    // Simulate adding all existing layers:
    _mapCallback->invokeOnLayerAdded(_map.get());


    osg::StateSet* stateset = getOrCreateStateSet();
    stateset->setName("MapNode");

    if ( _mapNodeOptions.enableLighting().isSet() )
    {
        stateset->setDefine(OE_LIGHTING_DEFINE, terrainOptions.enableLighting().get());

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

    // install a default material for everything in the map
    osg::Material* defaultMaterial = new MaterialGL3();
    defaultMaterial->setDiffuse(defaultMaterial->FRONT, osg::Vec4(1,1,1,1));
    defaultMaterial->setAmbient(defaultMaterial->FRONT, osg::Vec4(1,1,1,1));
    stateset->setAttributeAndModes(defaultMaterial, 1);
    MaterialCallback().operator()(defaultMaterial, 0L);

    dirtyBound();

    // install a callback that sets the viewport size uniform:
    this->addCullCallback(new InstallViewportSizeUniform());

    // register for event traversals so we can deal with blacklisted filenames
    ADJUST_EVENT_TRAV_COUNT( this, 1 );

    // remove the temporary reference.
    this->unref_nodelete();
}

MapNode::~MapNode()
{
    _map->removeMapCallback( _mapCallback.get() );

    _mapCallback->invokeOnLayerRemoved(_map.get());
    //ModelLayerVector modelLayers;
    //_map->getLayers( modelLayers );
    ////Remove our model callback from any of the model layers in the map
    //for (osgEarth::ModelLayerVector::iterator itr = modelLayers.begin(); itr != modelLayers.end(); ++itr)
    //{
    //    this->onModelLayerRemoved( itr->get() );
    //}

    _map->clear();

    this->clearExtensions();

    osg::observer_ptr<TerrainEngineNode> te = _terrainEngine;
    removeChildren(0, getNumChildren());
    
    OE_DEBUG << LC << "~MapNode (TerrainEngine="
        << (te.valid()? te.get()->referenceCount() : 0) << ", Map=" << _map->referenceCount() << ")\n";
}

Config
MapNode::getConfig() const
{
    Config mapConf("map");
    mapConf.set("version", "2");

    MapFrame mapf( _map.get() );

    // the map and node options:
    Config optionsConf = _map->getInitialMapOptions().getConfig();
    optionsConf.merge( getMapNodeOptions().getConfig() );
    mapConf.add( "options", optionsConf );

    // the layers
    LayerVector layers;
    mapf.getLayers(layers);

    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        const Layer* layer = i->get();

        Config layerConf = layer->getConfig();
        if (!layerConf.empty() && !layerConf.key().empty())
        {
            mapConf.add(layerConf);
        }
    }

    typedef std::vector< osg::ref_ptr<Extension> > Extensions;
    for(Extensions::const_iterator i = getExtensions().begin(); i != getExtensions().end(); ++i)
    {
        Extension* e = i->get();
        Config conf = e->getConfigOptions().getConfig();
        if ( !conf.key().empty() )
        {
            mapConf.add( conf );
        }
    }

    Config ext = externalConfig();
    if ( !ext.empty() )
    {
        ext.key() = "external";
        mapConf.add( ext );
    }

    return mapConf;
}

osg::BoundingSphere
MapNode::computeBound() const
{
    osg::BoundingSphere bs;
    if ( getTerrainEngine() )
    {
        bs.expandBy( getTerrainEngine()->getBound() );
    }

    if (_layerNodes)
    {
        bs.expandBy( _layerNodes->getBound() );
    }

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
    return _terrainEngine;
}

ResourceReleaser*
MapNode::getResourceReleaser() const
{
    return _resourceReleaser;
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

        OE_INFO << LC << "Added extension \"" << extension->getName() << "\"\n";
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
MapNode::getLayerNodeGroup() const
{
    return _layerNodes;
}

osg::Node*
MapNode::getLayerNode(Layer* layer) const
{
    return layer ? layer->getOrCreateNode() : 0L;
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

namespace
{
    void rebuildLayerNodes(const Map* map, osg::Group* layerNodes)
    {
        layerNodes->removeChildren(0, layerNodes->getNumChildren());

        LayerVector layers;
        map->getLayers(layers);
        for (LayerVector::iterator i = layers.begin(); i != layers.end(); ++i)
        {
            Layer* layer = i->get();
            osg::Node* node = layer->getOrCreateNode();
            if (node)
            {
                osg::Group* container = new osg::Group();
                container->setName(layer->getName());
                container->addChild(node);
                container->setStateSet(layer->getStateSet());
                layerNodes->addChild(container);
            }
        }
    }
}

void
MapNode::onLayerAdded(Layer* layer, unsigned index)
{
    if (!layer || !layer->getEnabled())
        return;
    
    // Communicate terrain resources to the layer:
    layer->setTerrainResources(getTerrainEngine()->getResources());

    // Compatibility, until we refactor things.
    ModelLayer* modelLayer = dynamic_cast<ModelLayer*>(layer);
    if (modelLayer)
    {
        // TODO:  Why go through all the MapNodeObserver stuff when we can just pass in the MapNode here?
        modelLayer->getOrCreateSceneGraph(_map.get(), _map->getReadOptions(), 0L);
        // Install the MapNodeObserverInstaller so that MapNodeObservers will be notified of the MapNode.
        modelLayer->getSceneGraphCallbacks()->add(new MapNodeObserverInstaller(this));
    }

    // Create the layer's node, if it has one:
    osg::Node* node = layer->getOrCreateNode();
    if (node)
    {
        // Call setMapNode on any MapNodeObservers on this initial creation.
        MapNodeReplacer replacer( this );
        //node->accept( replacer );

        rebuildLayerNodes(_map.get(), _layerNodes);

        OE_DEBUG << LC << "Adding node from layer \"" << layer->getName() << "\" to the scene graph\n";

        // TODO: move this logic into ModelLayer.
        if (modelLayer)
        {
            ModelSource* ms = modelLayer->getModelSource();
            if (ms)
            {
                // enfore a rendering bin if necessary:
                if (ms->getOptions().renderOrder().isSet())
                {
                    osg::StateSet* mss = node->getOrCreateStateSet();
                    mss->setRenderBinDetails(
                        ms->getOptions().renderOrder().value(),
                        mss->getBinName().empty() ? "DepthSortedBin" : mss->getBinName());
                }
                if (ms->getOptions().renderBin().isSet())
                {
                    osg::StateSet* mss = node->getOrCreateStateSet();
                    mss->setRenderBinDetails(
                        mss->getBinNumber(),
                        ms->getOptions().renderBin().get());
                }
            }
        }
    }
}

void
MapNode::onLayerRemoved(Layer* layer, unsigned index)
{
    if (layer && layer->getOrCreateNode())
    {
        rebuildLayerNodes(_map.get(), _layerNodes);
    }
}

void
MapNode::onLayerMoved(Layer* layer, unsigned oldIndex, unsigned newIndex)
{
    if (layer && layer->getOrCreateNode())
    {
        rebuildLayerNodes(_map.get(), _layerNodes);
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

    for (LayerVector::const_iterator i = frame.layers().begin();
        i != frame.layers().end();
        ++i)
    {
        tryOpenLayer(i->get());
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

    else
    {
        if (dynamic_cast<osgUtil::BaseOptimizerVisitor*>(&nv) == 0L)
            osg::Group::traverse( nv );
    }
}

void
MapNode::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Group::resizeGLObjectBuffers(maxSize);

    LayerVector layers;
    getMap()->getLayers(layers);
    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        if ((*i)->getStateSet()) {
            (*i)->getStateSet()->resizeGLObjectBuffers(maxSize);
        }
    }
}

void
MapNode::releaseGLObjects(osg::State* state) const
{
    osg::Group::releaseGLObjects(state);

    LayerVector layers;
    getMap()->getLayers(layers);
    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        if ((*i)->getStateSet()) {
            (*i)->getStateSet()->releaseGLObjects(state);
        }
    }
}

DrapingManager*
MapNode::getDrapingManager()
{
    return _drapingManager;
}

ClampingManager*
MapNode::getClampingManager()
{
    return _clampingManager;
}
