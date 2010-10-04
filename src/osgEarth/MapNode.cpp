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
#include <osgEarth/FindNode>
#include <osgEarth/Registry>

using namespace osgEarth;

#define LC "[MapNode] "

//---------------------------------------------------------------------------

// adapter that lets MapNode listen to Map events
struct MapNodeMapCallbackProxy : public MapCallback
{
    MapNodeMapCallbackProxy(MapNode* node) : _node(node) { }

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

    osg::observer_ptr<MapNode> _node;
};

//---------------------------------------------------------------------------

#if 0
namespace osgEarth
{
    struct MapNodeMapLayerController : public MapLayerController
    {
        MapNodeMapLayerController( MapNode* mapNode ) : _node(mapNode) { }

        void updateOpacity( MapLayer* layer ) 
        {
            _node->getTerrainEngine()->updateLayerOpacity( layer );
        }

        void updateEnabled( MapLayer* layer)
        {
            _node->getTerrainEngine()->updateLayerEnabled( layer );
        }

    private:
        MapNode* _node;
    };
}
#endif

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

    setName( "osgEarth::MapNode" );

    // so that our custom traverse() is called with an update visitor each frame
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
    if ( _mapNodeOptions.proxySettings().isSet() )
    {
		HTTPClient::setProxySettings( _mapNodeOptions.proxySettings().get() );
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

    // make a group for the model layers:
    _models = new osg::Group();
    _models->setName( "osgEarth::MapNode.modelsGroup" );
    addChild( _models.get() );

    // overlays:
    _pendingOverlayAutoSetTextureUnit = true;

    // go through the map and process any already-installed layers:
    // TODO: non-hard-code
    const TerrainOptions& terrainOptions = _mapNodeOptions.getTerrainOptions();
    _terrainEngine = TerrainEngineNodeFactory::create( _map.get(), terrainOptions );
    if ( _terrainEngine.valid() )
    {
        this->addChild( _terrainEngine.get() );
    }
    else
        OE_WARN << "FAILED to create a terrain engine for this map" << std::endl;

    // install any pre-existing model layers:
    ModelLayerVector modelLayers;
    _map->getModelLayers( modelLayers );
    for( ModelLayerVector::const_iterator k = modelLayers.begin(); k != modelLayers.end(); k++ )
    {
        onModelLayerAdded( k->get() );
    }

    // install any pre-existing mask layer:
    if ( _map->getTerrainMaskLayer() )
    {
        onMaskLayerAdded( _map->getTerrainMaskLayer() );
    }

    //updateStateSet();

    // install a layer callback for processing further map actions:
    _map->addMapCallback( new MapNodeMapCallbackProxy(this) );

    osg::StateSet* ss = getOrCreateStateSet();
	//ss->setAttributeAndModes( new osg::CullFace() ); //, osg::StateAttribute::ON);
    //ss->setAttributeAndModes( new osg::PolygonOffset( -1, -1 ) );

    if ( _mapNodeOptions.enableLighting().isSet() )
    {
        ss->setMode( GL_LIGHTING, _mapNodeOptions.enableLighting().value() ? 
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
            osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    }

    dirtyBound();
}

MapNode::~MapNode()
{
    removeChildren( 0, getNumChildren() );
}

osg::BoundingSphere
MapNode::computeBound() const
{
    osg::BoundingSphere bs;
    if ( _terrainEngine.valid() )
        bs.expandBy( _terrainEngine->getBound() );
    if ( _models.valid() )
        bs.expandBy( _models->getBound() );
    return bs;
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

const MapNodeOptions&
MapNode::getMapNodeOptions() const
{
    return _mapNodeOptions;
}

MapNode*
MapNode::findMapNode( osg::Node* graph )
{
    return findTopMostNodeOfType<MapNode>( graph );
}

bool
MapNode::isGeocentric() const
{
    return _map->isGeocentric();
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
            if ( dynamic_cast<osgSim::OverlayNode*>( node ) )
            {
              // treat overlay node as a special case
                installOverlayNode( static_cast<osgSim::OverlayNode*>( node ) );
            }
            else
            {
               _models->addChild( node );
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
        // look up the node associated with this model layer.
        ModelLayerNodeMap::iterator i = _modelLayerNodes.find( layer );
        if ( i != _modelLayerNodes.end() )
        {
            osg::Node* node = i->second;
            
            if ( dynamic_cast<osgSim::OverlayNode*>( node ) )
            {
                // treat overlay node as a special case
                uninstallOverlayNode( static_cast<osgSim::OverlayNode*>(node) );
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
        dirtyBound();
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
        dirtyBound();
    }
}

void
MapNode::addTerrainDecorator(osg::Group* decorator)
{    
    if ( _terrainEngine.valid() )
    {
        decorator->addChild( _terrainEngine.get() );
        _terrainEngine->getParent(0)->replaceChild( _terrainEngine.get(), decorator );
        dirtyBound();
    }
}

void
MapNode::removeTerrainDecorator(osg::Group* decorator)
{
    if ( _terrainEngine.valid() )
    {
        osg::Node* child = _terrainEngine.get();
        for( osg::Group* g = child->getParent(0); g != this; child = g, g = g->getParent(0) )
        {
            if ( g == decorator )
            {
                g->getParent(0)->replaceChild( g, child );
                break;
            }
        }
        dirtyBound();
    }
}

void
MapNode::installOverlayNode( osgSim::OverlayNode* overlay )
{
    if ( _terrainEngine.valid() )
    {
        overlay->addChild( _terrainEngine.get() );
        this->replaceChild( _terrainEngine.get(), overlay );
    }
}

void
MapNode::uninstallOverlayNode( osgSim::OverlayNode* overlay )
{
    if ( _terrainEngine.valid() )
    {
        osg::ref_ptr<osg::Node> overlayChild = overlay->getChild( 0 );
        this->replaceChild( overlay, overlayChild.get() );
    }
}

void
MapNode::traverse( osg::NodeVisitor& nv )
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

    osg::CoordinateSystemNode::traverse(nv);
}

