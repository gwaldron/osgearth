/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>
#include <osgEarth/Registry>
#include <osgEarth/TextureCompositor>
#include <osgEarth/NodeUtils>
#include <osgEarth/MapModelChange>
#include <osgDB/ReadFile>
#include <osg/CullFace>
#include <osg/PolygonOffset>
#include <osgViewer/View>

#define LC "[TerrainEngineNode] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace osgEarth
{
    struct TerrainEngineNodeCallbackProxy : public MapCallback
    {
        TerrainEngineNodeCallbackProxy(TerrainEngineNode* node) : _node(node) { }
        osg::observer_ptr<TerrainEngineNode> _node;

        void onMapInfoEstablished( const MapInfo& mapInfo )
        {
            osg::ref_ptr<TerrainEngineNode> safeNode = _node.get();
            if ( safeNode.valid() )
                safeNode->onMapInfoEstablished( mapInfo );
        }

        void onMapModelChanged( const MapModelChange& change )
        {
            osg::ref_ptr<TerrainEngineNode> safeNode = _node.get();
            if ( safeNode.valid() )
                safeNode->onMapModelChanged( change );
        }
    };
}


//------------------------------------------------------------------------


TerrainEngineNode::ImageLayerController::ImageLayerController(const Map*         map,
                                                              TerrainEngineNode* engine) :
_mapf  ( map, Map::IMAGE_LAYERS, "TerrainEngineNode.ImageLayerController" ),
_engine( engine )
{
    //nop
}


void
TerrainEngineNode::addEffect(TerrainEffect* effect)
{
    if ( effect )
    {
        effects_.push_back( effect );
        effect->onInstall( this );
    }
}


void
TerrainEngineNode::removeEffect(TerrainEffect* effect)
{
    if ( effect )
    {
        effect->onUninstall(this);
        TerrainEffectVector::iterator i = std::find(effects_.begin(), effects_.end(), effect);
        if ( i != effects_.end() )
            effects_.erase( i );
    }
}


TextureCompositor*
TerrainEngineNode::getTextureCompositor() const
{
    return _texCompositor.get();
}


// this handler adjusts the uniform set when a terrain layer's "enabed" state changes
void
TerrainEngineNode::ImageLayerController::onVisibleChanged( TerrainLayer* layer )
{
    _engine->dirty();
}


// this handler adjusts the uniform set when a terrain layer's "opacity" value changes
void
TerrainEngineNode::ImageLayerController::onOpacityChanged( ImageLayer* layer )
{
    _engine->dirty();
}

void
TerrainEngineNode::ImageLayerController::onVisibleRangeChanged( ImageLayer* layer )
{
    _engine->dirty();
}

void
TerrainEngineNode::ImageLayerController::onColorFiltersChanged( ImageLayer* layer )
{
    _engine->updateTextureCombining();
    _engine->dirty();
}



//------------------------------------------------------------------------


TerrainEngineNode::TerrainEngineNode() :
_verticalScale         ( 1.0f ),
_initStage             ( INIT_NONE ),
_dirtyCount            ( 0 )
{
    // register for event traversals so we can properly reset the dirtyCount
    ADJUST_EVENT_TRAV_COUNT( this, 1 );
}


TerrainEngineNode::~TerrainEngineNode()
{
    //Remove any callbacks added to the image layers
    if (_map.valid())
    {
        MapFrame mapf( _map.get(), Map::IMAGE_LAYERS, "TerrainEngineNode::~TerrainEngineNode" );
        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            i->get()->removeCallback( _imageLayerController.get() );
        }
    }
}


void
TerrainEngineNode::dirty()
{
    if ( 0 == _dirtyCount++ )
    {
        // notify any attached Views
        ViewVisitor<RequestRedraw> visitor;
        this->accept(visitor);
    }
}


void
TerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    _map = map;
    
    // fire up a terrain utility interface
    _terrainInterface = new Terrain( this, map->getProfile(), map->isGeocentric(), options );

    // set up the CSN values   
    _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( !_map->isGeocentric() )
        this->setEllipsoidModel( NULL );
    
    // install the proper layer composition technique:
    _texCompositor = new TextureCompositor();

    // then register the callback so we can process further map model changes
    _map->addMapCallback( new TerrainEngineNodeCallbackProxy( this ) );

    // enable backface culling
    osg::StateSet* set = getOrCreateStateSet();
    set->setMode( GL_CULL_FACE, 1 );

    if ( options.enableMercatorFastPath().isSet() )
    {
        OE_INFO 
            << LC << "Mercator fast path " 
            << (options.enableMercatorFastPath()==true? "enabled" : "DISABLED") << std::endl;
    }

    _initStage = INIT_PREINIT_COMPLETE;
}

void
TerrainEngineNode::postInitialize( const Map* map, const TerrainOptions& options )
{
    if ( _map.valid() ) // i think this is always true [gw]
    {
        // manually trigger the map callbacks the first time:
        if ( _map->getProfile() )
            onMapInfoEstablished( MapInfo(_map.get()) );

        // create a layer controller. This object affects the uniforms that control layer appearance properties
        _imageLayerController = new ImageLayerController( _map.get(), this );

        // register the layer Controller it with all pre-existing image layers:
        MapFrame mapf( _map.get(), Map::IMAGE_LAYERS, "TerrainEngineNode::initialize" );
        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            i->get()->addCallback( _imageLayerController.get() );
        }
    }

    _initStage = INIT_POSTINIT_COMPLETE;
}

osg::BoundingSphere
TerrainEngineNode::computeBound() const
{
    if ( getEllipsoidModel() )
    {
        double maxRad = std::max(
            getEllipsoidModel()->getRadiusEquator(),
            getEllipsoidModel()->getRadiusPolar());

        return osg::BoundingSphere( osg::Vec3(0,0,0), maxRad+25000 );
    }
    else
    {
        return osg::CoordinateSystemNode::computeBound();
    }
}

void
TerrainEngineNode::setVerticalScale( float value )
{
    _verticalScale = value;
    onVerticalScaleChanged();
}

void
TerrainEngineNode::onMapInfoEstablished( const MapInfo& mapInfo )
{
    // set up the CSN values   
    mapInfo.getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( !mapInfo.isGeocentric() )
        this->setEllipsoidModel( NULL );
}

void
TerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
    if ( _initStage == INIT_POSTINIT_COMPLETE )
    {
        if ( change.getAction() == MapModelChange::ADD_IMAGE_LAYER )
        {
            change.getImageLayer()->addCallback( _imageLayerController.get() );
        }
        else if ( change.getAction() == MapModelChange::REMOVE_IMAGE_LAYER )
        {
            change.getImageLayer()->removeCallback( _imageLayerController.get() );
        }
    }

    // notify that a redraw is required.
    dirty();
}

namespace
{
    Threading::Mutex s_opqlock;
}

void
TerrainEngineNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        // see if we need to set up the Terrain object with an update ops queue.
        if ( !_terrainInterface->_updateOperationQueue.valid() )
        {
            Threading::ScopedMutexLock lock(s_opqlock);
            if ( !_terrainInterface->_updateOperationQueue.valid() ) // double check pattern
            {
                //TODO: think, will this work with >1 view?
                osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
                if ( cv->getCurrentCamera() )
                {
                    osgViewer::View* view = dynamic_cast<osgViewer::View*>(cv->getCurrentCamera()->getView());
                    if ( view && view->getViewerBase() )
                    {
                        osg::OperationQueue* q = view->getViewerBase()->getUpdateOperations();
                        if ( !q ) {
                            q = new osg::OperationQueue();
                            view->getViewerBase()->setUpdateOperations( q );
                        }
                        _terrainInterface->_updateOperationQueue = q;
                    }
                }
            }
        }
    }

    else if ( nv.getVisitorType() == osg::NodeVisitor::EVENT_VISITOR )
    {
        _dirtyCount = 0;
    }

    osg::CoordinateSystemNode::traverse( nv );
}

//------------------------------------------------------------------------

#undef LC
#define LC "[TerrainEngineNodeFactory] "

TerrainEngineNode*
TerrainEngineNodeFactory::create( Map* map, const TerrainOptions& options )
{
    TerrainEngineNode* result = 0L;

    std::string driver = options.getDriver();
    if ( driver.empty() )
        driver = Registry::instance()->getDefaultTerrainEngineDriverName();

    std::string driverExt = std::string( ".osgearth_engine_" ) + driver;
    result = dynamic_cast<TerrainEngineNode*>( osgDB::readObjectFile( driverExt ) );
    if ( !result )
    {
        OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << driver << "\"" << std::endl;
    }

    return result;
}

//------------------------------------------------------------------------
TerrainDecorator::~TerrainDecorator()
{
}

void TerrainDecorator::onInstall( TerrainEngineNode* engine )
{
}

void TerrainDecorator::onUninstall( TerrainEngineNode* engine )
{
}

