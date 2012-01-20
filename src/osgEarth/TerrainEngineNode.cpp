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
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Capabilities>
#include <osgEarth/Registry>
#include <osgEarth/FindNode>
#include <osgEarth/TextureCompositor>
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

TerrainEngineNode::ImageLayerController::ImageLayerController( const Map* map ) :
_mapf( map, Map::IMAGE_LAYERS, "TerrainEngineNode.ImageLayerController" )
{
    //nop
}

// this handler adjusts the uniform set when a terrain layer's "enabed" state changes
void
TerrainEngineNode::ImageLayerController::onEnabledChanged( TerrainLayer* layer )
{
    if ( !Registry::instance()->getCapabilities().supportsGLSL() )
        return;

    _mapf.sync();
    int layerNum = _mapf.indexOf( static_cast<ImageLayer*>(layer) );
    if ( layerNum >= 0 )
        _layerEnabledUniform.setElement( layerNum, layer->getEnabled() );
    else
        OE_WARN << LC << "Odd, updateLayerOpacity did not find layer" << std::endl;
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

// this handler adjusts the uniform set when a terrain layer's "opacity" value changes
void
TerrainEngineNode::ImageLayerController::onOpacityChanged( ImageLayer* layer )
{
    if ( !Registry::instance()->getCapabilities().supportsGLSL() )
        return;

    _mapf.sync();
    int layerNum = _mapf.indexOf( layer );
    if ( layerNum >= 0 )
        _layerOpacityUniform.setElement( layerNum, layer->getOpacity() );
    else
        OE_WARN << LC << "Odd, onOpacityChanged did not find layer" << std::endl;
}

//------------------------------------------------------------------------

TerrainEngineNode::TerrainEngineNode() :
_verticalScale( 1.0f ),
_elevationSamplingRatio( 1.0f ),
_initStage( INIT_NONE )
{
    ctor();
}

TerrainEngineNode::TerrainEngineNode( const TerrainEngineNode& rhs, const osg::CopyOp& op ) :
osg::CoordinateSystemNode( rhs, op ),
_verticalScale( rhs._verticalScale ),
_elevationSamplingRatio( rhs._elevationSamplingRatio ),
_map( rhs._map.get() ),
_initStage( rhs._initStage )
{
    ctor();
}

void
TerrainEngineNode::ctor()
{
    //nop
}

void
TerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    _map = map;
    
    // fire up a terrain utility interface
    _terrainInterface = new Terrain( this, map->getProfile(), map->isGeocentric() );

    // set up the CSN values   
    _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( !_map->isGeocentric() )
        this->setEllipsoidModel( NULL );
    
    // install the proper layer composition technique:
    _texCompositor = new TextureCompositor( options );

    // prime the compositor with pre-existing image layers:
    MapFrame mapf(map, Map::IMAGE_LAYERS);
    for( unsigned i=0; i<mapf.imageLayers().size(); ++i )
    {
        _texCompositor->applyMapModelChange( MapModelChange(
            MapModelChange::ADD_IMAGE_LAYER,
            mapf.getRevision(),
            mapf.getImageLayerAt(i),
            i ) );
    }

    // then register the callback so we can process further map model changes
    _map->addMapCallback( new TerrainEngineNodeCallbackProxy( this ) );

    // enable backface culling
    osg::StateSet* set = getOrCreateStateSet();
    set->setAttributeAndModes( new osg::CullFace( osg::CullFace::BACK ), osg::StateAttribute::ON );

    // elevation uniform
    _cameraElevationUniform = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_CameraElevation" );
    _cameraElevationUniform->set( 0.0f );
    set->addUniform( _cameraElevationUniform.get() );
    
    set->getOrCreateUniform( "osgearth_ImageLayerAttenuation", osg::Uniform::FLOAT )->set(
        *options.attentuationDistance() );

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
        _imageLayerController = new ImageLayerController( _map.get() );

        // register the layer Controller it with all pre-existing image layers:
        MapFrame mapf( _map.get(), Map::IMAGE_LAYERS, "TerrainEngineNode::initialize" );
        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            i->get()->addCallback( _imageLayerController.get() );
        }

        updateImageUniforms();

        // then register the callback
        // NOTE: moved this into preInitialize
        //_map->addMapCallback( new TerrainEngineNodeCallbackProxy( this ) );
    }

    _initStage = INIT_POSTINIT_COMPLETE;
}

osg::BoundingSphere
TerrainEngineNode::computeBound() const
{
    if ( getEllipsoidModel() )
    {
        return osg::BoundingSphere( osg::Vec3(0,0,0), getEllipsoidModel()->getRadiusEquator()+25000 );
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
TerrainEngineNode::setElevationSamplingRatio( float value )
{
    _elevationSamplingRatio = value;
    onElevationSamplingRatioChanged();
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

        if (change.getAction() == MapModelChange::ADD_IMAGE_LAYER ||
            change.getAction() == MapModelChange::REMOVE_IMAGE_LAYER ||
            change.getAction() == MapModelChange::MOVE_IMAGE_LAYER )
        {
            updateImageUniforms();
        }
    }

    // if post-initialization has not yet happened, we need to make sure the 
    // compositor is up to date with the map model. (After post-initialization,
    // this happens in the subclass...something that probably needs to change
    // since this is unclear)
    else if ( _texCompositor.valid() )
    {
        _texCompositor->applyMapModelChange( change );
    }
}

void
TerrainEngineNode::updateImageUniforms()
{
    // don't bother if this is a hurting old card
    if ( !Registry::instance()->getCapabilities().supportsGLSL() )
        return;

    // update the layer uniform arrays:
    osg::StateSet* stateSet = this->getOrCreateStateSet();

    // get a copy of the image layer stack:
    MapFrame mapf( _map.get(), Map::IMAGE_LAYERS );

    _imageLayerController->_layerEnabledUniform.detach();
    _imageLayerController->_layerOpacityUniform.detach();
    _imageLayerController->_layerRangeUniform.detach();
    
    if ( mapf.imageLayers().size() > 0 )
    {
        // the "enabled" uniform is fixed size. this is handy to account for layers that are in flux...i.e., their source
        // layer count has changed, but the shader has not yet caught up. In the future we might use this to disable
        // "ghost" layers that used to exist at a given index, but no longer do.
        
        _imageLayerController->_layerEnabledUniform.attach( "osgearth_ImageLayerEnabled", osg::Uniform::BOOL,  stateSet, MAX_IMAGE_LAYERS );
        _imageLayerController->_layerOpacityUniform.attach( "osgearth_ImageLayerOpacity", osg::Uniform::FLOAT, stateSet, mapf.imageLayers().size() );
        _imageLayerController->_layerRangeUniform.attach  ( "osgearth_ImageLayerRange",   osg::Uniform::FLOAT, stateSet, 2 * mapf.imageLayers().size() );

        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            ImageLayer* layer = i->get();
            int index = (int)(i - mapf.imageLayers().begin());

            _imageLayerController->_layerOpacityUniform.setElement( index, layer->getOpacity() );
            _imageLayerController->_layerEnabledUniform.setElement( index, layer->getEnabled() );
            _imageLayerController->_layerRangeUniform.setElement( (2*index), layer->getImageLayerOptions().minVisibleRange().value() );
            _imageLayerController->_layerRangeUniform.setElement( (2*index)+1, layer->getImageLayerOptions().maxVisibleRange().value() );
        }

        // set the remainder of the layers to disabled 
        for( int j=mapf.imageLayers().size(); j<_imageLayerController->_layerEnabledUniform.getNumElements(); ++j)
            _imageLayerController->_layerEnabledUniform.setElement( j, false );
    }
}

void
TerrainEngineNode::validateTerrainOptions( TerrainOptions& options )
{
    // make sure all the requested properties are compatible, and fall back as necessary.
    //const Capabilities& caps = Registry::instance()->getCapabilities();

    // warn against mixing multipass technique with preemptive/sequential mode:
    if (options.compositingTechnique() == TerrainOptions::COMPOSITING_MULTIPASS &&
        options.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
    {
        OE_WARN << LC << "MULTIPASS compositor is incompatible with preemptive/sequential loading policy; "
            << "falling back on STANDARD mode" << std::endl;
        options.loadingPolicy()->mode() = LoadingPolicy::MODE_STANDARD;
    }
}

void
TerrainEngineNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        // see if we need to set up the Terrain object with an update ops queue.
        if ( !_terrainInterface->_updateOperationQueue.valid() )
        {
            static Threading::Mutex s_opqlock;
            Threading::ScopedMutexLock lock(s_opqlock);
            if ( !_terrainInterface->_updateOperationQueue.valid() ) // double check pattern
            {
                osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );
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


        if ( Registry::instance()->getCapabilities().supportsGLSL() )
        {
            _updateLightingUniformsHelper.cullTraverse( this, &nv );

            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );
            if ( cv )
            {
                osg::Vec3d eye = cv->getEyePoint();

                float elevation;
                if ( _map->isGeocentric() )
                    elevation = eye.length() - osg::WGS_84_RADIUS_EQUATOR;
                else
                    elevation = eye.z();

                _cameraElevationUniform->set( elevation );
            }
        }
    }

    //else if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    //{
    //    if ( Registry::instance()->getCapabilities().supportsGLSL() )
    //        _updateLightingUniformsHelper.updateTraverse( this );
    //}

    osg::CoordinateSystemNode::traverse( nv );
}

#if 0
void
TerrainEngineNode::addTerrainChangedCallback( TerrainChangedCallback* callback )
{
    Threading::ScopedWriteLock lock( _terrainChangedCallbacksMutex );
    _terrainChangedCallbacks.push_back( callback );
}

void
TerrainEngineNode::removeTerrainChangedCallback( TerrainChangedCallback* callback)
{
    Threading::ScopedWriteLock lock( _terrainChangedCallbacksMutex );
    TerrainChangedCallbackList::iterator i = std::find(_terrainChangedCallbacks.begin(), _terrainChangedCallbacks.end(), callback);
    if (i != _terrainChangedCallbacks.end()) _terrainChangedCallbacks.erase( i );    
}

void
TerrainEngineNode::fireTerrainChanged( const osgEarth::TileKey& tileKey, osg::Node* terrain )
{    
    Threading::ScopedReadLock lock( _terrainChangedCallbacksMutex );
    for (TerrainChangedCallbackList::iterator i = _terrainChangedCallbacks.begin(); i != _terrainChangedCallbacks.end(); i++)
    {
        i->get()->onTerrainChanged(tileKey, terrain);
    }
}
#endif


//------------------------------------------------------------------------

#undef LC
#define LC "[TerrainEngineFactory] "

TerrainEngineNode*
TerrainEngineNodeFactory::create( Map* map, const TerrainOptions& options )
{
    TerrainEngineNode* result = 0L;

    std::string driver = options.getDriver();
    if ( driver.empty() )
        driver = "osgterrain";

    std::string driverExt = std::string( ".osgearth_engine_" ) + driver;
    result = dynamic_cast<TerrainEngineNode*>( osgDB::readObjectFile( driverExt ) );
    if ( result )
    {
        TerrainOptions terrainOptions( options );
        result->validateTerrainOptions( terrainOptions );
    }
    else
    {
        OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << driver << "\"" << std::endl;
    }

    return result;
}

