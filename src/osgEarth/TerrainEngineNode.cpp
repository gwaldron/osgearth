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
#include <osgDB/ReadFile>
#include <osg/CullFace>

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

TerrainEngineNode::ImageLayerController::ImageLayerController( Map* map ) :
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
        _layerEnabledUniform->setElement( layerNum, layer->getEnabled() );
    else
        OE_WARN << LC << "Odd, updateLayerOpacity did not find layer" << std::endl;
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
        _layerOpacityUniform->setElement( layerNum, layer->getOpacity() );
    else
        OE_WARN << LC << "Odd, onOpacityChanged did not find layer" << std::endl;
}

// this handler adjusts the uniform set when an image layer's "gamma" value changes
void
TerrainEngineNode::ImageLayerController::onGammaChanged( ImageLayer* layer )
{
    //TODO
}

//------------------------------------------------------------------------

TerrainEngineNode::TerrainEngineNode() :
_verticalScale( 1.0f ),
_elevationSamplingRatio( 1.0f )
{
    //nop
}

TerrainEngineNode::TerrainEngineNode( const TerrainEngineNode& rhs, const osg::CopyOp& op ) :
osg::CoordinateSystemNode( rhs, op ),
_verticalScale( rhs._verticalScale ),
_elevationSamplingRatio( rhs._elevationSamplingRatio ),
_map( rhs._map.get() )
{
    //nop
}

void
TerrainEngineNode::preinitialize( const MapInfo& mapInfo, const TerrainOptions& options )
{
    // set up the CSN values   
    mapInfo.getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( !mapInfo.isGeocentric() )
        this->setEllipsoidModel( NULL );
    
    // install the proper layer composition technique:
    _texCompositor = new TextureCompositor( options ); //.compositingTechnique().value() );

    // enable backface culling
    getOrCreateStateSet()->setAttributeAndModes( new osg::CullFace( osg::CullFace::BACK ), osg::StateAttribute::ON );
}

void
TerrainEngineNode::initialize( Map* map, const TerrainOptions& options )
{
    _map = map;

    if ( _map.valid() )
    {
        // manually trigger the map callbacks the first time:
        if ( _map->getProfile() )
            onMapInfoEstablished( MapInfo(_map.get()) );

        // create a layer controller. This object affects the uniforms that control layer appearance properties
        _imageLayerController = new ImageLayerController( map );

        // register the layer Controller it with all pre-existing image layers:
        MapFrame mapf( _map.get(), Map::IMAGE_LAYERS, "TerrainEngineNode::initialize" );
        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            i->get()->addCallback( _imageLayerController.get() );
        }

        updateImageUniforms();

        // then register the callback
        _map->addMapCallback( new TerrainEngineNodeCallbackProxy( this ) );
    }
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
        //change.getAction() == MapModelChange::ADD_ELEVATION_LAYER ||
        //change.getAction() == MapModelChange::REMOVE_ELEVATION_LAYER ||
        //change.getAction() == MapModelChange::MOVE_ELEVATION_LAYER )
    {
        updateImageUniforms();
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

    stateSet->removeUniform( "osgearth_ImageLayerOpacity" );
    stateSet->removeUniform( "osgearth_ImageLayerEnabled" );
    stateSet->removeUniform( "osgearth_ImageLayerRange" );    
    stateSet->removeUniform( "osgearth_ImageLayerAttenuation" );
    
    if ( mapf.imageLayers().size() > 0 )
    {
        // the "enabled" uniform is fixed size. this is handy to account for layers that are in flux...i.e., their source
        // layer count has changed, but the shader has not yet caught up. In the future we might use this to disable
        // "ghost" layers that used to exist at a given index, but no longer do.
        _imageLayerController->_layerEnabledUniform = new osg::Uniform( osg::Uniform::BOOL, "osgearth_ImageLayerEnabled", 128 ); //mapf.imageLayers().size() );
        _imageLayerController->_layerOpacityUniform = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_ImageLayerOpacity", mapf.imageLayers().size() );
        _imageLayerController->_layerRangeUniform = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_ImageLayerRange", 2 * mapf.imageLayers().size() );

        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            ImageLayer* layer = i->get();
            int index = (int)(i - mapf.imageLayers().begin());

            _imageLayerController->_layerOpacityUniform->setElement( index, layer->getOpacity() );
            _imageLayerController->_layerEnabledUniform->setElement( index, layer->getEnabled() );
            _imageLayerController->_layerRangeUniform->setElement( (2*index), layer->getImageLayerOptions().minVisibleRange().value() );
            _imageLayerController->_layerRangeUniform->setElement( (2*index)+1, layer->getImageLayerOptions().maxVisibleRange().value() );
        }

        // set the remainder of the layers to disabled 
        for( int j=mapf.imageLayers().size(); j<128; ++j )
            _imageLayerController->_layerEnabledUniform->setElement( j, false );

        stateSet->addUniform( _imageLayerController->_layerOpacityUniform.get() );
        stateSet->addUniform( _imageLayerController->_layerEnabledUniform.get() );
        stateSet->addUniform( _imageLayerController->_layerRangeUniform.get() );
    }

    stateSet->getOrCreateUniform( "osgearth_ImageLayerAttenuation", osg::Uniform::FLOAT )->set(
        *getTerrainOptions().attentuationDistance() );
}

void
TerrainEngineNode::validateTerrainOptions( TerrainOptions& options )
{
    // make sure all the requested properties are compatible, and fall back as necessary.
    const Capabilities& caps = Registry::instance()->getCapabilities();

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
        if ( Registry::instance()->getCapabilities().supportsGLSL() )
            _updateLightingUniformsHelper.cullTraverse( this, &nv );
    }

    //else if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    //{
    //    if ( Registry::instance()->getCapabilities().supportsGLSL() )
    //        _updateLightingUniformsHelper.updateTraverse( this );
    //}

    osg::CoordinateSystemNode::traverse( nv );
}

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
        //result->initialize( map, terrainOptions );
    }
    else
    {
        OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << driver << "\"" << std::endl;
    }

    return result;
}
