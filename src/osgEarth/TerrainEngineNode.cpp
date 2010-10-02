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
#include <osgDB/ReadFile>

#define LC "[TerrainEngineNode] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace osgEarth
{
    struct TerrainEngineNodeCallbackProxy : public MapCallback
    {
        TerrainEngineNodeCallbackProxy(TerrainEngineNode* node) : _node(node) { }
        osg::observer_ptr<TerrainEngineNode> _node;

        void onMapProfileEstablished( const Profile* profile ) {
            _node->onMapProfileEstablished( profile );
        }
        void onImageLayerAdded( ImageLayer* layer, unsigned int index ) {
            _node->onImageLayerAdded( layer );
            _node->onMapLayerStackChanged();
        }
        void onImageLayerRemoved( ImageLayer* layer, unsigned int index ) {
            _node->onImageLayerRemoved( layer );
            _node->onMapLayerStackChanged();
        }
        void onImageLayerMoved( ImageLayer* layer, unsigned int from, unsigned int to ) {
            _node->onMapLayerStackChanged();
        }
        void onElevationLayerAdded( ElevationLayer* layer, unsigned int index ) {
            _node->onMapLayerStackChanged();
        }
        void onElevationLayerRemoved( ElevationLayer* layer, unsigned int index ) {
            _node->onMapLayerStackChanged();
        }
        void onElevationLayerMoved( ElevationLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
            _node->onMapLayerStackChanged();
        }
    };
}

//------------------------------------------------------------------------

TerrainEngineNode::ImageLayerController::ImageLayerController( Map* map ) :
_mapf( map, Map::IMAGE_LAYERS, "TerrainEngineNode.ImageLayerController" )
{
    //NOP
}

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
TerrainEngineNode::initialize( Map* map, const TerrainOptions& options )
{
    _map = map;

    if ( _map.valid() )
    {
        // manually trigger the map callbacks the first time:
        if ( _map->getProfile() )
            onMapProfileEstablished( map->getProfile() );

        // create a layer controller. This object affects the uniforms that control layer appearance properties
        _imageLayerController = new ImageLayerController( map );

        // register the layer Controller it with all pre-existing image layers:
        MapFrame mapf( _map.get(), Map::IMAGE_LAYERS, "TerrainEngineNode::initialize" );
        for( ImageLayerVector::iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            i->get()->addCallback( _imageLayerController.get() );
        }

        onMapLayerStackChanged();

        // then register the callback
        _map->addMapCallback( new TerrainEngineNodeCallbackProxy( this ) );
    }

    // apply visual options.
    if ( options.enableLighting().isSet() )
    {
        this->getOrCreateStateSet()->setMode( GL_LIGHTING, options.enableLighting() == true ? 1 : 0 );
    }
}

osg::BoundingSphere
TerrainEngineNode::computeBound() const
{
    if ( _map.valid() && _map->isGeocentric() && getEllipsoidModel() )
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
TerrainEngineNode::onMapProfileEstablished( const Profile* profile )
{
    // set up the CSN values
    if ( _map.valid() )
        _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( !_map->isGeocentric() )
        this->setEllipsoidModel( NULL );
}

void
TerrainEngineNode::onMapLayerStackChanged()
{
    updateUniforms();
}

void
TerrainEngineNode::onImageLayerAdded( ImageLayer* layer )
{
    layer->addCallback( _imageLayerController.get() );
}

void
TerrainEngineNode::onImageLayerRemoved( ImageLayer* layer )
{
    layer->removeCallback( _imageLayerController.get() );
}

void
TerrainEngineNode::updateUniforms()
{
    // don't bother if this is a hurting old card
    if ( !Registry::instance()->getCapabilities().supportsGLSL() )
        return;

    // update the layer uniform arrays:
    osg::StateSet* stateSet = this->getOrCreateStateSet();

    // get a copy of the image laye stack:
    ImageLayerVector imageLayers;
    _map->getImageLayers( imageLayers );

    stateSet->removeUniform( "osgearth_imagelayer_opacity" );
    stateSet->removeUniform( "osgearth_imagelayer_enabled" );
    
    if ( imageLayers.size() > 0 )
    {
        //Update the layer opacity uniform
        _imageLayerController->_layerOpacityUniform = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_imagelayer_opacity", imageLayers.size() );
        for( ImageLayerVector::const_iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
            _imageLayerController->_layerOpacityUniform->setElement( (int)(i-imageLayers.begin()), i->get()->getOpacity() );
        stateSet->addUniform( _imageLayerController->_layerOpacityUniform.get() );

        //Update the layer enabled uniform
        _imageLayerController->_layerEnabledUniform = new osg::Uniform( osg::Uniform::BOOL, "osgearth_imagelayer_enabled", imageLayers.size() );
        for( ImageLayerVector::const_iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
        {
            _imageLayerController->_layerEnabledUniform->setElement( (int)(i-imageLayers.begin()), i->get()->getEnabled() );
        }
        stateSet->addUniform( _imageLayerController->_layerEnabledUniform.get() );
    }
    stateSet->getOrCreateUniform( "osgearth_imagelayer_count", osg::Uniform::INT )->set( (int)imageLayers.size() );
}

void
TerrainEngineNode::validateTerrainOptions( TerrainOptions& options )
{
    // make sure all the requested properties are compatible, and fall back as necessary.
    const Capabilities& caps = Registry::instance()->getCapabilities();

    // warn against mixing multipass technique with preemptive/sequential mode:
    if (options.layeringTechnique() == TerrainOptions::LAYERING_MULTIPASS &&
        options.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
    {
        OE_WARN << LC << "MULTIPASS layering is incompatible with preemptive/sequential loading policy; "
            << "falling back on STANDARD mode" << std::endl;
        options.loadingPolicy()->mode() = LoadingPolicy::MODE_STANDARD;
    }
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
        result->initialize( map, terrainOptions );
    }
    else
    {
        OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << driver << "\"" << std::endl;
    }

    return result;
}
