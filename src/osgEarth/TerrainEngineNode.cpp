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
        void onMapLayerAdded( MapLayer* layer, unsigned int index ) {
            _node->onMapLayerStackChanged();
        }
        void onMapLayerRemoved( MapLayer* layer, unsigned int index ) {
            _node->onMapLayerStackChanged();
        }
        void onMapLayerMoved( MapLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
            _node->onMapLayerStackChanged();
        }
    };
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
TerrainEngineNode::updateUniforms()
{
    // don't bother if this is a hurting old card
    if ( !Registry::instance()->getCapabilities().supportsGLSL() )
        return;

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
TerrainEngineNode::updateLayerOpacity( MapLayer* layer )
{
    if ( !Registry::instance()->getCapabilities().supportsGLSL() )
        return;

    MapLayerList imageLayers;
    _map->getImageMapLayers( imageLayers );

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
TerrainEngineNode::updateLayerEnabled( MapLayer* layer )
{
    if ( !Registry::instance()->getCapabilities().supportsGLSL() )
        return;

    MapLayerList imageLayers;
    _map->getImageMapLayers( imageLayers );

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
