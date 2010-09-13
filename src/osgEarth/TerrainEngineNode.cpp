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

TerrainEngineNode::TerrainEngineNode( Map* map, const TerrainOptions& options ) :
_map( map ),
_terrainOptions( options )
{
    _map->addMapCallback( new TerrainEngineNodeCallbackProxy( this ) );
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
    _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( _map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED )
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
    //TODO...
}

//------------------------------------------------------------------------

#undef LC
#define LC "[TerrainEngineFactory] "

TerrainEngineNode*
TerrainEngineNodeFactory::create( const TerrainOptions& options )
{
    TerrainEngineNode* result = 0L;

    if ( !options.getDriver().empty() )
    {
        OE_INFO << LC << "Loading terrain engine from driver \"" << options.getDriver() << "\"" << std::endl;
    }

    std::string driverExt = std::string( ".osgearth_engine_" ) + options.getDriver();
    result = dynamic_cast<TerrainEngineNode*>( osgDB::readObjectFile( driverExt ) ); //, options ) );

    if ( !result )
    {
        OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << options.getDriver() << "\"" << std::endl;
    }

    return result;
}

