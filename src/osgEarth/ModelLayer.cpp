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
#include <osgEarth/ModelLayer>
#include <osgEarth/Map>
#include <osg/Depth>

using namespace osgEarth;

ModelLayer::ModelLayer( const std::string& name, const DriverOptions* options ) :
osg::Referenced( true ),
_name( name ),
_driverOptions( options ),
_enabled(true)
{
    if (options)
        fromConfig( options->config() );
}

ModelLayer::ModelLayer( const std::string& name, ModelSource* source ) :
osg::Referenced( true ),
_name( name ),
_modelSource( source ),
_enabled(true)
{
    //NOP
}

void
ModelLayer::initialize( const std::string& referenceURI, const Map* map )
{
    _referenceURI = referenceURI;

    if ( !_modelSource.valid() )
    {
        _modelSource = ModelSourceFactory::create( _driverOptions.get() );
    }

    if ( !_modelSource.valid() )
    {
        _modelSource = ModelSourceFactory::create( _name, _driver, _driverConf );
    }

    if ( _modelSource.valid() )
    {
        _modelSource->initialize( _referenceURI, map );
    }
}

osg::Node*
ModelLayer::getOrCreateNode( ProgressCallback* progress )
{
    if (!_node.valid() && _modelSource.valid())
    {
        _node = _modelSource->createNode( progress );

        if ( _enabled.isSet() )
            setEnabled( _enabled.value() );

        if ( _lighting.isSet() )
            setLightingEnabled( _lighting.value() );

        if ( _modelSource->getOptions()->depthTestEnabled() == false )            
        {
            if ( _node )
                _node->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ) );
        }
    }
    return _node.get();
}

Config
ModelLayer::toConfig() const
{
    Config conf = 
        _driverOptions.valid() ? _driverOptions->toConfig() : Config();

    conf.key() = "model";
    conf.attr("name") = _name;
    conf.updateIfSet( "enabled", _enabled );
    conf.updateIfSet( "lighting", _lighting );

    return conf;
}

bool
ModelLayer::getEnabled() const
{
    return _enabled.get();
}

void
ModelLayer::setEnabled(bool enabled)
{
    _enabled = enabled;
    if ( _node.valid() )
        _node->setNodeMask( _enabled.get() ? ~0 : 0 );
}

void
ModelLayer::setLightingEnabled( bool value )
{
    _lighting = value;
    if ( _node.valid() )
        _node->getOrCreateStateSet()->setMode( GL_LIGHTING, value ? 1 : 0 );
}

void
ModelLayer::fromConfig(const osgEarth::Config &conf)
{
    conf.getIfSet( "enabled", _enabled );
    conf.getIfSet( "lighting", _lighting );
}
