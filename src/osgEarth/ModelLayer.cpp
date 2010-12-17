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
//------------------------------------------------------------------------

ModelLayerOptions::ModelLayerOptions( const ConfigOptions& options ) :
ConfigOptions( options )
{
    setDefaults();
    fromConfig( _conf ); 
}

ModelLayerOptions::ModelLayerOptions( const std::string& name, const ModelSourceOptions& driverOptions ) :
ConfigOptions()
{
    setDefaults();
    fromConfig( _conf );
    _name = name;
    _driver = driverOptions;
}

void
ModelLayerOptions::setDefaults()
{
    _overlay.init( false );
    _enabled.init( true );
    _lighting.init( true );
}

Config
ModelLayerOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();

    conf.updateIfSet( "name", _name );
    conf.updateIfSet( "overlay", _overlay );
    conf.updateIfSet( "enabled", _enabled );
    conf.updateIfSet( "lighting", _lighting );

    return conf;
}

void
ModelLayerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "name", _name );
    conf.getIfSet( "overlay", _overlay );
    conf.getIfSet( "enabled", _enabled );
    conf.getIfSet( "lighting", _lighting );
}

void
ModelLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

ModelLayer::ModelLayer( const ModelLayerOptions& options ) :
_options( options ),
_enabled( true )
{
    // NOP
}

ModelLayer::ModelLayer( const std::string& name, const ModelSourceOptions& options ) :
_options( ModelLayerOptions( name, options ) ),
_enabled( true )
{
//    mergeConfig( options.getConfig() );
}

ModelLayer::ModelLayer( const ModelLayerOptions& options, ModelSource* source ) :
_options( options ),
_modelSource( source )
{
    //NOP
}

void
ModelLayer::initialize( const std::string& referenceURI, const Map* map )
{
    _referenceURI = referenceURI;

    if ( !_modelSource.valid() && _options.driver().isSet() )
    {
        _modelSource = ModelSourceFactory::create( *_options.driver() );
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

        if ( _options.enabled().isSet() )
            setEnabled( *_options.enabled() );

        if ( _options.lightingEnabled().isSet() )
            setLightingEnabled( *_options.lightingEnabled() );

        if ( _modelSource->getOptions().depthTestEnabled() == false )            
        {
            if ( _node )
            {
                osg::StateSet* ss = _node->getOrCreateStateSet();
                ss->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ) );
                ss->setRenderBinDetails( 99999, "RenderBin" ); //TODO: configure this bin ...
            }
        }
    }
    return _node.get();
}

//Config
//ModelLayer::getConfig() const
//{
//    Config conf = _driverOptions.getConfig();
//
//    conf.key() = "model";
//    conf.attr("name") = _name;
//    conf.updateIfSet( "enabled", _enabled );
//    conf.updateIfSet( "lighting", _lighting );
//
//    return conf;
//}
//
//void
//ModelLayer::mergeConfig(const osgEarth::Config &conf)
//{
//    conf.getIfSet( "enabled", _enabled );
//    conf.getIfSet( "lighting", _lighting );
//}

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
