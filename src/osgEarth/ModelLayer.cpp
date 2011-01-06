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
    //NOP
}

ModelLayer::ModelLayer( const ModelLayerOptions& options, ModelSource* source ) :
_modelSource( source ),
_options( options )
{
    //NOP
}

ModelLayer::ModelLayer(const std::string& name, osg::Node* node):
_options(ModelLayerOptions( name )),
_node(node)
{
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
    if ( _modelSource.valid() )
    {
        // if the model source has changed, regenerate the node.
        if ( _node.valid() && !_modelSource->inSyncWith(_modelSourceRev) )
        {
            _node = 0L;
        }

        if ( !_node.valid() )
        {
            _node = _modelSource->createNode( progress );

            if ( _enabled.isSet() )
                setEnabled( _enabled.get() );

            if ( _lighting.isSet() )
                setLightingEnabled( _lighting.get() );

            if ( _modelSource->getOptions().depthTestEnabled() == false )            
            {
                if ( _node )
                {
                    osg::StateSet* ss = _node->getOrCreateStateSet();
                    ss->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ) );
                    ss->setRenderBinDetails( 99999, "RenderBin" ); //TODO: configure this bin ...
                }
            }

            _modelSource->sync( _modelSourceRev );
        }
    }

    return _node.get();
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
        _node->getOrCreateStateSet()->setMode( GL_LIGHTING, value ? osg::StateAttribute::ON : osg::StateAttribute::OFF );
}

bool
ModelLayer::getOverlay() const
{
    return _overlay.get();
}

void
ModelLayer::setOverlay(bool overlay)
{
    if (_overlay != overlay)
    {
        _overlay = overlay;
        fireCallback( &ModelLayerCallback::onOverlayChanged );
    }
}

void
ModelLayer::addCallback( ModelLayerCallback* cb )
{
    _callbacks.push_back( cb );
}

void
ModelLayer::removeCallback( ModelLayerCallback* cb )
{
    ModelLayerCallbackList::iterator i = std::find( _callbacks.begin(), _callbacks.end(), cb );
    if ( i != _callbacks.end() ) 
        _callbacks.erase( i );
}


void
ModelLayer::fireCallback( ModelLayerCallbackMethodPtr method )
{
    for( ModelLayerCallbackList::const_iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
    {
        ModelLayerCallback* cb = i->get();
        (cb->*method)( this );
    }
}
