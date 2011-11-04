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

#define LC "[ModelLayer] "

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
_initOptions( options )
{
    copyOptions();
}

ModelLayer::ModelLayer( const std::string& name, const ModelSourceOptions& options ) :
_initOptions( ModelLayerOptions( name, options ) )
{
    copyOptions();
}

ModelLayer::ModelLayer( const ModelLayerOptions& options, ModelSource* source ) :
_modelSource( source ),
_initOptions( options )
{
    copyOptions();
}

ModelLayer::ModelLayer(const std::string& name, osg::Node* node):
_initOptions(ModelLayerOptions( name )),
_node(node)
{
    copyOptions();
}

void
ModelLayer::copyOptions()
{
    _runtimeOptions = _initOptions;
}

void
ModelLayer::initialize( const osgDB::Options* dbOptions, const Map* map )
{
    _dbOptions = osg::clone(dbOptions);

    if ( !_modelSource.valid() && _initOptions.driver().isSet() )
    {
        _modelSource = ModelSourceFactory::create( *_initOptions.driver() );
    }

    if ( _modelSource.valid() )
    {
        _modelSource->initialize( dbOptions, map );
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

            if ( _runtimeOptions.enabled().isSet() )
                setEnabled( *_runtimeOptions.enabled() );

            if ( _runtimeOptions.lightingEnabled().isSet() )
                setLightingEnabled( *_runtimeOptions.lightingEnabled() );

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
    return *_runtimeOptions.enabled();
}

void
ModelLayer::setEnabled(bool enabled)
{
    _runtimeOptions.enabled() = enabled;
    if ( _node.valid() )
        _node->setNodeMask( enabled ? ~0 : 0 );
}

void
ModelLayer::setLightingEnabled( bool value )
{
    _runtimeOptions.lightingEnabled() = value;
    if ( _node.valid() )
    {
        _node->getOrCreateStateSet()->setMode( 
            GL_LIGHTING, value ? osg::StateAttribute::ON : 
            (osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED) );

    }
}

bool
ModelLayer::isLightingEnabled() const
{
    return *_runtimeOptions.lightingEnabled();
}

bool
ModelLayer::getOverlay() const
{
    return *_runtimeOptions.overlay();
}

void
ModelLayer::setOverlay(bool overlay)
{
    if ( _runtimeOptions.overlay() != overlay )
    {
        _runtimeOptions.overlay() = overlay;
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
