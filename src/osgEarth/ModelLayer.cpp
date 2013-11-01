/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderFactory>
#include <osg/Depth>

#define LC "[ModelLayer] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    /**
     * Most basic of model sources; used to support the osg::Node* constructor to ModelLayer.
     */
    struct NodeModelSource : public ModelSource
    {
        NodeModelSource( osg::Node* node ) : _node(node) { }

        osg::Node* createNodeImplementation(const Map* map, const osgDB::Options* dbOptions, ProgressCallback* progress) {
            return _node.get();
        }

        osg::ref_ptr<osg::Node> _node;
    };
}

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
    _enabled.init     ( true );
    _visible.init     ( true );
    _lighting.init    ( true );
}

Config
ModelLayerOptions::getConfig() const
{
    //Config conf = ConfigOptions::getConfig();
    Config conf = ConfigOptions::newConfig();

    conf.updateIfSet( "name", _name );
    conf.updateIfSet( "enabled", _enabled );
    conf.updateIfSet( "visible", _visible );
    conf.updateIfSet( "lighting", _lighting );

    // Merge the ModelSource options
    if ( driver().isSet() )
        conf.merge( driver()->getConfig() );

    return conf;
}

void
ModelLayerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "name", _name );
    conf.getIfSet( "enabled", _enabled );
    conf.getIfSet( "visible", _visible );
    conf.getIfSet( "lighting", _lighting );

    if ( conf.hasValue("driver") )
        driver() = ModelSourceOptions(conf);
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
_initOptions( ModelLayerOptions(name) ),
_modelSource( new NodeModelSource(node) )
{
    copyOptions();
}

ModelLayer::~ModelLayer()
{
    OE_DEBUG << "~ModelLayer" << std::endl;
}

void
ModelLayer::copyOptions()
{
    _runtimeOptions = _initOptions;
}

void
ModelLayer::initialize( const osgDB::Options* dbOptions )
{
    if ( !_modelSource.valid() && _initOptions.driver().isSet() )
    {
        _modelSource = ModelSourceFactory::create( *_initOptions.driver() );

        if ( _modelSource.valid() )
        {
            _modelSource->initialize( dbOptions );
        }
    }
}

osg::Node*
ModelLayer::createSceneGraph(const Map*            map,
                             const osgDB::Options* dbOptions,
                             ProgressCallback*     progress )
{
    osg::Node* node = 0L;

    if ( _modelSource.valid() )
    {
        node = _modelSource->createNode( map, dbOptions, progress );

        if ( node )
        {
            if ( _runtimeOptions.visible().isSet() )
            {
                node->setNodeMask( *_runtimeOptions.visible() ? ~0 : 0 );
            }

            if ( _runtimeOptions.lightingEnabled().isSet() )
            {
                setLightingEnabled( *_runtimeOptions.lightingEnabled() );
            }

            if ( _modelSource->getOptions().depthTestEnabled() == false )
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ) );
                ss->setRenderBinDetails( 99999, "RenderBin" ); //TODO: configure this bin ...
            }

#if 0 // moved the MapNode level.
            if ( Registry::capabilities().supportsGLSL() )
            {
                // install a callback that keeps the shader uniforms up to date
                node->addCullCallback( new UpdateLightingUniformsHelper() );
            }
#endif

            _modelSource->sync( _modelSourceRev );

            // save an observer reference to the node so we can change the visibility/lighting/etc.
            _nodeSet.insert( node );
        }
    }

    return node;
}

bool
ModelLayer::getEnabled() const
{
    return *_runtimeOptions.enabled();
}

bool
ModelLayer::getVisible() const
{
    return getEnabled() && *_runtimeOptions.visible();
}

void
ModelLayer::setVisible(bool value)
{
    if ( _runtimeOptions.visible() != value )
    {
        _runtimeOptions.visible() = value;

        for( NodeObserverSet::iterator i = _nodeSet.begin(); i != _nodeSet.end(); ++i )
        {
            if ( i->valid() )
            {
                i->get()->setNodeMask( value ? ~0 : 0 );
            }
        }

        fireCallback( &ModelLayerCallback::onVisibleChanged );
    }
}

void
ModelLayer::setLightingEnabled( bool value )
{
    _runtimeOptions.lightingEnabled() = value;

    for( NodeObserverSet::iterator i = _nodeSet.begin(); i != _nodeSet.end(); ++i )
    {
        if ( i->valid() )
        {
            osg::StateSet* stateset = i->get()->getOrCreateStateSet();

            stateset->setMode( 
                GL_LIGHTING, value ? osg::StateAttribute::ON : 
                (osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED) );

            if ( Registry::capabilities().supportsGLSL() )
            {
                stateset->addUniform( Registry::shaderFactory()->createUniformForGLMode(
                    GL_LIGHTING, value ) );
            }
        }
    }
}

bool
ModelLayer::isLightingEnabled() const
{
    return *_runtimeOptions.lightingEnabled();
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
