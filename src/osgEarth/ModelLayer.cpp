/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

        Status initialize(const osgDB::Options* readOptions) {
            return Status::OK();
        }

        osg::Node* createNodeImplementation(const Map* map, ProgressCallback* progress) {
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

ModelLayerOptions::ModelLayerOptions(const ModelLayerOptions& rhs) :
ConfigOptions(rhs.getConfig())
{
    _name = optional<std::string>(rhs._name);
    _driver = optional<ModelSourceOptions>(rhs._driver);
    _enabled = optional<bool>(rhs._enabled);
    _visible = optional<bool>(rhs._visible);
    _opacity = optional<float>(rhs._opacity);
    _lighting = optional<bool>(rhs._lighting);
    _maskOptions = optional<MaskSourceOptions>(rhs._maskOptions);
    _maskMinLevel = optional<unsigned>(rhs._maskMinLevel);
    _terrainPatch = optional<bool>(rhs._terrainPatch);
    _cachePolicy = optional<CachePolicy>(rhs._cachePolicy);
    _cacheId = optional<std::string>(rhs._cacheId);
}

ModelLayerOptions& ModelLayerOptions::operator =(const ModelLayerOptions& rhs)
{
    ConfigOptions::operator =(rhs);

    _name = optional<std::string>(rhs._name);
    _driver = optional<ModelSourceOptions>(rhs._driver);
    _enabled = optional<bool>(rhs._enabled);
    _visible = optional<bool>(rhs._visible);
    _opacity = optional<float>(rhs._opacity);
    _lighting = optional<bool>(rhs._lighting);
    _maskOptions = optional<MaskSourceOptions>(rhs._maskOptions);
    _maskMinLevel = optional<unsigned>(rhs._maskMinLevel);
    _terrainPatch = optional<bool>(rhs._terrainPatch);
    _cachePolicy = optional<CachePolicy>(rhs._cachePolicy);
    _cacheId = optional<std::string>(rhs._cacheId);

    return *this;
}

void
ModelLayerOptions::setDefaults()
{
    _enabled.init     ( true );
    _visible.init     ( true );
    _lighting.init    ( true );
    _opacity.init     ( 1.0f );
    _maskMinLevel.init( 0 );
    _terrainPatch.init( false );

    // Expressly set it here since we want no caching by default on a model layer.
    _cachePolicy = CachePolicy::NO_CACHE;
}

Config
ModelLayerOptions::getConfig() const
{
    Config conf = ConfigOptions::newConfig();

    conf.updateIfSet( "name",           _name );
    conf.updateIfSet( "enabled",        _enabled );
    conf.updateIfSet( "visible",        _visible );
    conf.updateIfSet( "lighting",       _lighting );
    conf.updateIfSet( "opacity",        _opacity );
    conf.updateIfSet( "mask_min_level", _maskMinLevel );
    conf.updateIfSet( "patch",          _terrainPatch );  

    conf.updateObjIfSet( "cache_policy", _cachePolicy );  
    conf.updateIfSet("cacheid", _cacheId);

    // Merge the ModelSource options
    if ( driver().isSet() )
        conf.merge( driver()->getConfig() );

    // Merge the MaskSource options
    if ( maskOptions().isSet() )
        conf.add( "mask", maskOptions()->getConfig() );

    return conf;
}

void
ModelLayerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "name",           _name );
    conf.getIfSet( "enabled",        _enabled );
    conf.getIfSet( "visible",        _visible );
    conf.getIfSet( "lighting",       _lighting );
    conf.getIfSet( "opacity",        _opacity );
    conf.getIfSet( "mask_min_level", _maskMinLevel );
    conf.getIfSet( "patch",          _terrainPatch );

    conf.getObjIfSet( "cache_policy", _cachePolicy );  
    conf.getIfSet("cacheid", _cacheId);

    if ( conf.hasValue("driver") )
        driver() = ModelSourceOptions(conf);

    if ( conf.hasChild("mask") )
        maskOptions() = MaskSourceOptions(conf.child("mask"));
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
    //nop
}

void
ModelLayer::copyOptions()
{
    _runtimeOptions = _initOptions;

    _alphaEffect = new AlphaEffect();
    _alphaEffect->setAlpha( *_initOptions.opacity() );
}

const Status&
ModelLayer::open()
{
    if ( !_modelSource.valid() && _initOptions.driver().isSet() )
    {
        std::string driverName = _initOptions.driver()->getDriver();

        OE_INFO << LC << "Opening model layer \"" << getName() << "\", driver=\"" << driverName << "\"" << std::endl;
        
        // Try to create the model source:
        _modelSource = ModelSourceFactory::create( *_initOptions.driver() );
        if ( _modelSource.valid() )
        {
            _modelSource->setName( this->getName() );

            const Status& modelStatus = _modelSource->open( _readOptions.get() );
            if (modelStatus.isOK())
            {
                // the mask, if there is one:
                if ( !_maskSource.valid() && _initOptions.maskOptions().isSet() )
                {
                    OE_INFO << LC << "...initializing mask, driver=" << driverName << std::endl;

                    _maskSource = MaskSourceFactory::create( *_initOptions.maskOptions() );
                    if ( _maskSource.valid() )
                    {
                        const Status& maskStatus = _maskSource->open(_readOptions.get());
                        if (maskStatus.isError())
                        {
                            _status = maskStatus;
                        }
                    }
                    else
                    {
                        _status = Status::Error(Status::ServiceUnavailable, Stringify() << "Cannot find mask driver \"" << _initOptions.maskOptions()->getDriver() << "\"");
                    }
                }
            }
            else
            {
                // propagate the model source's error status
                _status = modelStatus;
            }
        }
        else
        {
            _status = Status::Error(Status::ServiceUnavailable, Stringify() << "Failed to create driver \"" << driverName << "\"");
        }
    }

    return _status;
}

void
ModelLayer::setReadOptions(const osgDB::Options* readOptions)
{
    _readOptions = Registry::cloneOrCreateOptions(readOptions);

    // Create some local cache settings for this layer:
    CacheSettings* oldSettings = CacheSettings::get(readOptions);
    _cacheSettings = oldSettings ? new CacheSettings(*oldSettings) : new CacheSettings();

    // bring in the new policy for this layer if there is one:
    _cacheSettings->integrateCachePolicy(_initOptions.cachePolicy());

    // if caching is a go, install a bin.
    if (_cacheSettings->isCacheEnabled())
    {
        std::string binID;
        if (_initOptions.cacheId().isSet() && !_initOptions.cacheId()->empty())
        {
            binID = _initOptions.cacheId().get();
        }
        else
        {
            Config conf = _initOptions.driver()->getConfig();
            binID = hashToString(conf.toJSON(false));
        }

        // make our cacheing bin!
        CacheBin* bin = _cacheSettings->getCache()->addBin(binID);
        if (bin)
        {
            OE_INFO << LC << "Layer \"" << getName() << "\" cache bin is [" << binID << "]\n";
            _cacheSettings->setCacheBin( bin );
        }
        else
        {
            // failed to create the bin, so fall back on no cache mode.
            OE_WARN << LC << "Layer " << getName() << " failed to open a cache bin [" << binID << "], disabling caching\n";
            _cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
        }
    }

    // Store it for further propagation!
    _cacheSettings->store(_readOptions.get());
}

osg::Node*
ModelLayer::getSceneGraph(const UID& mapUID) const
{
    Threading::ScopedMutexLock lock(_mutex);
    Graphs::const_iterator i = _graphs.find( mapUID );
    return i == _graphs.end() ? 0L : i->second.get();
}

osg::Node*
ModelLayer::getOrCreateSceneGraph(const Map*        map,
                                  ProgressCallback* progress )
{
    if (getStatus().isError())
        return 0L;

    // exclusive lock for cache lookup/update.
    Threading::ScopedMutexLock lock( _mutex );

    // There can be one node graph per Map. See if it already exists
    // and if so, return it.
    Graphs::iterator i = _graphs.find(map->getUID());
    if ( i != _graphs.end() && i->second.valid() )
        return i->second.get();

    // need to create it.
    osg::Node* node = 0L;

    if ( _modelSource.valid() )
    {
        node = _modelSource->createNode( map, progress );

        if ( node )
        {
            if ( _runtimeOptions.lightingEnabled().isSet() )
            {
                setLightingEnabledNoLock( *_runtimeOptions.lightingEnabled() );
            }

            _modelSource->sync( _modelSourceRev );


            // add a parent group for shaders/effects to attach to without overwriting any model programs directly
            osg::Group* group = new osg::Group();
            group->addChild(node);
            _alphaEffect->attach( group->getOrCreateStateSet() );
            node = group;

            // Toggle visibility if necessary
            if ( _runtimeOptions.visible().isSet() )
            {
                node->setNodeMask( *_runtimeOptions.visible() ? ~0 : 0 );
            }

            // Handle disabling depth testing
            if ( _modelSource->getOptions().depthTestEnabled() == false )
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ) );
                ss->setRenderBinDetails( 99999, "RenderBin" ); //TODO: configure this bin ...
            }

            // save it.
            _graphs[map->getUID()] = node;
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

        _mutex.lock();
        for(Graphs::iterator i = _graphs.begin(); i != _graphs.end(); ++i)
        {
            if ( i->second.valid() )
                i->second->setNodeMask( value ? ~0 : 0 );
        }
        _mutex.unlock();

        fireCallback( &ModelLayerCallback::onVisibleChanged );
    }
}

float
ModelLayer::getOpacity() const
{
    return *_runtimeOptions.opacity();
}

void
ModelLayer::setOpacity(float opacity)
{
    if ( _runtimeOptions.opacity() != opacity )
    {
        _runtimeOptions.opacity() = opacity;

        _alphaEffect->setAlpha(opacity);

        fireCallback( &ModelLayerCallback::onOpacityChanged );
    }
}

void
ModelLayer::setLightingEnabled( bool value )
{
    Threading::ScopedMutexLock lock(_mutex);
    setLightingEnabledNoLock( value );
}

void
ModelLayer::setLightingEnabledNoLock(bool value)
{
    _runtimeOptions.lightingEnabled() = value;

    for(Graphs::iterator i = _graphs.begin(); i != _graphs.end(); ++i)
    {
        if ( i->second.valid() )
        {
            osg::StateSet* stateset = i->second->getOrCreateStateSet();

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


osg::Vec3dArray*
ModelLayer::getOrCreateMaskBoundary(float                   heightScale,
                                    const SpatialReference* srs, 
                                    ProgressCallback*       progress )
{
    if (_maskSource.valid() && !_maskBoundary.valid() && getStatus().isOK())
    {
        Threading::ScopedMutexLock excl(_mutex);

        if ( !_maskBoundary.valid() ) // double-check pattern
        {
            // make the geometry:
            _maskBoundary = _maskSource->createBoundary( srs, progress );
            if (_maskBoundary.valid())
            {
                // scale to the height scale factor:
                for (osg::Vec3dArray::iterator vIt = _maskBoundary->begin(); vIt != _maskBoundary->end(); ++vIt)
                    vIt->z() = vIt->z() * heightScale;
            }
            else
            {
                setStatus(Status::Error("Failed to create masking boundary"));
            }
        }
    }

    return _maskBoundary.get();
}
