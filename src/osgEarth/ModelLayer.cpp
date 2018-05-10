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
#include <osgEarth/Lighting>
#include <osg/Depth>

#define LC "[ModelLayer] Layer \"" << getName() << "\" "

using namespace osgEarth;

namespace osgEarth {
    REGISTER_OSGEARTH_LAYER(model, ModelLayer);
}

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

ModelLayerOptions::ModelLayerOptions(const ConfigOptions& options) :
VisibleLayerOptions( options )
{
    setDefaults();
    fromConfig( _conf ); 
}

ModelLayerOptions::ModelLayerOptions(const std::string& in_name, const ModelSourceOptions& driverOptions) :
VisibleLayerOptions()
{
    setDefaults();
    fromConfig( _conf );
    _driver = driverOptions;
    name() = in_name;
}

ModelLayerOptions::ModelLayerOptions(const ModelLayerOptions& rhs) :
VisibleLayerOptions(rhs)
{
    _driver = optional<ModelSourceOptions>(rhs._driver);
    _lighting = optional<bool>(rhs._lighting);
    _maskOptions = optional<MaskSourceOptions>(rhs._maskOptions);
    _maskMinLevel = optional<unsigned>(rhs._maskMinLevel);
    _terrainPatch = optional<bool>(rhs._terrainPatch);
}

ModelLayerOptions& ModelLayerOptions::operator =(const ModelLayerOptions& rhs)
{
    VisibleLayerOptions::operator =(rhs);
    _driver = optional<ModelSourceOptions>(rhs._driver);
    _lighting = optional<bool>(rhs._lighting);
    _maskOptions = optional<MaskSourceOptions>(rhs._maskOptions);
    _maskMinLevel = optional<unsigned>(rhs._maskMinLevel);
    _terrainPatch = optional<bool>(rhs._terrainPatch);

    return *this;
}

void
ModelLayerOptions::setDefaults()
{
    _lighting.init    ( true );
    _maskMinLevel.init( 0 );
    _terrainPatch.init( false );
}

Config
ModelLayerOptions::getConfig() const
{
    Config conf = VisibleLayerOptions::getConfig();
    conf.key() = "model";

    conf.set( "name",           _name );
    conf.set( "lighting",       _lighting );
    conf.set( "mask_min_level", _maskMinLevel );
    conf.set( "patch",          _terrainPatch );  

    // Merge the MaskSource options
    if ( mask().isSet() )
        conf.set( "mask", mask()->getConfig() );

    return conf;
}

void
ModelLayerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "lighting",       _lighting );
    conf.getIfSet( "mask_min_level", _maskMinLevel );
    conf.getIfSet( "patch",          _terrainPatch );

    if ( conf.hasValue("driver") )
        driver() = ModelSourceOptions(conf);

    if ( conf.hasChild("mask") )
        mask() = MaskSourceOptions(conf.child("mask"));
}

void
ModelLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

ModelLayer::ModelLayer() :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

ModelLayer::ModelLayer(const ModelLayerOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

ModelLayer::ModelLayer(const std::string& name, const ModelSourceOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(ModelLayerOptions(name, options))
{
    init();
}

ModelLayer::ModelLayer(const ModelLayerOptions& options, ModelSource* source) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options),
_modelSource( source )
{
    init();
}

ModelLayer::ModelLayer(const std::string& name, osg::Node* node) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(ModelLayerOptions(name)),
_modelSource( new NodeModelSource(node) )
{
    init();
}

ModelLayer::~ModelLayer()
{
    //nop
}

Config
ModelLayer::getConfig() const
{
    Config layerConf = Layer::getConfig(); //getModelLayerOptions().getConfig();
    layerConf.set("name", getName()); // redundant?
    layerConf.set("driver", options().driver()->getDriver());
    layerConf.key() = "model";
    return layerConf;
}

void
ModelLayer::init()
{
    VisibleLayer::init();
    _sgCallbacks = new SceneGraphCallbacks();
}

const Status&
ModelLayer::open()
{
    if ( !_modelSource.valid() && options().driver().isSet() )
    {
        std::string driverName = options().driver()->getDriver();

        OE_INFO << LC << "Opening; driver=\"" << driverName << "\"" << std::endl;
        
        // Try to create the model source:
        _modelSource = ModelSourceFactory::create( options().driver().get() );
        if ( _modelSource.valid() )
        {
            _modelSource->setName( this->getName() );

            const Status& modelStatus = _modelSource->open( _readOptions.get() );
            if (modelStatus.isOK())
            {
                // the mask, if there is one:
                if ( !_maskSource.valid() && options().mask().isSet() )
                {
                    OE_INFO << LC << "...initializing mask, driver=" << driverName << std::endl;

                    _maskSource = MaskSourceFactory::create( options().mask().get() );
                    if ( _maskSource.valid() )
                    {
                        const Status& maskStatus = _maskSource->open(_readOptions.get());
                        if (maskStatus.isError())
                        {
                            setStatus(maskStatus);
                        }
                    }
                    else
                    {
                        setStatus(Status::Error(Status::ServiceUnavailable, Stringify() << "Cannot find mask driver \"" << options().mask()->getDriver() << "\""));
                    }
                }
            }
            else
            {
                // propagate the model source's error status
                setStatus(modelStatus);
            }
        }
        else
        {
            setStatus(Status::Error(Status::ServiceUnavailable, Stringify() << "Failed to create driver \"" << driverName << "\""));
        }
    }

    return getStatus();
}

#if 0
void
ModelLayer::setReadOptions(const osgDB::Options* readOptions)
{
    Layer::setReadOptions(readOptions);

    // Create some local cache settings for this layer:
    CacheSettings* oldSettings = CacheSettings::get(readOptions);
    _cacheSettings = oldSettings ? new CacheSettings(*oldSettings) : new CacheSettings();

    // bring in the new policy for this layer if there is one:
    _cacheSettings->integrateCachePolicy(options().cachePolicy());

    // if caching is a go, install a bin.
    if (_cacheSettings->isCacheEnabled())
    {
        std::string binID;
        if (options().cacheId().isSet() && !options().cacheId()->empty())
        {
            binID = options().cacheId().get();
        }
        else
        {
            Config conf = options().driver()->getConfig();
            binID = hashToString(conf.toJSON(false));
        }

        // make our cacheing bin!
        CacheBin* bin = _cacheSettings->getCache()->addBin(binID);
        if (bin)
        {
            OE_INFO << LC << "Cache bin is [" << binID << "]\n";
            _cacheSettings->setCacheBin( bin );
        }
        else
        {
            // failed to create the bin, so fall back on no cache mode.
            OE_WARN << LC << "Failed to open a cache bin [" << binID << "], disabling caching\n";
            _cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
        }
    }

    // Store it for further propagation!
    _cacheSettings->store(_readOptions.get());
}
#endif

std::string
ModelLayer::getCacheID() const
{
    // Customized from the base class to use the driver() config instead of full config:
    std::string binID;
    if (options().cacheId().isSet() && !options().cacheId()->empty())
    {
        binID = options().cacheId().get();
    }
    else
    {
        Config conf = options().driver()->getConfig();
        binID = hashToString(conf.toJSON(false));
    }
    return binID;
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
        _modelSource->setSceneGraphCallbacks(_sgCallbacks.get());

        node = _modelSource->createNode( map, progress );

        if ( node )
        {
            if ( options().lightingEnabled().isSet() )
            {
                setLightingEnabledNoLock( options().lightingEnabled().get() );
            }

            _modelSource->sync( _modelSourceRev );


            // add a parent group for shaders/effects to attach to without overwriting any model programs directly
            osg::Group* group = new osg::Group();
            group->addChild(node);

            // assign the layer's stateset to the group.
            osg::StateSet* groupSS = getOrCreateStateSet();
            group->setStateSet(groupSS);

            node = group;

            // Toggle visibility if necessary
            if ( options().visible().isSet() )
            {
                node->setNodeMask( options().visible().get() ? ~0 : 0 );
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

osg::Node*
ModelLayer::getOrCreateNode()
{
    if (!_graphs.empty())
        return _graphs.begin()->second.get();
    else
        return 0L;
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
    options().lightingEnabled() = value;

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
                stateset->setDefine(OE_LIGHTING_DEFINE, value);
            }
        }
    }
}

bool
ModelLayer::isLightingEnabled() const
{
    return options().lightingEnabled().get();
}

void
ModelLayer::fireCallback(ModelLayerCallback::MethodPtr method)
{
    for(CallbackVector::const_iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
    {
        ModelLayerCallback* cb = dynamic_cast<ModelLayerCallback*>(i->get());
        if (cb) (cb->*method)( this );
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
