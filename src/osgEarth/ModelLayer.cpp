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

ModelLayerOptions::ModelLayerOptions() :
VisibleLayerOptions()
{
    setDefaults();
    fromConfig(_conf);
}

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
    if (!driverOptions.empty())
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
}

ModelLayerOptions& ModelLayerOptions::operator =(const ModelLayerOptions& rhs)
{
    VisibleLayerOptions::operator =(rhs);
    _driver = optional<ModelSourceOptions>(rhs._driver);
    _lighting = optional<bool>(rhs._lighting);
    _maskOptions = optional<MaskSourceOptions>(rhs._maskOptions);
    _maskMinLevel = optional<unsigned>(rhs._maskMinLevel);

    return *this;
}

void
ModelLayerOptions::setDefaults()
{
    _lighting.init    ( true );
    _maskMinLevel.init( 0 );
}

Config
ModelLayerOptions::getConfig() const
{
    Config conf = VisibleLayerOptions::getConfig();
    conf.key() = "model";

    conf.set( "name",           _name );
    conf.set( "lighting",       _lighting );
    conf.set( "mask_min_level", _maskMinLevel );

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
_optionsConcrete(ModelLayerOptions())
{
    options().name() = name;
    init();
    if (node)
    {
        _root->addChild(node);
        setStatus(Status::OK());
    }
}

ModelLayer::~ModelLayer()
{
    //nop
}

Config
ModelLayer::getConfig() const
{
    Config layerConf = Layer::getConfig();
    layerConf.key() = "model";
    layerConf.set("driver", options().driver()->getDriver());
    return layerConf;
}

void
ModelLayer::init()
{
    VisibleLayer::init();
    _root = new osg::Group();
    _root->setName(getName());
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

void
ModelLayer::addedToMap(const Map* map)
{
    if (getStatus().isError())
        return;

    if ( _modelSource.valid() )
    {
        // reset the scene graph.
        while (_root->getNumChildren() > 0)
        {
            getSceneGraphCallbacks()->fireRemoveNode(_root->getChild(0));
            _root->removeChildren(0, 1);
        }

        // Share the scene graph callbacks with the model source:
        _modelSource->setSceneGraphCallbacks(getSceneGraphCallbacks());

        // Create the scene graph from the mode source:
        osg::ref_ptr<osg::Node> node = _modelSource->createNode(map, 0L);
        if (node.valid())
        {
            if ( options().lightingEnabled().isSet() )
            {
                setLightingEnabledNoLock( options().lightingEnabled().get() );
            }

            _modelSource->sync( _modelSourceRev );

            // Handle disabling depth testing
            if ( _modelSource->getOptions().depthTestEnabled() == false )
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ) );              
                ss->setRenderBinDetails( 99999, "RenderBin" ); //TODO: configure this bin ...
            }

            // enfore a rendering bin if necessary:
            if (_modelSource->getOptions().renderOrder().isSet())
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setRenderBinDetails(
                    _modelSource->getOptions().renderOrder().value(),
                    ss->getBinName().empty() ? "DepthSortedBin" : ss->getBinName());
            }

            if (_modelSource->getOptions().renderBin().isSet())
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setRenderBinDetails(
                    ss->getBinNumber(),
                    _modelSource->getOptions().renderBin().get());
            }

            _root->addChild(node.get());
        }
    }
}

osg::Node*
ModelLayer::getOrCreateNode()
{
    return _root.get();
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

#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
            stateset->setMode( 
                GL_LIGHTING, value ? osg::StateAttribute::ON : 
                (osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED) );
#endif

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
