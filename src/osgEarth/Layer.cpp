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
#include <osgEarth/Layer>
#include <osgEarth/Registry>
#include <osgEarth/ShaderLoader>
#include <osgDB/Registry>
#include <osgUtil/CullVisitor>

using namespace osgEarth;

#define LC "[Layer] Layer \"" << getName() << "\" "

//.................................................................

LayerOptions::LayerOptions() :
ConfigOptions()
{
    fromConfig(_conf);
}

LayerOptions::LayerOptions(const ConfigOptions& co) :
ConfigOptions(co)
{
    fromConfig(_conf);
}

Config LayerOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();
    conf.set("name", _name);
    conf.set("enabled", _enabled);
    conf.set("cacheid", _cacheId);
    if (_cachePolicy.isSet() && !_cachePolicy->empty())
        conf.setObj("cache_policy", _cachePolicy);
    conf.set("shader_define", _shaderDefine);
    conf.set("shader", _shader);
    return conf;
}

void LayerOptions::fromConfig(const Config& conf)
{
    _enabled.init(true);

    conf.getIfSet("name", _name);
    conf.getIfSet("enabled", _enabled);
    conf.getIfSet("cache_id", _cacheId); // compat
    conf.getIfSet("cacheid", _cacheId);
    conf.getObjIfSet("cache_policy", _cachePolicy);

    // legacy support:
    if (!_cachePolicy.isSet())
    {
        if ( conf.value<bool>( "cache_only", false ) == true )
            _cachePolicy->usage() = CachePolicy::USAGE_CACHE_ONLY;
        if ( conf.value<bool>( "cache_enabled", true ) == false )
            _cachePolicy->usage() = CachePolicy::USAGE_NO_CACHE;
    }
    conf.getIfSet("shader_define", _shaderDefine);
    conf.getIfSet("shader", _shader);
}

void LayerOptions::mergeConfig(const Config& conf)
{
    ConfigOptions::mergeConfig(conf);
    fromConfig(_conf);
}

//.................................................................

Layer::Layer() :
_options(&_optionsConcrete)
{
    _uid = osgEarth::Registry::instance()->createUID();
    _renderType = RENDERTYPE_NONE;
    _status = Status::OK();

    init();
}

Layer::Layer(LayerOptions* optionsPtr) :
_options(optionsPtr? optionsPtr : &_optionsConcrete)
{
    _uid = osgEarth::Registry::instance()->createUID();
    _renderType = RENDERTYPE_NONE;
    _status = Status::OK();
}

Layer::~Layer()
{
    OE_DEBUG << LC << "~Layer\n";
}

void
Layer::setReadOptions(const osgDB::Options* options)
{
    _readOptions = Registry::cloneOrCreateOptions(options);
}

Config
Layer::getConfig() const
{
    return options().getConfig();
}

bool
Layer::getEnabled() const
{
    return (options().enabled() == true) && getStatus().isOK();
}

void
Layer::init()
{
    // Copy the layer options name into the Object name.
    // This happens here AND in open.
    if (options().name().isSet())
    {
        osg::Object::setName(options().name().get());
    }
}

const Status&
Layer::open()
{
    // Copy the layer options name into the Object name.
    if (options().name().isSet())
    {
        osg::Object::setName(options().name().get());
    }
    
    // Install any shader #defines
    if (options().shaderDefine().isSet() && !options().shaderDefine()->empty())
    {
        OE_INFO << LC << "Setting shader define " << options().shaderDefine().get() << "\n";
        getOrCreateStateSet()->setDefine(options().shaderDefine().get());
    }

    // Load any user defined shaders
    if (options().shader().isSet() && !options().shader()->empty())
    {
        OE_INFO << LC << "Installing inline shader code\n";
        VirtualProgram* vp = VirtualProgram::getOrCreate(this->getOrCreateStateSet());
        ShaderPackage package;
        package.add("", options().shader().get());
        package.loadAll(vp, getReadOptions());
    }

    return _status;
}

void
Layer::setName(const std::string& name)
{
    osg::Object::setName(name);
    options().name() = name;
}

const char*
Layer::getTypeName() const
{
    return typeid(*this).name();
}

#define LAYER_OPTIONS_TAG "osgEarth.LayerOptions"

Layer*
Layer::create(const ConfigOptions& options)
{
    return create(options.getConfig().key(), options);
}

Layer*
Layer::create(const std::string& name, const ConfigOptions& options)
{
    if ( name.empty() )
    {
        OE_WARN << "[Layer] ILLEGAL- Layer::create requires a plugin name" << std::endl;
        return 0L;
    }

    // convey the configuration options:
    osg::ref_ptr<osgDB::Options> dbopt = Registry::instance()->cloneOrCreateOptions();
    dbopt->setPluginData( LAYER_OPTIONS_TAG, (void*)&options );

    //std::string pluginExtension = std::string( ".osgearth_" ) + name;
    std::string pluginExtension = std::string( "." ) + name;

    // use this instead of osgDB::readObjectFile b/c the latter prints a warning msg.
    osgDB::ReaderWriter::ReadResult rr = osgDB::Registry::instance()->readObject( pluginExtension, dbopt.get() );
    if ( !rr.validObject() || rr.error() )
    {
        // quietly fail so we don't get tons of msgs.
        return 0L;
    }

    Layer* layer = dynamic_cast<Layer*>( rr.getObject() );
    if ( layer == 0L )
    {
        // TODO: communicate an error somehow
        return 0L;
    }

    if (layer->getName().empty())
        layer->setName(name);

    rr.takeObject();
    return layer;
}

const ConfigOptions&
Layer::getConfigOptions(const osgDB::Options* options)
{
    static ConfigOptions s_default;
    const void* data = options->getPluginData(LAYER_OPTIONS_TAG);
    return data ? *static_cast<const ConfigOptions*>(data) : s_default;
}

void
Layer::addCallback(LayerCallback* cb)
{
    _callbacks.push_back( cb );
}

void
Layer::removeCallback(LayerCallback* cb)
{
    CallbackVector::iterator i = std::find( _callbacks.begin(), _callbacks.end(), cb );
    if ( i != _callbacks.end() ) 
        _callbacks.erase( i );
}

bool
Layer::preCull(osgUtil::CullVisitor* cv) const
{
    //if (getStateSet())
    //    cv->pushStateSet(getStateSet());
    return true;
}

void
Layer::postCull(osgUtil::CullVisitor* cv) const
{
    //if (getStateSet())
    //    cv->popStateSet();
}
