/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/Cache>
#include <osgEarth/Registry>
#include <osgEarth/SceneGraphCallback>
#include <osgEarth/ShaderLoader>
#include <osgEarth/TileKey>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainResources>
#include <osg/StateSet>

using namespace osgEarth;

#define LC "[Layer] Layer \"" << getName() << "\" "

//.................................................................

Config
Layer::Options::getConfig() const
{
    Config conf = ConfigOptions::getConfig();
    conf.set("name", name());
    conf.set("enabled", enabled());
    conf.set("cacheid", cacheId());
    if (cachePolicy().isSet() && !cachePolicy()->empty())
        conf.set("cache_policy", cachePolicy());
    conf.set("shader_define", shaderDefine());
    conf.set("attribution", attribution());
    conf.set("terrain", terrainPatch());
    conf.set("proxy", _proxySettings );
    conf.set("osg_options", osgOptionString());
    conf.set("l2_cache_size", l2CacheSize());

    for(std::vector<ShaderOptions>::const_iterator i = shaders().begin();
        i != shaders().end();
        ++i)
    {
        conf.add("shader", i->getConfig());
    }

    return conf;
}

void
Layer::Options::fromConfig(const Config& conf)
{
    // defaults:
    _enabled.init(true);
    _terrainPatch.init(false);

    conf.get("name", name());
    conf.get("enabled", enabled());
    conf.get("cache_id", cacheId()); // compat
    conf.get("cacheid", cacheId());
    conf.get("attribution", attribution());
    conf.get("cache_policy", cachePolicy());
    conf.get("l2_cache_size", l2CacheSize());

    // legacy support:
    if (!cachePolicy().isSet())
    {
        if ( conf.value<bool>( "cache_only", false ) == true )
            _cachePolicy->usage() = CachePolicy::USAGE_CACHE_ONLY;
        if ( conf.value<bool>( "cache_enabled", true ) == false )
            _cachePolicy->usage() = CachePolicy::USAGE_NO_CACHE;
        if (conf.value<bool>("caching", true) == false)
            _cachePolicy->usage() = CachePolicy::USAGE_NO_CACHE;
    }
    conf.get("shader_define", shaderDefine());

    const ConfigSet& shadersConf = conf.children("shader");
    for(ConfigSet::const_iterator i = shadersConf.begin(); i != shadersConf.end(); ++i)
        shaders().push_back(ShaderOptions(*i));

    conf.get("terrain", terrainPatch());
    conf.get("patch", terrainPatch());
    conf.get("proxy", _proxySettings );
    conf.get("osg_options", osgOptionString());
}

//.................................................................

void
Layer::TraversalCallback::traverse(osg::Node* node, osg::NodeVisitor* nv) const
{
    node->accept(*nv);
}

//.................................................................

Layer::Layer() :
_options(&_optionsConcrete),
_revision(1),
_mutex(NULL),
_layerName(osg::Object::_name) // for the debugger
{
    init();
}

Layer::Layer(Layer::Options* optionsPtr) :
_options(optionsPtr? optionsPtr : &_optionsConcrete),
_revision(1),
_mutex(NULL),
_layerName(osg::Object::_name) // for the debugger
{
    // init() will be called by base class
}

Layer::Layer(const Layer& rhs, const osg::CopyOp& op) :
    osg::Object(rhs, op),
    _layerName(osg::Object::_name)
{
    //nop
}

Layer::~Layer()
{
    OE_DEBUG << LC << "~Layer\n";
    if (_mutex)
        delete _mutex;
}

void
Layer::dirty()
{
    bumpRevision();
}

void
Layer::bumpRevision()
{
    ++_revision;
}

void
Layer::setReadOptions(const osgDB::Options* readOptions)
{
    // We are storing _cacheSettings both in the Read Options AND
    // as a class member. This is probably not strictly necessary
    // but we will keep the ref in the Layer just to be on the safe
    // side - gw

    _readOptions = Registry::cloneOrCreateOptions(readOptions);

    // store the referrer for relative-path resolution
    URIContext(options().referrer()).store(_readOptions.get());

    //Store the proxy settings in the options structure.
    if (options().proxySettings().isSet())
    {
        options().proxySettings()->apply(_readOptions.get());
    }

    if (options().osgOptionString().isSet())
    {
        _readOptions->setOptionString(
            options().osgOptionString().get() + " " +
            _readOptions->getOptionString());
    }
}

const osgDB::Options*
Layer::getReadOptions() const
{
    return _readOptions.get();
}

void
Layer::setCacheID(const std::string& value)
{
    _runtimeCacheId = "";
    setOptionThatRequiresReopen(options().cacheId(), value);
}

std::string
Layer::getCacheID() const
{
    // create the unique cache ID for the cache bin.
    if (_runtimeCacheId.empty() == false)
    {
        return _runtimeCacheId;
    }
    else if (options().cacheId().isSet() && !options().cacheId()->empty())
    {
        // user expliticy set a cacheId in the terrain layer options.
        // this appears to be a NOP; review for removal -gw
        return options().cacheId().get();
    }
    else
    {
        // system will generate a cacheId from the layer configuration.
        Config hashConf = options().getConfig();

        // remove non-data properties.
        // TODO: move these a virtual function called
        // getNonDataProperties() or something.
        hashConf.remove("accept_draping");
        hashConf.remove("altitude");
        hashConf.remove("async");
        hashConf.remove("attenuation_range");
        hashConf.remove("attribution");
        hashConf.remove("blend");
        hashConf.remove("cacheid");
        hashConf.remove("cache_id");
        hashConf.remove("cache_enabled");
        hashConf.remove("cache_only");
        hashConf.remove("cache_policy");
        hashConf.remove("caching");
        hashConf.remove("color_filters");
        hashConf.remove("enabled");
        hashConf.remove("fid_attribute");
        hashConf.remove("geo_interpolation");
        hashConf.remove("l2_cache_size");
        hashConf.remove("max_data_level");
        hashConf.remove("max_filter");
        hashConf.remove("max_level");
        hashConf.remove("max_range");
        hashConf.remove("min_filter");
        hashConf.remove("min_level");
        hashConf.remove("min_range");
        hashConf.remove("name");
        hashConf.remove("open_write");
        hashConf.remove("proxy");
        hashConf.remove("rewind_polygons");
        hashConf.remove("shader");
        hashConf.remove("shaders");
        hashConf.remove("shader_define");
        hashConf.remove("shared");
        hashConf.remove("shared_sampler");
        hashConf.remove("shared_matrix");
        hashConf.remove("terrain");
        hashConf.remove("texture_compression");
        hashConf.remove("visible");

        unsigned hash = osgEarth::hashString(hashConf.toJSON());
        std::stringstream buf;
        const char hyphen = '-';
        if (getName().empty() == false)
            buf << toLegalFileName(getName(), false, &hyphen) << hyphen;
        buf << std::hex << std::setw(8) << std::setfill('0') << hash;
        return buf.str();
    }
}

Config
Layer::getConfig() const
{
    Config conf = options().getConfig();
    conf.key() = getConfigKey();
    return conf;
}

bool
Layer::getEnabled() const
{
    return (options().enabled() == true);
}

void
Layer::setEnabled(bool value)
{
    if (options().enabled() != value)
    {
        options().enabled() = value;
        if (value == false && isOpen())
        {
            close();
            _status.set(Status::ResourceUnavailable, "Layer disabled");
        }
    }
}

const Status&
Layer::setStatus(const Status& status) const
{
    _status = status;
    return _status;
}

const Status&
Layer::setStatus(const Status::Code& code, const std::string& message) const
{
    return setStatus(Status(code, message));
}

void
Layer::setCachePolicy(const CachePolicy& value)
{
    options().cachePolicy() = value;

    if (_cacheSettings.valid())
    {
        _cacheSettings->integrateCachePolicy(options().cachePolicy());
    }
}

const CachePolicy&
Layer::getCachePolicy() const
{
    return options().cachePolicy().get();
}

void
Layer::init()
{
    _uid = osgEarth::Registry::instance()->createUID();
    _renderType = RENDERTYPE_NONE;
    _status.set(Status::ResourceUnavailable, getEnabled() ? "Layer closed" : "Layer disabled");
    _isClosing = false;

    // For detecting scene graph changes at runtime
    _sceneGraphCallbacks = new SceneGraphCallbacks(this);

    // Copy the layer options name into the Object name.
    // This happens here AND in open.
    if (options().name().isSet())
    {
        osg::Object::setName(options().name().get());
    }

    _mutex = new Threading::Mutex(options().name().isSet() ? options().name().get() : "Unnamed Layer(OE)");
}

Status
Layer::open()
{
    // Cannot open a layer that's already open OR is disabled.
    if (isOpen() || !getEnabled())
    {
        return getStatus();
    }

    // be optimistic :)
    _status.set(Status::NoError);

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

    setStatus(openImplementation());

    if (isOpen())
    {
        fireCallback(&LayerCallback::onOpen);
    }

    return getStatus();
}

Status
Layer::open(const osgDB::Options* readOptions)
{
    setReadOptions(readOptions);
    return open();
}

Status
Layer::openImplementation()
{
    // Create some local cache settings for this layer.
    // There might be a CacheSettings object in the readoptions that
    // came from the map. If so, copy it.
    CacheSettings* oldSettings = CacheSettings::get(_readOptions.get());
    _cacheSettings = oldSettings ? new CacheSettings(*oldSettings) : new CacheSettings();

    // If the layer hints are set, integrate that cache policy next.
    _cacheSettings->integrateCachePolicy(layerHints().cachePolicy());

    // bring in the new policy for this layer if there is one:
    _cacheSettings->integrateCachePolicy(options().cachePolicy());

    // if caching is a go, install a bin.
    if (_cacheSettings->isCacheEnabled())
    {
        _runtimeCacheId = getCacheID();

        // make our cacheing bin!
        CacheBin* bin = _cacheSettings->getCache()->addBin(_runtimeCacheId);
        if (bin)
        {
            OE_INFO << LC << "Cache bin is [" << _runtimeCacheId << "]\n";
            _cacheSettings->setCacheBin(bin);
        }
        else
        {
            // failed to create the bin, so fall back on no cache mode.
            OE_WARN << LC << "Failed to open a cache bin [" << _runtimeCacheId << "], disabling caching\n";
            _cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
        }
    }

    // Store it for further propagation!
    _cacheSettings->store(_readOptions.get());

    return Status::OK();
}

Status
Layer::closeImplementation()
{
    _cacheSettings = NULL;
    _runtimeCacheId.clear();
    return Status::NoError;
}

Status
Layer::close()
{
    if (isOpen())
    {
        _isClosing = true;
        closeImplementation();
        _status.set(Status::ResourceUnavailable, "Layer closed");
        fireCallback(&LayerCallback::onClose);
        _isClosing = false;
    }
    return getStatus();
}

bool
Layer::isOpen() const
{
    return getStatus().isOK();
}

const Status&
Layer::getStatus() const
{
    return _status;
}

void
Layer::invoke_prepareForRendering(TerrainEngine* engine)
{
    prepareForRendering(engine);

    // deprecation path; call this in case some older layer is still
    // implementing it.
    setTerrainResources(engine->getResources());
}

void
Layer::prepareForRendering(TerrainEngine* engine)
{
    // Install an earth-file shader if necessary (once)
    for (const auto& shaderOptions : options().shaders())
    {
        LayerShader* shader = new LayerShader(shaderOptions);
        shader->install(this, engine->getResources());
        _shaders.emplace_back(shader);
    }
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
    std::string name = options.getConfig().key();

    if (name.empty())
        name = options.getConfig().value("driver");

    if ( name.empty() )
    {
        OE_WARN << "[Layer] ILLEGAL- Layer::create requires a valid driver name" << std::endl;
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

SceneGraphCallbacks*
Layer::getSceneGraphCallbacks() const
{
    return _sceneGraphCallbacks.get();
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

void
Layer::apply(osg::Node* node, osg::NodeVisitor* nv) const
{
    if (_traversalCallback.valid())
    {
        _traversalCallback->operator()(node, nv);
    }
    else
    {
        node->accept(*nv);
    }
}

void
Layer::setCullCallback(TraversalCallback* cb)
{
    _traversalCallback = cb;
}

const Layer::TraversalCallback*
Layer::getCullCallback() const
{
    return _traversalCallback.get();
}

const GeoExtent&
Layer::getExtent() const
{
    static GeoExtent s_invalid = GeoExtent::INVALID;
    return s_invalid;
}

DateTimeExtent
Layer::getDateTimeExtent() const
{
    return DateTimeExtent();
}

osg::StateSet*
Layer::getOrCreateStateSet()
{
    if (!_stateSet.valid())
    {
        _stateSet = new osg::StateSet();
        _stateSet->setName("Layer");
    }
    return _stateSet.get();
}

osg::StateSet*
Layer::getStateSet() const
{
    return _stateSet.get();
}

void
Layer::fireCallback(LayerCallback::MethodPtr method)
{
    for (CallbackVector::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
    {
        LayerCallback* cb = dynamic_cast<LayerCallback*>(i->get());
        if (cb) (cb->*method)(this);
    }
}

std::string
Layer::getAttribution() const
{
    // Get the attribution from the layer if it's set.
    return options().attribution().get();
}

void
Layer::setAttribution(const std::string& attribution)
{
    options().attribution() = attribution;
}

void
Layer::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Object::resizeGLObjectBuffers(maxSize);
    if (getNode())
        getNode()->resizeGLObjectBuffers(maxSize);
    if (getStateSet())
        getStateSet()->resizeGLObjectBuffers(maxSize);
}

void
Layer::releaseGLObjects(osg::State* state) const
{
    osg::Object::releaseGLObjects(state);
    if (getNode())
        getNode()->releaseGLObjects(state);
    if (getStateSet())
        getStateSet()->releaseGLObjects(state);
}

void
Layer::modifyTileBoundingBox(const TileKey& key, osg::BoundingBox& box) const
{
    //NOP
}

const Layer::Hints&
Layer::getHints() const
{
    return _hints;
}

Layer::Hints&
Layer::layerHints()
{
    return _hints;
}

const std::string&
Layer::getOsgOptionString() const
{
    return options().osgOptionString().get();
}

void
Layer::setUserProperty(
    const std::string& key,
    const std::string& value)
{
    options()._internal().set(key, value);
}
