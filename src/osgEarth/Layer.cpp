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
#include <osgDB/Registry>

using namespace osgEarth;

#define LC "[Layer] "

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
    //isolate ? LayerOp::newConfig() : ConfigOptions::getConfig();
    Config conf = ConfigOptions::newConfig();
    conf.addIfSet("name", _name);
    conf.addIfSet("enabled", _enabled);
    return conf;
}

void LayerOptions::fromConfig(const Config& conf)
{
    _enabled.init(true);

    conf.getIfSet("name", _name);
    conf.getIfSet("enabled", _enabled);
}

void LayerOptions::mergeConfig(const Config& conf)
{
    ConfigOptions::mergeConfig(conf);
    fromConfig(_conf);
}

//.................................................................

Layer::Layer() :
_layerOptions(&_layerOptionsConcrete)
{
    _uid = osgEarth::Registry::instance()->createUID();
    _renderType = RENDERTYPE_NONE;
    _status = Status::OK();

    init();
}

Layer::Layer(LayerOptions* optionsPtr) :
_layerOptions(optionsPtr? optionsPtr : &_layerOptionsConcrete)
{
    _uid = osgEarth::Registry::instance()->createUID();
    _renderType = RENDERTYPE_NONE;
    _status = Status::OK();
}

Layer::~Layer()
{
    OE_DEBUG << LC << "~Layer\n";
}

Config
Layer::getConfig() const
{
    return getLayerOptions().getConfig();
}

void
Layer::init()
{
    // Copy the layer options name into the Object name.
    if (getLayerOptions().name().isSet())
    {
        osg::Object::setName(getLayerOptions().name().get());
    }
}

void
Layer::setName(const std::string& name)
{
    osg::Object::setName(name);
    mutableLayerOptions().name() = name;
}


#define LAYER_OPTIONS_TAG "osgEarth.LayerOptions"

Layer*
Layer::create(const std::string& name, const ConfigOptions& options)
{
    if ( name.empty() )
    {
        OE_WARN << LC << "ILLEGAL- Layer::create requires a plugin name" << std::endl;
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
