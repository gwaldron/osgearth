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
#include <osgEarthFeatures/FeatureModelLayer>
#include <osgEarthFeatures/FeatureModelGraph>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[FeatureModelLayer] "

REGISTER_OSGEARTH_LAYER(feature_model, FeatureModelLayer);

//...........................................................................

FeatureModelLayerOptions::FeatureModelLayerOptions(const ConfigOptions& options) :
VisibleLayerOptions(options),
FeatureModelOptions(options),
GeometryCompilerOptions(options)
{
    fromConfig(_conf);
}
        
void FeatureModelLayerOptions::fromConfig(const Config& conf)
{
    conf.getIfSet("feature_source", _featureSourceLayer);
}

Config
FeatureModelLayerOptions::getConfig() const
{
    Config conf = VisibleLayerOptions::getConfig();
    conf.merge(FeatureModelOptions::getConfig());
    conf.merge(GeometryCompilerOptions::getConfig());
    conf.key() = "feature_model";

    conf.set("feature_source", _featureSourceLayer);
    return conf;
}

void FeatureModelLayerOptions::mergeConfig(const Config& conf)
{
    VisibleLayerOptions::mergeConfig(conf);
    fromConfig(conf);
}

//...........................................................................

FeatureModelLayer::FeatureModelLayer() :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

FeatureModelLayer::FeatureModelLayer(const FeatureModelLayerOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

FeatureModelLayer::~FeatureModelLayer()
{
    //NOP
}

void
FeatureModelLayer::init()
{
    VisibleLayer::init();

    _root = new osg::Group();

    // Callbacks for paged data
    _sgCallbacks = new SceneGraphCallbacks();
}

void
FeatureModelLayer::setFeatureSourceLayer(FeatureSourceLayer* layer)
{
    if (layer && layer->getStatus().isError())
    {
        setStatus(Status::Error(Status::ResourceUnavailable, "Feature source layer is unavailable; check for error"));
        return;
    }

    if (layer)
        OE_INFO << LC << "Feature source layer is \"" << layer->getName() << "\"\n";

    setFeatureSource(layer ? layer->getFeatureSource() : 0L);
}

void
FeatureModelLayer::setFeatureSource(FeatureSource* source)
{
    if (_featureSource != source)
    {
        if (source)
            OE_INFO << LC << "Setting feature source \"" << source->getName() << "\"\n";

        _featureSource = source;

        if (source && source->getStatus().isError())
        {
            setStatus(source->getStatus());
            return;
        }

        create();
    }
}

osg::Node*
FeatureModelLayer::getOrCreateNode()
{
    OE_DEBUG << LC << "getNode\n";
    return _root.get();
}

const Status&
FeatureModelLayer::open()
{
    OE_DEBUG << LC << "open\n";

    if (options().featureSource().isSet())
    {
        FeatureSource* fs = FeatureSourceFactory::create(options().featureSource().get());
        if (fs)
        {
            fs->setReadOptions(getReadOptions());
            fs->open();
            setFeatureSource(fs);
        }
        else
        {
            setStatus(Status(Status::ConfigurationError, "Cannot create feature source"));
        }
    }
    return VisibleLayer::open();
}

void
FeatureModelLayer::addedToMap(const Map* map)
{
    OE_DEBUG << LC << "addedToMap\n";

    // Save a reference to the map since we'll need it to
    // create a new session object later.
    _session = new Session(map);
    _session->setStyles( options().styles().get() );

    if (options().featureSourceLayer().isSet())
    {
        _featureSourceLayerListener.listen(
            map,
            options().featureSourceLayer().get(),
            this,
            &FeatureModelLayer::setFeatureSourceLayer);
    }

    create();
}

void
FeatureModelLayer::removedFromMap(const Map* map)
{
    _featureSourceLayerListener.clear();
}

void
FeatureModelLayer::create()
{
    if (_featureSource.valid() && _session.valid())
    {
        // connect the session to the features:
        _session->setFeatureSource(_featureSource.get());

        // the factory builds nodes for the model graph:
        FeatureNodeFactory* nodeFactory = new GeomFeatureNodeFactory(options());

        // group that will build all the feature geometry:
        FeatureModelGraph* fmg = new FeatureModelGraph(
            _session.get(),
            options(),
            nodeFactory,
            _sgCallbacks.get());

        _root->removeChildren(0, _root->getNumChildren());
        _root->addChild(fmg);

        setStatus(Status::OK());
    }

    else if (getStatus().isOK())
    {
        setStatus(Status(Status::ConfigurationError));
    }
}
