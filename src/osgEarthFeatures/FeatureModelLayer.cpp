/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

#define OE_TEST OE_NULL

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
    conf.get("feature_source", _featureSourceLayer);
}

Config
FeatureModelLayerOptions::getConfig() const
{
    Config conf = VisibleLayerOptions::getConfig();
    conf.merge(FeatureModelOptions::getConfig());
    conf.merge(GeometryCompilerOptions::getConfig());

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

    // Assign the layer's state set to the root node:
    _root->setStateSet(this->getOrCreateStateSet());

    // Graph needs rebuilding
    _graphDirty = true;

    // Depth sorting by default
    getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // activate opacity support
    installDefaultOpacityShader();
}

void FeatureModelLayer::setFeatureModelLayerOptions(const FeatureModelLayerOptions& options)
{
    *_options = options;
    dirty();
}

void FeatureModelLayer::dirty()
{
    // feature source changed, so the graph needs rebuilding
    _graphDirty = true;

    // create the scene graph
    create();
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
    OE_TEST << LC << "setFeatureSource" << std::endl;

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

        dirty();
    }
}

FeatureSource*
FeatureModelLayer::getFeatureSource() const
{
    return _featureSource.get();
}

void
FeatureModelLayer::setCreateFeatureNodeFactoryCallback(CreateFeatureNodeFactoryCallback* value)
{
    _createFactoryCallback = value;
}

FeatureModelLayer::CreateFeatureNodeFactoryCallback*
FeatureModelLayer::getCreateFeatureNodeFactoryCallback() const
{
    return _createFactoryCallback.get();
}

osg::Node*
FeatureModelLayer::getNode() const
{
    return _root.get();
}

const Status&
FeatureModelLayer::open()
{
    OE_TEST << LC << "open" << std::endl;

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

const GeoExtent&
FeatureModelLayer::getExtent() const
{
    static GeoExtent s_invalid;

    return _featureSource.valid() && _featureSource->getFeatureProfile() ?
        _featureSource->getFeatureProfile()->getExtent() :
        s_invalid;
}

void
FeatureModelLayer::addedToMap(const Map* map)
{
    OE_TEST << LC << "addedToMap" << std::endl;

    // Save a reference to the map since we'll need it to
    // create a new session object later.
    _session = new Session(
        map, 
        options().styles().get(), 
        0L,  // feature source - will set later
        getReadOptions());

    if (options().featureSourceLayer().isSet())
    {
        _featureSourceLayerListener.listen(
            map,
            options().featureSourceLayer().get(),
            this,
            &FeatureModelLayer::setFeatureSourceLayer);
    }
    else
    {
        // re-create the graph if necessary.
        create();
    }
}

void
FeatureModelLayer::removedFromMap(const Map* map)
{
    _featureSourceLayerListener.clear();
    
    if (_root.valid())
    {
        _root->removeChildren(0, _root->getNumChildren());
    }

    _session = 0L;
}

void
FeatureModelLayer::create()
{
    OE_TEST << LC << "create" << std::endl;

    if (_graphDirty)
    {
        if (_featureSource.valid() && _session.valid())
        {
            // connect the session to the features:
            _session->setFeatureSource(_featureSource.get());

            // the factory builds nodes for the model graph:
            FeatureNodeFactory* nodeFactory = createFeatureNodeFactory();

            // group that will build all the feature geometry:
            FeatureModelGraph* fmg = new FeatureModelGraph(
                _session.get(),
                options(),
                nodeFactory,
                getSceneGraphCallbacks());

            _root->removeChildren(0, _root->getNumChildren());
            _root->addChild(fmg);

            // clear the dirty flag.
            _graphDirty = false;

            setStatus(Status::OK());
        }

        //else if (getStatus().isOK())
        //{
        //    if (!_featureSource.valid())
        //        setStatus(Status(Status::ConfigurationError, "No feature source"));
        //    else if (!_session.valid())
        //        setStatus(Status(Status::ConfigurationError, "No Session"));
        //}
    }
}

FeatureNodeFactory*
FeatureModelLayer::createFeatureNodeFactoryImplementation() const
{
    return new GeomFeatureNodeFactory(options());
}

FeatureNodeFactory*
FeatureModelLayer::createFeatureNodeFactory()
{
    if (_createFactoryCallback.valid())
        return _createFactoryCallback->createFeatureNodeFactory(options());
    else
        return createFeatureNodeFactoryImplementation();
}
