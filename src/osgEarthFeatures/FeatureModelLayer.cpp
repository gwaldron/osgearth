/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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

REGISTER_OSGEARTH_LAYER(FeatureModel, FeatureModelLayer);

REGISTER_OSGEARTH_LAYER(feature_model, FeatureModelLayer); // backwards compatibility

//...........................................................................

FeatureModelLayer::Options::Options() :
VisibleLayer::Options(),
FeatureModelOptions(),
GeometryCompilerOptions()
{
    fromConfig(_conf);
}

FeatureModelLayer::Options::Options(const ConfigOptions& options) :
VisibleLayer::Options(options),
FeatureModelOptions(options),
GeometryCompilerOptions(options)
{
    fromConfig(_conf);
}

void FeatureModelLayer::Options::fromConfig(const Config& conf)
{
    conf.get("features", _featureSourceLayer);
    conf.get("feature_source", _featureSourceLayer);

    // Check for an embedded feature source
    for(ConfigSet::const_iterator i = conf.children().begin();
        i != conf.children().end();
        ++i)
    {
        osg::ref_ptr<FeatureSource> fs = FeatureSource::create(*i);
        if (fs.valid())
        {
            _featureSource = FeatureSource::Options(*i);
            break;
        }
    }
}

Config
FeatureModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    conf.merge(FeatureModelOptions::getConfig());
    conf.merge(GeometryCompilerOptions::getConfig());

    conf.set("features", _featureSourceLayer);

    if (_featureSource.isSet())
        conf.set(_featureSource->getConfig());

    return conf;
}

void FeatureModelLayer::Options::mergeConfig(const Config& conf)
{
    VisibleLayer::Options::mergeConfig(conf);
    fromConfig(conf);
}

//...........................................................................

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

//void FeatureModelLayer::setFeatureModelLayerOptions(const FeatureModelLayerOptions& options)
//{
//    *_options = options;
//    dirty();
//}

void FeatureModelLayer::dirty()
{
    // feature source changed, so the graph needs rebuilding
    _graphDirty = true;

    // create the scene graph
    create();
}

void
FeatureModelLayer::setFeatureSource(FeatureSource* source)
{
    if (source && source->getStatus().isError())
    {
        setStatus(source->getStatus());
        return;
    }

    if (source)
    {
        OE_INFO << LC << "Feature source layer is \"" << source->getName() << "\"\n";
    }

    if (_features.get() != source)
    {
        if (source)
            OE_INFO << LC << "Setting feature source \"" << source->getName() << "\"\n";

        _features = source;

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
FeatureModelLayer::setStyleSheet(StyleSheet* value)
{
    options().styles() = value;
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
    if (options().featureSource().isSet())
    {
        FeatureSource* fs = FeatureSource::create(options().featureSource().get());
        if (fs)
        {
            fs->setReadOptions(getReadOptions());
            const Status& fsStatus = fs->open();
            if (fsStatus.isError())
            {
                return setStatus(fsStatus);
            }
            setFeatureSource(fs);
        }
    }

    return VisibleLayer::open();
}

const GeoExtent&
FeatureModelLayer::getExtent() const
{
    static GeoExtent s_invalid;

    return _features.valid() && _features->getFeatureProfile() ?
        _features->getFeatureProfile()->getExtent() :
        s_invalid;
}

void
FeatureModelLayer::addedToMap(const Map* map)
{
    OE_TEST << LC << "addedToMap" << std::endl;
    VisibleLayer::addedToMap(map);

    // Save a reference to the map since we'll need it to
    // create a new session object later.
    _session = new Session(
        map, 
        options().styles().get(), 
        0L,  // feature source - will set later
        getReadOptions());

    // If we have a layer name but no feature source, fire up a
    // listener so we'll be notified when the named layer is 
    // added to the map.
    if (!_features.valid() && options().featureSourceLayer().isSet())
    {
        _featureLayerListener.listen(
            map,
            options().featureSourceLayer().get(),
            this,
            &FeatureModelLayer::setFeatureSource);
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
    VisibleLayer::removedFromMap(map);

    _featureLayerListener.clear();
    
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
        if (_features.valid() && _session.valid())
        {
            // connect the session to the features:
            _session->setFeatureSource(_features.get());

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
