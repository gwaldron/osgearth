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
#include <osgEarth/TiledFeatureModelLayer>
#include <osgEarth/TiledFeatureModelGraph>

using namespace osgEarth;

#define LC "[TiledFeatureModelLayer] "

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(TiledFeatureModel, TiledFeatureModelLayer);

//...........................................................................

TiledFeatureModelLayer::Options::Options() :
VisibleLayer::Options(),
FeatureModelOptions(),
GeometryCompilerOptions()
{
    fromConfig(_conf);
}

TiledFeatureModelLayer::Options::Options(const ConfigOptions& options) :
VisibleLayer::Options(options),
FeatureModelOptions(options),
GeometryCompilerOptions(options)
{
    fromConfig(_conf);
}

void TiledFeatureModelLayer::Options::fromConfig(const Config& conf)
{
    additive().setDefault(false);

    conf.get("additive", additive());
    featureSource().get(conf, "features");
}

Config
TiledFeatureModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();

    Config fmConf = FeatureModelOptions::getConfig();
    conf.merge(fmConf);

    Config gcConf = GeometryCompilerOptions::getConfig();
    conf.merge(gcConf);

    conf.set("additive", additive());

    featureSource().set(conf, "features");

    return conf;
}

void TiledFeatureModelLayer::Options::mergeConfig(const Config& conf)
{
    VisibleLayer::Options::mergeConfig(conf);
    fromConfig(conf);
}

//...........................................................................

OE_LAYER_PROPERTY_IMPL(TiledFeatureModelLayer, bool, AlphaBlending, alphaBlending);
OE_LAYER_PROPERTY_IMPL(TiledFeatureModelLayer, bool, EnableLighting, enableLighting);
OE_LAYER_PROPERTY_IMPL(TiledFeatureModelLayer, bool, Additive, additive);

TiledFeatureModelLayer::~TiledFeatureModelLayer()
{
    //NOP
}

void
TiledFeatureModelLayer::init()
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

void TiledFeatureModelLayer::dirty()
{
    // feature source changed, so the graph needs rebuilding
    _graphDirty = true;

    // create the scene graph
    create();
}

Config
TiledFeatureModelLayer::getConfig() const
{
    Config conf = VisibleLayer::getConfig();
    return conf;
}

void
TiledFeatureModelLayer::setFeatureSource(FeatureSource* source)
{
    if (getFeatureSource() != source)
    {
        options().featureSource().setLayer(source);

        if (source && source->getStatus().isError())
        {
            setStatus(source->getStatus());
            return;
        }

        dirty();
    }
}

FeatureSource*
TiledFeatureModelLayer::getFeatureSource() const
{
    return options().featureSource().getLayer();
}

void
TiledFeatureModelLayer::setStyleSheet(StyleSheet* value)
{
    if (getStyleSheet() != value)
    {
        options().styleSheet().setLayer(value);
        dirty();
    }
}

StyleSheet*
TiledFeatureModelLayer::getStyleSheet() const
{
    return options().styleSheet().getLayer();
}

osg::Node*
TiledFeatureModelLayer::getNode() const
{
    return _root.get();
}

Status
TiledFeatureModelLayer::openImplementation()
{
    Status parent = VisibleLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    Status ssStatus = options().styleSheet().open(getReadOptions());
    if (ssStatus.isError())
        return ssStatus;

    return Status::NoError;
}

Status
TiledFeatureModelLayer::closeImplementation()
{
    options().featureSource().close();
    options().styleSheet().close();
    _graphDirty = true;
    return getStatus();
}

const GeoExtent&
TiledFeatureModelLayer::getExtent() const
{
    static GeoExtent s_invalid;

    FeatureSource* fs = getFeatureSource();
    return fs && fs->getFeatureProfile() ?
        fs->getFeatureProfile()->getExtent() :
        s_invalid;
}

void
TiledFeatureModelLayer::addedToMap(const Map* map)
{
    OE_TEST << LC << "addedToMap" << std::endl;
    VisibleLayer::addedToMap(map);

    options().featureSource().addedToMap(map);
    options().styleSheet().addedToMap(map);

    if (getFeatureSource() && getStyleSheet())
    {
        // Save a reference to the map since we'll need it to
        // create a new session object later.
        _session = new Session(
            map,
            getStyleSheet(),
            getFeatureSource(),
            getReadOptions());

        // re-create the graph if necessary.
        create();
    }
}

void
TiledFeatureModelLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);

    options().featureSource().removedFromMap(map);
    options().styleSheet().removedFromMap(map);

    if (_root.valid())
    {
        osg::ref_ptr<TiledFeatureModelGraph> tfmg = findTopMostNodeOfType<TiledFeatureModelGraph>(_root.get());
        if (tfmg.valid()) tfmg->setDone();
        _root->removeChildren(0, _root->getNumChildren());
    }

    _session = 0L;
}

void
TiledFeatureModelLayer::create()
{
    OE_TEST << LC << "create" << std::endl;

    if (_graphDirty)
    {
        if (getFeatureSource() && getStyleSheet() && _session.valid())
        {
            // connect the session to the features:
            _session->setFeatureSource(getFeatureSource());
            _session->setResourceCache(new ResourceCache());

            // initialize filters if necessary
            osg::ref_ptr<FeatureFilterChain> chain = FeatureFilterChain::create(options().filters(), getReadOptions());

            // group that will build all the feature geometry:
            osg::ref_ptr<TiledFeatureModelGraph> fmg = new TiledFeatureModelGraph(_session->getMap(), getFeatureSource(), getStyleSheet(), _session.get());
            fmg->setOwnerName(getName());
            fmg->setFilterChain(chain.get());
            fmg->setAdditive(*_options->additive());
            fmg->build();

            _root->removeChildren(0, _root->getNumChildren());
            _root->addChild(fmg.get());

            // clear the dirty flag.
            _graphDirty = false;

            setStatus(Status::OK());

        }
    }
}