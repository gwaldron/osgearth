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
    _additive.init(false);
    conf.get("additive", _additive);

    LayerReference<FeatureSource>::get(conf, "features", featureSourceLayer(), featureSource());
}

Config
TiledFeatureModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();

    Config fmConf = FeatureModelOptions::getConfig();
    conf.merge(fmConf);

    Config gcConf = GeometryCompilerOptions::getConfig();
    conf.merge(gcConf);

    LayerReference<FeatureSource>::set(conf, "features", featureSourceLayer(), featureSource());

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

    if (_featureSource.isSetByUser())
        conf.set(_featureSource.getLayer()->getConfig());

    if (_styleSheet.isSetByUser())
        conf.set(_styleSheet.getLayer()->getConfig());

    return conf;
}

void
TiledFeatureModelLayer::setFeatureSource(FeatureSource* source)
{
    if (getFeatureSource() != source)
    {
        _featureSource.setLayer(source);

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
    return _featureSource.getLayer();
}

void
TiledFeatureModelLayer::setStyleSheet(StyleSheet* value)
{
    if (getStyleSheet() != value)
    {
        _styleSheet.setLayer(value);
        dirty();
    }
}

StyleSheet*
TiledFeatureModelLayer::getStyleSheet() const
{
    return _styleSheet.getLayer();
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

    Status fsStatus = _featureSource.open(options().featureSource(), getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    Status ssStatus = _styleSheet.open(options().styleSheet(), getReadOptions());
    if (ssStatus.isError())
        return ssStatus;

    return Status::NoError;
}

Status
TiledFeatureModelLayer::closeImplementation()
{
    _featureSource.close();
    _styleSheet.close();
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

    _featureSource.findInMap(map, options().featureSourceLayer());
    _styleSheet.findInMap(map, options().styleSheetLayer());

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

    _featureSource.releaseFromMap(map);
    _styleSheet.releaseFromMap(map);
    
    if (_root.valid())
    {
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

            // group that will build all the feature geometry:
            osg::ref_ptr<TiledFeatureModelGraph> fmg = new TiledFeatureModelGraph(getFeatureSource(), getStyleSheet(), _session.get());
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