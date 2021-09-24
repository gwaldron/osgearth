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
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/FeatureModelGraph>

using namespace osgEarth;

#define LC "[FeatureModelLayer] \"" << getName() << "\": "

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
    featureSource().get(conf, "features");
}

Config
FeatureModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();

    Config fmConf = FeatureModelOptions::getConfig();
    conf.merge(fmConf);

    Config gcConf = GeometryCompilerOptions::getConfig();
    conf.merge(gcConf);

    featureSource().set(conf, "features");

    return conf;
}

void FeatureModelLayer::Options::mergeConfig(const Config& conf)
{
    VisibleLayer::Options::mergeConfig(conf);
    fromConfig(conf);
}

//...........................................................................

OE_LAYER_PROPERTY_IMPL(FeatureModelLayer, bool, AlphaBlending, alphaBlending);
OE_LAYER_PROPERTY_IMPL(FeatureModelLayer, bool, EnableLighting, enableLighting);

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

void FeatureModelLayer::dirty()
{
    //// feature source changed, so the graph needs rebuilding
    //_graphDirty = true;

    //// create the scene graph
    //if (isOpen())
    //{
    //    create();
    //}
}

Config
FeatureModelLayer::getConfig() const
{
    Config conf = VisibleLayer::getConfig();
    return conf;
}

void
FeatureModelLayer::setFeatureSource(FeatureSource* source)
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
FeatureModelLayer::getFeatureSource() const
{
    return options().featureSource().getLayer();
}

void
FeatureModelLayer::setStyleSheet(StyleSheet* value)
{
    if (getStyleSheet() != value)
    {
        options().styleSheet().setLayer(value);
        dirty();
    }
}

StyleSheet*
FeatureModelLayer::getStyleSheet() const
{
    return options().styleSheet().getLayer();
}

void
FeatureModelLayer::setLayout(const FeatureDisplayLayout& value)
{
    options().layout() = value;
}

const FeatureDisplayLayout&
FeatureModelLayer::getLayout() const
{
    return options().layout().get();
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

Status
FeatureModelLayer::openImplementation()
{
    Status parent = VisibleLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    Status ssStatus =  options().styleSheet().open(getReadOptions());
    if (ssStatus.isError())
        return ssStatus;

    return Status::NoError;
}

Status
FeatureModelLayer::closeImplementation()
{
    options().featureSource().close();
    options().styleSheet().close();
    _graphDirty = true;
    if (_root.valid())
    {
        FeatureModelGraph* fmg = findTopMostNodeOfType<FeatureModelGraph>(_root.get());
        if (fmg) fmg->shutdown();
    }
    return getStatus();
}

const GeoExtent&
FeatureModelLayer::getExtent() const
{
    static GeoExtent s_invalid;

    FeatureSource* fs = getFeatureSource();
    return fs && fs->getFeatureProfile() ?
        fs->getFeatureProfile()->getExtent() :
        s_invalid;
}

void
FeatureModelLayer::addedToMap(const Map* map)
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
FeatureModelLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);

    options().featureSource().removedFromMap(map);
    options().styleSheet().removedFromMap(map);
    
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

    //if (_graphDirty)
    {
        if (isOpen() && getFeatureSource() && getStyleSheet() && _session.valid())
        {
            _session->setFeatureSource(getFeatureSource());

            // group that will build all the feature geometry:
            osg::ref_ptr<FeatureModelGraph> fmg = new FeatureModelGraph(options());
            fmg->setOwnerName(this->getName());
            fmg->setSession(_session.get());
            fmg->setNodeFactory(createFeatureNodeFactory());
            fmg->setSceneGraphCallbacks(getSceneGraphCallbacks());
            fmg->setStyleSheet(getStyleSheet());

            // pass though the min/max ranges
            if (options().maxVisibleRange().isSet())
                fmg->setMaxRange(options().maxVisibleRange().get());
            if (options().minVisibleRange().isSet())
                fmg->setMinRange(options().minVisibleRange().get());

            Status status = fmg->open();

            if (status.isError())
            {
                OE_WARN << LC << "ERROR intializing the FMG: " << status.toString() << std::endl;
                setStatus(status);
            }
            else
            {
                _root->removeChildren(0, _root->getNumChildren());
                _root->addChild(fmg);

                // clear the dirty flag.
                _graphDirty = false;

                setStatus(Status::OK());
            }
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
