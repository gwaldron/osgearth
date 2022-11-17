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
#include <osgEarth/XYZModelLayer>
#include <osgEarth/XYZModelGraph>
#include <osgEarth/NodeUtils>

using namespace osgEarth;

#define LC "[XYZModelLayer] "

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(XYZModel, XYZModelLayer);

//...........................................................................

XYZModelLayer::Options::Options() :
VisibleLayer::Options()
{
    fromConfig(_conf);
}

XYZModelLayer::Options::Options(const ConfigOptions& options) :
VisibleLayer::Options(options)
{
    fromConfig(_conf);
}

void XYZModelLayer::Options::fromConfig(const Config& conf)
{
    invertY().setDefault(false);
    additive().setDefault(false);

    conf.get("additive", additive());
    conf.get("url", url());
    conf.get("min_level", minLevel());
    conf.get("max_level", maxLevel());
    conf.get("profile", profile());
}

Config
XYZModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    conf.set("additive", additive());
    conf.set("url", url());
    conf.set("min_level", minLevel());
    conf.set("max_level", maxLevel());
    conf.set("profile", profile());

    return conf;
}

void XYZModelLayer::Options::mergeConfig(const Config& conf)
{
    VisibleLayer::Options::mergeConfig(conf);
    fromConfig(conf);
}

//...........................................................................

OE_LAYER_PROPERTY_IMPL(XYZModelLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(XYZModelLayer, bool, Additive, additive);
OE_LAYER_PROPERTY_IMPL(XYZModelLayer, int, MinLevel, minLevel);
OE_LAYER_PROPERTY_IMPL(XYZModelLayer, int, MaxLevel, maxLevel);

XYZModelLayer::~XYZModelLayer()
{
    //NOP
}

void XYZModelLayer::setProfile(const Profile* profile)
{
    _profile = profile;
    if (_profile)
    {
        options().profile() = profile->toProfileOptions();
    }
}

void
XYZModelLayer::init()
{
    VisibleLayer::init();

    _root = new osg::Group();

    // Assign the layer's state set to the root node:
    _root->setStateSet(this->getOrCreateStateSet());

    // Graph needs rebuilding
    _graphDirty = true;

    // Depth sorting by default
    getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
}

void XYZModelLayer::dirty()
{
    _graphDirty = true;

    // create the scene graph
    create();
}

Config
XYZModelLayer::getConfig() const
{
    Config conf = VisibleLayer::getConfig();
    return conf;
}

osg::Node*
XYZModelLayer::getNode() const
{
    return _root.get();
}

Status
XYZModelLayer::openImplementation()
{
    Status parent = VisibleLayer::openImplementation();
    if (parent.isError())
        return parent;

    _profile = Profile::create(*options().profile());

    return Status::NoError;
}

Status
XYZModelLayer::closeImplementation()
{
    _graphDirty = true;
    return getStatus();
}

void
XYZModelLayer::addedToMap(const Map* map)
{
    OE_TEST << LC << "addedToMap" << std::endl;
    VisibleLayer::addedToMap(map);

    _map = map;

    _graphDirty = true;

    // re-create the graph if necessary.
    create();
}

void
XYZModelLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);

    if (_root.valid())
    {
        osg::ref_ptr<XYZModelGraph> node = findTopMostNodeOfType<XYZModelGraph>(_root.get());
        if (node.valid()) node->setDone();
        _root->removeChildren(0, _root->getNumChildren());
    }
}

void
XYZModelLayer::create()
{
    OE_TEST << LC << "create" << std::endl;

    if (_graphDirty && _profile)
    {             
        osg::ref_ptr<XYZModelGraph> xyzGraph = new XYZModelGraph(_map.get(), _profile.get(), *_options->url(), *_options->invertY(), getReadOptions());
        xyzGraph->setOwnerName(getName());
        xyzGraph->setAdditive(*_options->additive());
        xyzGraph->setMinLevel(*_options->minLevel());
        xyzGraph->setMaxLevel(*_options->maxLevel());
        xyzGraph->build();

        _root->removeChildren(0, _root->getNumChildren());
        _root->addChild(xyzGraph.get());

        // clear the dirty flag.
        _graphDirty = false;

        setStatus(Status::OK());
    }
}