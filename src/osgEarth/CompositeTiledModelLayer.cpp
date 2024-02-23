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
#include <osgEarth/CompositeTiledModelLayer>
#include <osgEarth/NodeUtils>
#include <osgEarth/SimplePager>

#include <vector>

using namespace osgEarth;

#define LC "[CompositeTiledModelLayer] "

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(CompositeTiledModel, CompositeTiledModelLayer);

void CompositeTiledModelLayer::Options::fromConfig(const Config& conf)
{
    conf.get("profile", profile());

    const ConfigSet& layers = conf.child("layers").children();
    for (ConfigSet::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        _layers.push_back(ConfigOptions(*i));
    }
}

Config
CompositeTiledModelLayer::Options::getConfig() const
{
    Config conf = TiledModelLayer::Options::getConfig();
    conf.set("profile", profile());

    if (_layers.empty() == false)
    {
        Config layersConf("layers");
        for (std::vector<ConfigOptions>::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            layersConf.add(i->getConfig());
        }
        conf.set(layersConf);
    }

    return conf;
}

//...........................................................................

CompositeTiledModelLayer::~CompositeTiledModelLayer()
{
    //NOP
}

void CompositeTiledModelLayer::setProfile(const Profile* profile)
{
    _profile = profile;
    if (_profile)
    {
        options().profile() = profile->toProfileOptions();
    }
}

Config
CompositeTiledModelLayer::getConfig() const
{
    Config conf = TiledModelLayer::getConfig();
    return conf;
}

Status
CompositeTiledModelLayer::openImplementation()
{
    Status parent = super::openImplementation();
    if (parent.isError())
        return parent;

    _profile = Profile::create(*options().profile());

    // Open all the layers
    for (auto& layerConf : options().layers())
    {
        osg::ref_ptr< Layer > layer = Layer::create(layerConf);
        TiledModelLayer* tiledLayer = dynamic_cast<TiledModelLayer*>(layer.get());
        if (tiledLayer)
        {
            tiledLayer->open();
            tiledLayer->setReadOptions(getReadOptions());
            _layers.push_back(tiledLayer);
        }
        else
        {
            OE_WARN << LC << "Layer is not a TiledModelLayer" << std::endl;
        }
    }

    return Status::NoError;
}

void
CompositeTiledModelLayer::addedToMap(const Map* map)
{
    for (auto& layer : _layers)
    {
        layer->addedToMap(map);
    }

    // Check to make sure the layers are all using the same profile
    for (auto& layer : _layers)
    {
        if (!layer->getProfile()->isEquivalentTo(_profile))
        {
            OE_WARN << LC << "Layer " << layer->getName() << " does not have the same profile as the composite layer" << std::endl;
        }
    }

    // Compute the min and max levels
    if (!_layers.empty())
    {
        _minLevel = 1000;
        _maxLevel = 0;
        for (auto& layer : _layers)
        {
            if (layer->getMinLevel() < _minLevel)
            {
                _minLevel = layer->getMinLevel();
            }
            if (layer->getMaxLevel() > _maxLevel)
            {
                _maxLevel = layer->getMaxLevel();
            }
        }
    }

    super::addedToMap(map);
}

void
CompositeTiledModelLayer::removedFromMap(const Map* map)
{
    super::removedFromMap(map);

    for (auto& layer : _layers)
    {
        layer->removedFromMap(map);
    }
}

osg::ref_ptr<osg::Node>
CompositeTiledModelLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr<osg::Group> group = new osg::Group();

    for (unsigned int i = 0; i < this->_layers.size(); i++)
    {
        osg::ref_ptr<osg::Node> node = this->_layers[i]->createTile(key, progress);
        if (node.valid())
        {
            group->addChild(node);
        }
    }
    return group;
}

unsigned
CompositeTiledModelLayer::getMinLevel() const
{
    return _minLevel;
}

unsigned
CompositeTiledModelLayer::getMaxLevel() const
{
    return _maxLevel;
}

