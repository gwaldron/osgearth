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
#include <osgEarthProcedural/ProceduralTiledModelLayer>
#include <osgEarth/Registry>
#include <osgEarth/Progress>

using namespace osgEarth;
using namespace osgEarth::Procedural;


#define LC "[ProceduralTiledModelLayer] "

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(ProceduralTiledModel, ProceduralTiledModelLayer);


void ProceduralTiledModelLayer::Options::fromConfig(const Config& conf)
{
    conf.get("min_level", minLevel());
    conf.get("max_level", maxLevel());
    conf.get("profile", profile());
    conf.get("url", url());
}

Config
ProceduralTiledModelLayer::Options::getConfig() const
{
    Config conf = TiledModelLayer::Options::getConfig();
    conf.set("min_level", minLevel());
    conf.set("max_level", maxLevel());
    conf.set("profile", profile());
    conf.set("url", url());
    return conf;
}


ProceduralTiledModelLayer::~ProceduralTiledModelLayer()
{
    //NOP
}

void
ProceduralTiledModelLayer::init()
{
    TiledModelLayer::init();

    // Initialize a NodeGraph with a default NodeOutputOperation
    _nodeGraph = std::make_shared<NodeGraph>();

    // Make a final node output
    auto output = std::make_shared< NodeOutputOperation >();
    _nodeGraph->operations.push_back(output);

    if (getProfile() == nullptr)
    {
        setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
    }

    // some reasonable defaults
    options().maxLevel().setDefault(14u);
    options().additive().setDefault(false);
}

Config
ProceduralTiledModelLayer::getConfig() const
{
    Config conf = super::getConfig();
    return conf;
}


Status
ProceduralTiledModelLayer::openImplementation()
{
    Status parent = super::openImplementation();
    if (parent.isError())
        return parent;

    _profile = Profile::create(*options().profile());

    if (options().url().isSet())
    {
        //TODO
    }

    return Status::NoError;
}

Status
ProceduralTiledModelLayer::closeImplementation()
{
    super::closeImplementation();    
    return getStatus();
}

const GeoExtent&
ProceduralTiledModelLayer::getExtent() const
{
    return getProfile()->getExtent();    
}

void
ProceduralTiledModelLayer::addedToMap(const Map* map)
{
    OE_TEST << LC << "addedToMap" << std::endl;    

    TiledModelLayer::addedToMap(map);    
}

void
ProceduralTiledModelLayer::removedFromMap(const Map* map)
{
    super::removedFromMap(map);    
}

osg::ref_ptr<osg::Node>
ProceduralTiledModelLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (progress && progress->isCanceled())
        return nullptr;

    auto map = getMap();

    if (_nodeGraph && map.valid())
    {
        NodeGraphNode* mt = new NodeGraphNode;
        mt->_tileKey = key;
        mt->_map = map.get();
        osg::Matrixd l2w;
        auto centroid = key.getExtent().getCentroid();
        auto centroidMap = centroid.transform(map->getSRS());
        centroidMap.createLocalToWorld(l2w);
//        mt->setMatrix(l2w);
        mt->_nodeGraph = _nodeGraph;
        mt->build();
        const_cast<ProceduralTiledModelLayer*>(this)->registerNode(mt);
        return mt;
    }

    return nullptr;
}

void
ProceduralTiledModelLayer::setMinLevel(unsigned value) {
    options().minLevel() = value;
}

unsigned
ProceduralTiledModelLayer::getMinLevel() const {
    return options().minLevel().get();
}

void
ProceduralTiledModelLayer::setMaxLevel(unsigned value) {
    options().maxLevel() = value;
}

unsigned
ProceduralTiledModelLayer::getMaxLevel() const {
    return options().maxLevel().get();
}

void ProceduralTiledModelLayer::setProfile(const Profile* profile)
{
    _profile = profile;
    if (_profile)
    {
        options().profile() = profile->toProfileOptions();
    }
}
