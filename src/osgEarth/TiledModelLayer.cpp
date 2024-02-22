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
#include "TiledModelLayer"

using namespace osgEarth;

void TiledModelLayer::Options::fromConfig(const Config& conf)
{
    additive().setDefault(false);
    rangeFactor().setDefault(6.0);
    conf.get("additive", additive());
    conf.get("range_factor", rangeFactor());
}

Config
TiledModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    conf.set("additive", additive());
    conf.set("range_factor", rangeFactor());
    return conf;
}

OE_LAYER_PROPERTY_IMPL(TiledModelLayer, bool, Additive, additive);
OE_LAYER_PROPERTY_IMPL(TiledModelLayer, float, RangeFactor, rangeFactor);

osg::ref_ptr<osg::Node>
TiledModelLayer::createTile(const TileKey& key, ProgressCallback* progress) const
{
    if (key.getProfile()->isEquivalentTo(getProfile()))
    {
        return createTileImplementation(key, progress);
    }
    else
    {
        osg::ref_ptr<osg::Group> group = new osg::Group();

        std::vector<TileKey> i_keys;
        getProfile()->getIntersectingTiles(key, i_keys);

        if (i_keys.empty())
            return {};

        std::set<TileKey> unique_keys;

        for (auto unique_key : i_keys)
        {
            if (unique_key.getLOD() > getMaxLevel())
                unique_key = unique_key.createAncestorKey(getMaxLevel());
            if (unique_key.valid())
                unique_keys.insert(unique_key);
        }

        for (auto& unique_key : unique_keys)
        {
            osg::ref_ptr<osg::Node> part = createTileImplementation(unique_key, progress);
            if (part.valid())
            {
                group->addChild(part);
            }
        }

        return group;
    }
}
