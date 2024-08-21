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
#include "SimplePager"
#include "NodeUtils"
#include "Chonk"
#include "Registry"
#include "ShaderGenerator"
#include "Style"
#include <osg/BlendFunc>

using namespace osgEarth;

void TiledModelLayer::Options::fromConfig(const Config& conf)
{
    additive().setDefault(false);
    rangeFactor().setDefault(6.0);
    conf.get("additive", additive());
    conf.get("range_factor", rangeFactor());
    conf.get("min_level", minLevel());
    conf.get("max_level", maxLevel());
    conf.get("nvgl", nvgl());
}

Config
TiledModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    conf.set("additive", additive());
    conf.set("range_factor", rangeFactor());
    conf.set("min_level", minLevel());
    conf.set("max_level", maxLevel());
    conf.set("nvgl", nvgl());
    return conf;
}

OE_LAYER_PROPERTY_IMPL(TiledModelLayer, bool, Additive, additive);
OE_LAYER_PROPERTY_IMPL(TiledModelLayer, float, RangeFactor, rangeFactor);

void TiledModelLayer::setMinLevel(unsigned value)
{
    options().minLevel() = value;
}

unsigned TiledModelLayer::getMinLevel() const
{
    return options().minLevel().get();
}

void TiledModelLayer::setMaxLevel(unsigned value)
{
    options().maxLevel() = value;
}

unsigned TiledModelLayer::getMaxLevel() const
{
    return options().maxLevel().get();
}

osg::ref_ptr<osg::Node>
TiledModelLayer::createTile(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr<osg::Node> result;

    if (key.getProfile()->isHorizEquivalentTo(getProfile()))
    {
        result = createTileImplementation(key, progress);
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

        result = group;
    }

    if (result.valid())
    {
        if (_textures.valid())
        {
            forEachNodeOfType<StyleGroup>(result, [&](StyleGroup* group)
                {
                    auto drawable = new ChonkDrawable();
                    drawable->add(group, _chonkFactory);
                    group->removeChildren(0, group->getNumChildren());
                    group->addChild(drawable);

                    auto* render = group->_style.get<RenderSymbol>();
                    if (render)
                        render->applyTo(group);
                });
        }
        else
        {
            osgEarth::Registry::shaderGenerator().run(result.get(), _statesetCache);
        }
    }

    return result;
}

// The Node representing this layer.
osg::Node* TiledModelLayer::getNode() const
{
    return _root.get();
}

// called by the map when this layer is added
void
TiledModelLayer::addedToMap(const Map* map)
{
    super::addedToMap(map);

    if (*options().nvgl() == true && GLUtils::useNVGL() && !_textures.valid())
    {
        _textures = new TextureArena();
        getOrCreateStateSet()->setAttribute(_textures, 1);

        // auto release requires that we install this update callback!
        _textures->setAutoRelease(true);

        _chonkFactory.textures = _textures;

        getNode()->addUpdateCallback(new LambdaCallback<>([this](osg::NodeVisitor& nv)
            {
                _textures->update(nv);
                return true;
            }));

        getOrCreateStateSet()->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }

    _map = map;

    _graphDirty = true;

    // re-create the graph if necessary.
    create();
}

// called by the map when this layer is removed
void
TiledModelLayer::removedFromMap(const Map* map)
{
    super::removedFromMap(map);

    if (_root.valid())
    {
        osg::ref_ptr<SimplePager> node = findTopMostNodeOfType<SimplePager>(_root.get());
        if (node.valid())
            node->setDone();

        _root->removeChildren(0, _root->getNumChildren());
    }
}

void TiledModelLayer::dirty()
{
    _graphDirty = true;

    // create the scene graph
    create();
}

// post-ctor initialization
void TiledModelLayer::init()
{
    super::init();

    // Create the container group

    _root = new osg::Group();

    // Assign the layer's state set to the root node:
    _root->setStateSet(this->getOrCreateStateSet());

    // Graph needs rebuilding
    _graphDirty = true;

    // Depth sorting by default
    getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
}

void TiledModelLayer::create()
{
    if (_map.valid() && _graphDirty)
    {
        _root->removeChildren(0, _root->getNumChildren());

        auto pager = new SimplePager(_map.get(), getProfile());

        pager->setCreateNodeFunction([layer_weak{ osg::observer_ptr<TiledModelLayer>(this) }](const TileKey& key, ProgressCallback* progress)
            {
                osg::ref_ptr<TiledModelLayer> layer;
                return layer_weak.lock(layer) ? layer->createTile(key, progress) : osg::ref_ptr<osg::Node>();
            });

        pager->setAdditive(this->getAdditive());
        pager->setRangeFactor(this->getRangeFactor());
        pager->setMinLevel(this->getMinLevel());
        pager->setMaxLevel(this->getMaxLevel());
        pager->build();

        // TODO:  NVGL
        _root->addChild(pager);
        _graphDirty = false;
    }
}   