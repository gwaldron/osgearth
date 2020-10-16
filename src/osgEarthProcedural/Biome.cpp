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
#include "Biome"

using namespace osgEarth;
using namespace osgEarth::Procedural;

#undef LC
#define LC "[Biome] "


ModelAsset::ModelAsset(const Config& conf)
{
    conf.get("url", modelURI());
    conf.get("name", name());
    conf.get("side_url", sideBillboardURI());
    conf.get("top_url", topBillboardURI());
    conf.get("width", width());
    conf.get("height", height());
    conf.get("size_variation", sizeVariation());
    conf.get("selection_weight", selectionWeight());
    //conf.get("fill", fill());
}

Config
ModelAsset::getConfig() const
{
    Config conf("model");
    conf.set("name", name());
    conf.set("url", modelURI());
    conf.set("side_url", sideBillboardURI());
    conf.set("top_url", topBillboardURI());
    conf.set("width", width());
    conf.set("height", height());
    conf.set("size_variation", sizeVariation());
    conf.set("selection_weight", selectionWeight());
    //conf.set("fill", fill());
    return conf;
}

//...................................................................

GroundTextureAsset::GroundTextureAsset(const Config& conf)
{
    conf.get("name", name());
    conf.get("url", uri());
    conf.get("width", width());
    conf.get("height", height());
}

Config
GroundTextureAsset::getConfig() const
{
    Config conf("texture");
    conf.set("name", name());
    conf.set("url", uri());
    conf.set("width", width());
    conf.set("height", height());
    return conf;
}

//...................................................................

AssetCatalog::AssetCatalog(const Config& conf)
{
    ConfigSet texturesConf = conf.child("groundtextures").children("texture");
    for (const auto& textureConf : texturesConf)
    {
        auto obj = new GroundTextureAsset(textureConf);
        _textures.push_back(obj);
        //_textures[obj->name().get()] = obj;
    }

    ConfigSet modelsConf = conf.child("models").children("model");
    for (const auto& modelConf : modelsConf)
    {
        auto obj = new ModelAsset(modelConf);
        _models[obj->name().get()] = obj;
    }
}

Config
AssetCatalog::getConfig() const
{
    Config conf("AssetCatalog");

    Config textures("GroundTextures");
    for (auto& i : _textures)
        textures.add(i->getConfig()); // i.second->getConfig());
    if (!textures.empty())
        conf.add(textures);

    Config models("Models");
    for (auto& i : _models)
        models.add(i.second->getConfig());
    if (!models.empty())
        conf.add(models);

    return conf;
}

#if 0
GroundTextureAsset*
AssetCatalog::getTexture(const std::string& name) const
{
    auto i = _textures.find(name);
    return i != _textures.end() ? i->second.get() : nullptr;
}
#endif

const std::vector<osg::ref_ptr<GroundTextureAsset>>&
AssetCatalog::getTextures() const
{
    return _textures;
}

ModelAsset*
AssetCatalog::getModel(const std::string& name) const
{
    auto i = _models.find(name);
    return i != _models.end() ? i->second.get() : nullptr;
}

//..........................................................

ModelCategory::ModelCategory(const Config& conf, AssetCatalog* assets)
{
    conf.get("name", name());
    ConfigSet mc = conf.children("model");
    for (const auto& c : mc)
    {
        ModelAsset* asset = assets->getModel(c.value("name"));
        if (asset) {
            _assets.emplace_back(Usage());
            _assets.back().asset = asset;
            _assets.back().weight = (float)c.value("weight", 1.0f);
        }
        else {
            OE_WARN << LC << "ModelCategory \"" << name().get() << "\""
                << " references unknown asset \"" << c.value("name")  << "\""
                << std::endl;
        }
    }
}

Config
ModelCategory::getConfig() const
{
    Config conf;
    //TODO
    OE_WARN << __func__ << " not implemented" << std::endl;
    return conf;
}

//...................................................................

LandUseType::LandUseType(const Config& conf)
{
    dense().setDefault(0.0f);
    lush().setDefault(0.0f);
    rugged().setDefault(0.5f);

    conf.get("id", id());
    conf.get("dense", dense());
    conf.get("lush", lush());
    conf.get("rugged", rugged());
}

Config
LandUseType::getConfig() const
{
    Config conf;
    //TODO
    OE_WARN << __func__ << " not implemented" << std::endl;
    return conf;
}

//...................................................................

LandUseCatalog::LandUseCatalog(const Config& conf)
{
    const ConfigSet& children = conf.children();
    for (const auto& child : children)
    {
        if (!child.empty())
        {
            LandUseType type(child);

            if (type.id().isSet())
            {
                landUseTypes().push_back(type);
                const LandUseType* ptr = &landUseTypes().back();
                _lut[type.id().get()] = type;
            }
        }
    }
}

Config 
LandUseCatalog::getConfig() const
{
    Config conf;
    //TODO
    OE_WARN << __func__ << " not implemented" << std::endl;
    return conf;
}

const LandUseType*
LandUseCatalog::getLandUse(const std::string& id) const
{
    auto iter = _lut.find(id);
    return iter != _lut.end() ? &iter->second : nullptr;
}

//...................................................................

Biome::Biome(const Config& conf, AssetCatalog* assets)
{
    conf.get("name", name());
    conf.get("id", id());

    ConfigSet mt = conf.children("modelcategory");
    for (const auto& c : mt)
    {
        modelCategories().push_back(new ModelCategory(c, assets));
    }
}

Config
Biome::getConfig() const
{
    Config conf("biome");
    conf.set("name", name());
    conf.set("id", id());
    //TODO
    OE_WARN << __func__ << " not implemented" << std::endl;

    return conf;
}

const ModelCategory*
Biome::getModelCategory(const std::string& name) const
{
    for (const auto i : modelCategories())
    {
        if (Strings::ciEquals(i->name().get(), name))
        {
            return i.get();
        }
    }
    return nullptr;
}

//..........................................................

BiomeCatalog::BiomeCatalog(const Config& conf)
{
    _assets = new AssetCatalog(conf.child("assetcatalog"));

    _landuse = new LandUseCatalog(conf.child("landusecatalog"));

    ConfigSet biomes_conf = conf.child("biomes").children("biome");
    for (const auto& b_conf : biomes_conf)
    {
        Biome* biome = new Biome(b_conf, _assets.get());
        _biomes[biome->id().get()] = biome;
    }
}

Config
BiomeCatalog::getConfig() const
{
    Config conf;
    //TODO
    OE_WARN << __func__ << " not implemented" << std::endl;
    return conf;
}

const Biome*
BiomeCatalog::getBiome(int id) const
{
    const auto i = _biomes.find(id);
    return i != _biomes.end() ? i->second.get() : nullptr;
}

void
BiomeCatalog::getBiomes(std::vector<osg::ref_ptr<const Biome>>& output) const
{
    output.clear();
    for (const auto i : _biomes)
        output.push_back(i.second);
}

const AssetCatalog*
BiomeCatalog::getAssets() const
{
    return _assets.get();
}

const LandUseCatalog*
BiomeCatalog::getLandUse() const
{
    return _landuse.get();
}
