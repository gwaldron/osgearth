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

    // save the original so the user can extract user-defined values
    _sourceConfig = conf;
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

LifeMapTextureAsset::LifeMapTextureAsset(const Config& conf)
{
    code().setDefault(0);

    conf.get("name", name());
    conf.get("url", uri());
    conf.get("width", width());
    conf.get("height", height());
}

Config
LifeMapTextureAsset::getConfig() const
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
    ConfigSet texturesConf = conf.child("lifemaptextures").children("texture");
    for (const auto& c : texturesConf)
    {
        _textures.emplace_back(c);
    }
    ConfigSet specialConf = conf.child("specialtextures").children("texture");
    for (const auto& c : specialConf)
    {
        _specialTextures.emplace_back(c);
    }

    ConfigSet modelsConf = conf.child("models").children("model");
    for (const auto& c : modelsConf)
    {
        ModelAsset asset(c);
        _models[asset.name().get()] = asset;
    }
}

Config
AssetCatalog::getConfig() const
{
    Config conf("AssetCatalog");

    Config textures("LifeMapTextures");
    for (auto& texture : _textures)
        textures.add(texture.getConfig());
    if (!textures.empty())
        conf.add(textures);

    Config specifictextures("SpecificTextures");
    for (auto& texture : _specialTextures)
        specifictextures.add(texture.getConfig());
    if (!specifictextures.empty())
        conf.add(specifictextures);

    Config models("Models");
    for (auto& model : _models)
        models.add(model.second.getConfig());
    if (!models.empty())
        conf.add(models);

    return conf;
}

const std::vector<LifeMapTextureAsset>&
AssetCatalog::getLifeMapTextures() const
{
    return _textures;
}

const std::vector<LifeMapTextureAsset>&
AssetCatalog::getSpecialTextures() const
{
    return _specialTextures;
}

const ModelAsset*
AssetCatalog::getModel(const std::string& name) const
{
    auto i = _models.find(name);
    return i != _models.end() ? &i->second : nullptr;
}

bool
AssetCatalog::empty() const
{
    return _models.empty() && _textures.empty();
}

//..........................................................

ModelCategory::ModelCategory(const Config& conf, AssetCatalog* assets)
{
    conf.get("name", name());
    ConfigSet mc = conf.children("model");
    for (const auto& c : mc)
    {
        const ModelAsset* asset = assets->getModel(c.value("name"));
        if (asset)
        {
            Member member;
            member.asset = asset;
            member.weight = (float)c.value("weight", 1.0f);
            member.fill = (float)c.value("fill", 1.0f);
            _members.push_back(std::move(member));
        }
        else
        {
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

LifeMapValue::LifeMapValue(const Config& conf)
{
    dense().setDefault(0.0f);
    lush().setDefault(0.0f);
    rugged().setDefault(0.5f);

    conf.get("id", id());
    conf.get("dense", dense());
    conf.get("lush", lush());
    conf.get("rugged", rugged());
    conf.get("special", special());
}

Config
LifeMapValue::getConfig() const
{
    Config conf;
    //TODO
    OE_WARN << __func__ << " not implemented" << std::endl;
    return conf;
}

//...................................................................

LifeMapValueTable::LifeMapValueTable(const Config& conf)
{
    const ConfigSet& children = conf.child("classes").children();

    if (children.size() > 0)
        values().reserve(children.size());

    for (const auto& child : children)
    {
        if (!child.empty())
        {
            LifeMapValue type(child);

            if (type.id().isSet())
            {
                values().push_back(type);
                const LifeMapValue* ptr = &values().back();
                _lut[type.id().get()] = type;
            }
        }
    }
}

Config 
LifeMapValueTable::getConfig() const
{
    Config conf;
    //TODO
    OE_WARN << __func__ << " not implemented" << std::endl;
    return conf;
}

const LifeMapValue*
LifeMapValueTable::getValue(const std::string& id) const
{
    auto iter = _lut.find(id);
    return iter != _lut.end() ? &iter->second : nullptr;
}

//...................................................................

Biome::Biome(const Config& conf, AssetCatalog* assets)
{
    conf.get("name", name());
    conf.get("id", id());

    ConfigSet categories = conf.children("modelcategory");
    for (const auto& child : categories)
    {
        modelCategories().emplace_back(child, assets);
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
    for (const auto& i : modelCategories())
    {
        if (Strings::ciEquals(i.name().get(), name))
        {
            return &i;
        }
    }
    return nullptr;
}

//..........................................................

BiomeCatalog::BiomeCatalog(const Config& conf)
{
    _assets = AssetCatalog(conf.child("assetcatalog"));

    if (conf.hasChild("landuse_lifemap_table"))
        _landUseTable = std::make_shared<LifeMapValueTable>(conf.child("landuse_lifemap_table"));

    if (conf.hasChild("landcover_lifemap_table"))
        _landCoverTable = std::make_shared<LifeMapValueTable>(conf.child("landcover_lifemap_table"));

    ConfigSet biomes_conf = conf.child("biomes").children("biome");
    for (const auto& b_conf : biomes_conf)
    {
        Biome biome(b_conf, &_assets);
        _biomes[biome.id().get()] = std::move(biome);
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
    return i != _biomes.end() ? &i->second : nullptr;
}

std::vector<const Biome*>
BiomeCatalog::getBiomes() const
{
    std::vector<const Biome*> result;
    for (auto& i : _biomes)
        result.push_back(&i.second);
    return std::move(result);
}

const AssetCatalog&
BiomeCatalog::getAssets() const
{
    return _assets;
}

const LifeMapValueTable*
BiomeCatalog::getLandUseTable() const
{
    return _landUseTable.get();
}

const LifeMapValueTable*
BiomeCatalog::getLandCoverTable() const
{
    return _landCoverTable.get();
}
