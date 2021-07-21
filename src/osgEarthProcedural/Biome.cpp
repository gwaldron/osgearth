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

namespace
{
    static std::string asset_group_names[2] = {
        "trees",
        "undergrowth"
    };
}


AssetGroup::Type
AssetGroup::type(const std::string& name)
{
    if (name == asset_group_names[TREES])
        return TREES;
    else if (name == asset_group_names[UNDERGROWTH])
        return UNDERGROWTH;
    else
        return UNDEFINED;
}

std::string
AssetGroup::name(AssetGroup::Type group)
{

    return group < NUM_ASSET_GROUPS ?
        asset_group_names[group] :
        std::string("undefined");
}


ModelAsset::ModelAsset(const Config& conf)
{
    conf.get("url", modelURI());
    conf.get("name", name());
    conf.get("side_url", sideBillboardURI());
    conf.get("top_url", topBillboardURI());
    conf.get("width", width());
    conf.get("height", height());
    conf.get("size_variation", sizeVariation());

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
    _lifemapMatrixWidth = as<unsigned>(conf.child("lifemaptextures").value("width"), 1);
    _lifemapMatrixHeight = as<unsigned>(conf.child("lifemaptextures").value("height"), 1);

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

    ConfigSet modelassetgroups = conf.child("modelassets").children("group");
    for (const auto& c : modelassetgroups)
    {
        AssetGroup::Type group = AssetGroup::type(c.value("name"));

        if (group < NUM_ASSET_GROUPS)
        {
            ConfigSet modelassets = c.children("asset");
            for (const auto& m : modelassets)
            {
                ModelAsset asset(m);
                asset._group = group;
                _models[asset.name().get()] = asset;
            }
        }
    }
}

Config
AssetCatalog::getConfig() const
{
    Config conf("AssetCatalog");

    //TODO - incomplete/wrong

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

    Config models("ModelAssets");
    for (auto& model : _models)
        models.add(model.second.getConfig());
    if (!models.empty())
        conf.add(models);

    return conf;
}

unsigned
AssetCatalog::getLifeMapMatrixWidth() const
{
    return _lifemapMatrixWidth;
}

unsigned
AssetCatalog::getLifeMapMatrixHeight() const
{
    return _lifemapMatrixHeight;
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

Biome::Biome(const Config& conf, AssetCatalog* assetCatalog)
{
    conf.get("name", name());
    conf.get("id", id());

    ConfigSet assets = conf.child("assets").children("asset");
    for (const auto& child : assets)
    {
        ModelAssetPointer m;
        m.asset = assetCatalog->getModel(child.value("name"));
        if (m.asset)
        {
            m.weight = 1.0f;
            child.get("weight", m.weight);
            m.fill = 1.0f;
            child.get("fill", m.fill);

            if (m.asset->_group < NUM_ASSET_GROUPS)
                _modelgroups[m.asset->_group].emplace_back(m);
        }
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
