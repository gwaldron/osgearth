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
    scale().setDefault(1.0f);
    stiffness().setDefault(0.0f);

    conf.get("url", modelURI());
    conf.get("name", name());
    conf.get("side_url", sideBillboardURI());
    conf.get("top_url", topBillboardURI());
    conf.get("width", width());
    conf.get("height", height());
    conf.get("scale", scale());
    conf.get("size_variation", sizeVariation());
    conf.get("stiffness", stiffness());
    conf.get("traits", traits());

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
    conf.set("scale", scale());
    conf.set("size_variation", sizeVariation());
    conf.set("stiffness", stiffness());
    conf.set("traits", traits());
    return conf;
}

//...................................................................

LifeMapTextureAsset::LifeMapTextureAsset(const Config& conf)
{
    conf.get("name", name());
    conf.get("url", uri());
    conf.get("size", size());
}

Config
LifeMapTextureAsset::getConfig() const
{
    Config conf("texture");
    conf.set("name", name());
    conf.set("url", uri());
    conf.set("size", size());
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

    ConfigSet modelassetgroups = conf.child("models").children("group");
    if (modelassetgroups.empty()) modelassetgroups = conf.child("modelassets").children("group");
    for (const auto& c : modelassetgroups)
    {
        AssetGroup::Type group = AssetGroup::type(c.value("name"));

        if (group < NUM_ASSET_GROUPS)
        {
            ConfigSet modelassets = c.children("asset");
            for (const auto& m : modelassets)
            {
                ModelAsset asset(m);
                asset.group() = group;
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

    Config models("Models");
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

#if 1
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
#endif

//...................................................................

Biome::Biome() :
    _index(-1),
    _parentBiome(nullptr),
    _implicit(false)
{
    //nop
}

Biome::Biome(const Config& conf, AssetCatalog* assetCatalog) :
    _index(-1),
    _parentBiome(nullptr),
    _implicit(false)
{
    conf.get("id", id());
    conf.get("name", name());
    conf.get("parent", parentId());
    conf.get("inherits_from", parentId());

    ConfigSet assets = conf.child("assets").children("asset");
    for (const auto& child : assets)
    {
        ModelAssetToUse m;
        m.asset() = assetCatalog->getModel(child.value("name"));
        if (m.asset())
        {
            m.weight() = 1.0f;
            child.get("weight", m.weight());
            m.coverage() = 1.0f;
            child.get("fill", m.coverage()); // backwards compat
            child.get("coverage", m.coverage());

            if (m.asset()->group() < NUM_ASSET_GROUPS)
            {
                _assetsToUse[m.asset()->group()].emplace_back(m);
            }
        }
    }
}

Config
Biome::getConfig() const
{
    Config conf("biome");
    conf.set("id", id());
    conf.set("name", name());
    conf.set("parent", parentId());

    //TODO
    OE_WARN << __func__ << " not implemented" << std::endl;
    // when we do implement this, take care to skip writing the asset pointers
    // when the parent biome is set.

    return conf;
}

const Biome::ModelAssetsToUse&
Biome::getModelAssetsToUse(int type) const
{   
    OE_HARD_ASSERT(type >= 0 && type < NUM_ASSET_GROUPS);

    const ModelAssetsToUse& assets = _assetsToUse[type];

    // use this list if it's not empty
    if (assets.empty() == false)
        return assets;

    // otherwise use the parent's list if there's a parent
    else if (_parentBiome != nullptr)
        return _parentBiome->getModelAssetsToUse(type);

    // otherwise just return this empty list.
    else
        return assets;
}

//..........................................................

BiomeCatalog::BiomeCatalog(const Config& conf) :
    _biomeIndexGenerator(1) // start at 1; 0 means "undefined"
{
    _assets = AssetCatalog(conf.child("assetcatalog"));

#if 0
    if (conf.hasChild("landuse_lifemap_table")) // not used
        _landUseTable = std::make_shared<LifeMapValueTable>(conf.child("landuse_lifemap_table"));

    if (conf.hasChild("landcover_lifemap_table")) // deprecated
        _landCoverTable = std::make_shared<LifeMapValueTable>(conf.child("landcover_lifemap_table"));
#endif

    ConfigSet biome_defs = conf.child("biomedefinitions").children("biome");
    if (biome_defs.empty())
        biome_defs = conf.child("biomecollection").children("biome"); // backwards compat
    if (biome_defs.empty())
        biome_defs = conf.child("biomes").children("biome"); // backwards compat

    // bring in all the raw biome definitions.
    for (const auto& b_conf : biome_defs)
    {
        Biome biome(b_conf, &_assets);
        biome._index = _biomeIndexGenerator++;
        _biomes_by_index[biome._index] = std::move(biome);
    }

    // resolve the explicit parent biome pointers.
    for (auto& index_and_biome : _biomes_by_index)
    {
        Biome& biome = index_and_biome.second;
        if (biome.parentId().isSet())
        {
            const Biome* parent = getBiome(biome.parentId().get());
            if (parent)
            {
                biome._parentBiome = parent;
            }
        }
    }

    // check for cyclical parent relationships, which are illegal
    for (auto& index_and_biome : _biomes_by_index)
    {
        Biome& biome = index_and_biome.second;
        std::vector<const Biome*> visited;
        visited.push_back(&biome);

        const Biome* parent = biome._parentBiome;
        while (parent != nullptr)
        {
            if (std::find(visited.begin(), visited.end(), parent) != visited.end())
            {
                std::ostringstream buf;
                for (auto& bptr : visited)
                    buf << bptr->id().get() << " -> ";
                buf << parent->id().get();
                OE_WARN << LC << "***** I detected a parent loop in the biome catalog: " << buf.str() << std::endl;
                biome._parentBiome = nullptr;
                break;
            }
            else
            {
                visited.push_back(parent);
                parent = parent->_parentBiome;
            }
        }
    }

    // next, scan the biomes for any assets with traits, and add a
    // new biome for each combination
    for (auto& index_and_biome : _biomes_by_index)
    {
        Biome& biome = index_and_biome.second;

        // skip implicit biomes since we already processed them
        if (biome._implicit)
            continue;

        // Collect all assets with traits so we can make a biome for each.
        // traverse "up" through the parent biomes to find them all.
        std::unordered_map<
            std::string,
            std::set<Biome::ModelAssetToUse>[NUM_ASSET_GROUPS]> traits_table;

        for(const Biome* biome_ptr = &biome; 
            biome_ptr != nullptr;
            biome_ptr = biome_ptr->_parentBiome)
        {
            for(int g=0; g<NUM_ASSET_GROUPS; ++g)
            {
                const Biome::ModelAssetsToUse& assetsToUse = biome_ptr->getModelAssetsToUse(g);
                int count = 0;
                for (auto& asset_ptr : assetsToUse)
                {
                    const std::string& traits = asset_ptr.asset()->traits().get();
                    if (!traits.empty())
                    {
                        traits_table[traits][g].insert(asset_ptr);
                    }
                }
            }
        }

        // for each filter found, make a new subbiome and copy all the corresponding
        // assets to it.
        for(auto& iter : traits_table)
        {
            const std::string& traits = iter.first;
            auto& traits_asset_groups = iter.second;

            // create a new biome for this traits if it doesn't already exist:
            std::string sub_biome_id = biome.id().get() + "." + traits;
            const Biome* sub_biome = getBiome(sub_biome_id);
            if (sub_biome == nullptr)
            {
                // lastly, insert it into the index.
                int new_index = _biomeIndexGenerator++;
                Biome& new_biome = _biomes_by_index[new_index];
                new_biome._index = new_index;

                new_biome.id() = sub_biome_id;
                new_biome.name() = biome.name().get() + " (" + traits + ")";

                // marks this biome as one that was derived from traits data,
                // not defined by the configuration.
                new_biome._implicit = true;

                // By default the parent is unset, but this may change in the
                // search block that follows. An implicit biome without a 
                // similarly filtered parent should render nothing.
                new_biome.parentId().clear();

                // serach "up the chain" to see if there's a parent that can 
                // support fallback for this particular filter
                const Biome* ptr = &biome;
                while (ptr)
                {
                    // find the natural parent; if none, bail.
                    const Biome* parent = getBiome(ptr->parentId().get());
                    if (parent)
                    {
                        // find a selector variation of the natural parent. If found, success.
                        std::string parent_with_traits_id = parent->id().get() + "." + traits;
                        const Biome* temp = getBiome(parent_with_traits_id);
                        if (temp)
                        {
                            new_biome.parentId() = parent_with_traits_id;
                            new_biome._parentBiome = temp;
                            break;
                        }
                    }

                    // if not found, parent up and try again.
                    ptr = parent;
                }

                // copy over asset pointers for each asset matching the traits
                for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
                {
                    for (auto& asset_ptr : traits_asset_groups[i])
                    {
                        new_biome._assetsToUse[i].push_back(asset_ptr);
                    }
                }
            }
        }
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
BiomeCatalog::getBiomeByIndex(int index) const
{
    const auto i = _biomes_by_index.find(index);
    return i != _biomes_by_index.end() ? &i->second : nullptr;
}

const Biome*
BiomeCatalog::getBiome(const std::string& id) const
{
    for (auto& iter : _biomes_by_index)
    {
        if (iter.second.id().get() == id)
        {
            return &iter.second;
        }
    }
    return nullptr;
}

std::vector<const Biome*>
BiomeCatalog::getBiomes() const
{
    std::vector<const Biome*> result;
    for (auto& iter : _biomes_by_index)
        result.push_back(&iter.second);
    return std::move(result);
}

const AssetCatalog&
BiomeCatalog::getAssets() const
{
    return _assets;
}

#if 1
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
#endif
