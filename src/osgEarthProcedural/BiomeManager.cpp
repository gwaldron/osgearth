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
#include "BiomeManager"

using namespace osgEarth;
using namespace osgEarth::Procedural;

#undef LC
#define LC "[BiomeManager] "

#define OE_TEST OE_NULL

#define NORMAL_MAP_TEX_UNIT 1
#define PBR_TEX_UNIT 2

//...................................................................


BiomeManager::BiomeManager()
{
    _assetManager = std::make_shared<ModelAssetManager>();
}

void
BiomeManager::ref(const Biome* biome)
{
    std::lock_guard<std::mutex> lock(_mutex);

    auto item = _refs.emplace(biome, 0);
    ++item.first->second;
    if (item.first->second == 1) // ref count of 1 means it's new
    {
        _assetManager->revision++;
        OE_TEST << LC << "Hello, " << biome->name().get() << " (" << biome->index() << ")" << std::endl;
    }
}

void
BiomeManager::unref(const Biome* biome)
{
    std::lock_guard<std::mutex> lock(_mutex);

    auto iter = _refs.find(biome);

    // silent assertion
    if (iter == _refs.end() || iter->second == 0)
    {
        OE_SOFT_ASSERT(iter != _refs.end() && iter->second != 0);
        return;
    }

    if (!_locked)
    {
        --iter->second;
        if (iter->second == 0)
        {
            // Technically yes, this creates a new revision, but it
            // would also result in re-loading assets that are already
            // resident... think on this -gw
            //_assetManager->revision++;
            OE_TEST << LC << "Goodbye, " << biome->name().get() << "(" << biome->index() << ")" << std::endl;
        }
    }
}

int
BiomeManager::getRevision() const
{
    return _assetManager->revision;
}

void
BiomeManager::reset()
{
    // reset the reference counts, and bump the revision so the
    // next call to update will remove any resident data
    {
        std::lock_guard<std::mutex> lock(_mutex);

        for (auto& iter : _refs)
        {
            const Biome* biome = iter.first;
            OE_TEST << LC << "Goodbye, " << biome->name().get() << std::endl;
            iter.second = 0;
        }

        _assetManager->revision++;
    }

    // Resolve the references and unload any resident assets from memory.
    recalculateResidentBiomes();
}

void
BiomeManager::flush()
{
    recalculateResidentBiomes();

    _assetManager->getTextures()->flush();
    //if (_textures.valid())
    //    _textures->flush();
}

void
BiomeManager::recalculateResidentBiomes()
{
    // Figure out which biomes we need to load and which we can discard.
    {
        std::lock_guard<std::mutex> lock(_mutex);

        std::vector<const Biome*> biomes_to_add;
        std::vector<const Biome*> biomes_to_remove;

        for (auto& ref : _refs)
        {
            const Biome* biome = ref.first;
            int refcount = ref.second;

            if (refcount > 0)
            {
                biomes_to_add.push_back(biome);
            }
            else if (!_locked)
            {
                biomes_to_remove.push_back(biome);
            }
        }

        // Update the resident biome data structure:

        // add biomes that might need adding
        for (auto biome : biomes_to_add)
        {
            auto& entry = _residentBiomes[biome->id()];
            entry.biome = biome;
        }

        // get rid of biomes we no longer need.
        for (auto biome : biomes_to_remove)
        {
            _residentBiomes.erase(biome->id());
        }
    }

    // finally, update the collection of resident assets to
    // reflect the reference counts.
    _assetManager->recalculateResidentAssets();
}

std::vector<const Biome*>
BiomeManager::getActiveBiomes() const
{
    std::lock_guard<std::mutex> lock(_mutex);

    std::vector<const Biome*> result;

    for (auto& ref : _refs)
    {
        if (ref.second > 0)
            result.push_back(ref.first);
    }
    return result;
}