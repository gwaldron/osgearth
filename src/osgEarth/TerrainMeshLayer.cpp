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
#include "TerrainMeshLayer"
#include "TileMesher"
#include "TerrainEngineNode"

using namespace osgEarth;

#define LC "[TerrainMeshLayer] "

REGISTER_OSGEARTH_LAYER(TerrainMesh, TerrainMeshLayer);

//...........................................................................

void TerrainMeshLayer::Options::fromConfig(const Config& conf)
{
    invertY().setDefault(false);

    conf.get("url", uri());
    conf.get("uri", uri());
    conf.get("invert_y", invertY());
}

Config
TerrainMeshLayer::Options::getConfig() const
{
    Config conf = TileLayer::Options::getConfig();
    conf.set("url", uri());
    conf.set("invert_y", invertY());
    return conf;
}

//...........................................................................

void
TerrainMeshLayer::init()
{
    TileLayer::init();

    setRenderType(RENDERTYPE_NONE);
}

Status
TerrainMeshLayer::openImplementation()
{
    auto s = TileLayer::openImplementation();
    if (s.isError()) return s;

    // Set up a rotating element in the template
    _template = options().uri()->full();
    auto start = _template.find('[');
    auto end = _template.find(']');
    if (start != std::string::npos && end != std::string::npos && end - start > 1)
    {
        _rotateString = _template.substr(start, end - start + 1);
        _rotateChoices = _template.substr(start + 1, end - start - 1);
    }

    return STATUS_OK;
}

void
TerrainMeshLayer::addedToMap(const Map* map)
{
    TileLayer::addedToMap(map);
    _map = map;
}

void
TerrainMeshLayer::removedFromMap(const Map* map)
{
    TileLayer::removedFromMap(map);
    _map = nullptr;
}

void
TerrainMeshLayer::prepareForRendering(TerrainEngine* engine)
{
    _engine = engine;
}

URI
TerrainMeshLayer::createURI(const TileKey& key) const
{
    unsigned x, y;
    key.getTileXY(x, y);
    unsigned cols = 0, rows = 0;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), cols, rows);
    unsigned inverted_y = rows - y - 1;

    if (options().invertY() == true)
    {
        y = inverted_y;
    }

    std::string location = _template;

    // support OpenLayers template style:
    replaceIn(location, "${x}", std::to_string(x));
    replaceIn(location, "${y}", std::to_string(y));
    replaceIn(location, "${-y}", std::to_string(inverted_y));
    replaceIn(location, "${z}", std::to_string(key.getLevelOfDetail()));

    // failing that, legacy osgearth style:
    replaceIn(location, "{x}", std::to_string(x));
    replaceIn(location, "{y}", std::to_string(y));
    replaceIn(location, "{-y}", std::to_string(inverted_y));
    replaceIn(location, "{z}", std::to_string(key.getLevelOfDetail()));

    std::string cacheKey;

    if (!_rotateChoices.empty())
    {
        cacheKey = location;
        unsigned index = (++_rotateIter) % _rotateChoices.size();
        replaceIn(location, _rotateString, Stringify() << _rotateChoices[index]);
    }

    return URI(location, options().uri()->context());
}

TileMesh
TerrainMeshLayer::createTile(
    const TileKey& key,
    ProgressCallback* progress) const
{
    //TODO: caching
    return createTileImplementation(key, progress);
}

TileMesh
TerrainMeshLayer::createTileImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    // Set up a tile mesher:
    TileMesher mesher;
    if (_engine)
        mesher.setTerrainOptions(_engine->getOptions());

    // process any constraints:
    TileMesher::Edits edits;
    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
    {
        mesher.getEdits(key, map.get(), edits, progress);
    }

    // create the mesh
    TileMesh mesh = mesher.createMesh(key, edits, progress);
    if (!mesh.indices.valid())
    {

        mesh.indices = mesher.getOrCreateStandardIndices();
    }

    return mesh;
}

void
TerrainMeshLayer::applyConstraints(const TileKey& key, TileMesh& mesh) const
{
    TileMesher mesher;
    if (_engine)
        mesher.setTerrainOptions(_engine->getOptions());

    // process any constraints:
    TileMesher::Edits edits;
    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
    {
        mesher.getEdits(key, map.get(), edits, nullptr);
        if (!edits.empty())
        {
            mesh = mesher.createMesh(key, mesh, edits, nullptr);
        }
    }
}
