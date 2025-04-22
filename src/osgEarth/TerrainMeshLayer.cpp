/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "TerrainMeshLayer"
#include "TileMesher"
#include "TerrainEngineNode"
#include "TerrainConstraintLayer"

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
    Config conf = super::getConfig();
    conf.set("url", uri());
    conf.set("invert_y", invertY());
    return conf;
}

//...........................................................................

void
TerrainMeshLayer::init()
{
    super::init();

    setRenderType(RENDERTYPE_NONE);

    // layer is always visible:
    _canSetVisible = false;
}

Status
TerrainMeshLayer::openImplementation()
{
    OE_RETURN_STATUS_ON_ERROR(super::openImplementation());

    return STATUS_OK;
}

void
TerrainMeshLayer::addedToMap(const Map* map)
{
    super::addedToMap(map);
    _map = map;
}

void
TerrainMeshLayer::removedFromMap(const Map* map)
{
    super::removedFromMap(map);
    _map = nullptr;
}

void
TerrainMeshLayer::prepareForRendering(TerrainEngine* engine)
{
    _engine = engine;
}

TileMesh
TerrainMeshLayer::createTile(const TileKey& key, ProgressCallback* progress) const
{
    //TODO: caching
    return createTileImplementation(key, progress);
}

TileMesh
TerrainMeshLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    // Set up a tile mesher:
    TileMesher mesher;
    if (_engine)
        mesher.setTerrainOptions(_engine->getOptions());

    // process any constraints:
    MeshConstraints edits;

    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
    {
        TerrainConstraintQuery query(map.get());
        query.getConstraints(key, edits, progress);
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
    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
    {
        TileMesher mesher;
        if (_engine)
            mesher.setTerrainOptions(_engine->getOptions());

        MeshConstraints edits;

        TerrainConstraintQuery query(map.get());
        if (query.getConstraints(key, edits, {}))
        {
            mesh = mesher.createMesh(key, mesh, edits, nullptr);
        }
    }
}
