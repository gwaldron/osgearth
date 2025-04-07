/* osgEarth
* Copyright 2008-2014 Pelican Mapping
* MIT License
*/
#include "LoadTileData"
#include "SurfaceNode"
#include "TileNode"
#include "EngineContext"

#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Terrain>
#include <osgEarth/Metrics>
#include <osg/NodeVisitor>

using namespace osgEarth::REX;
using namespace osgEarth;

#define LC "[LoadTileData] "

LoadTileDataOperation::LoadTileDataOperation(
    TileNode* tilenode, 
    EngineContext* context) :

    _tilenode(tilenode),
    _enableCancel(true),
    _dispatched(false),
    _merged(false)
{
    _engine = context->getEngine();
    _name = tilenode->getKey().str();
}

LoadTileDataOperation::LoadTileDataOperation(
    const CreateTileManifest& manifest, 
    TileNode* tilenode, 
    EngineContext* context) :

    _manifest(manifest),
    _tilenode(tilenode),
    _enableCancel(true),
    _dispatched(false),
    _merged(false)
{
    _engine = context->getEngine();
    _name = tilenode->getKey().str();
}

LoadTileDataOperation::~LoadTileDataOperation()
{
    //if (!_dispatched || !_merged)
    //{
    //    OE_INFO << _name << " dispatched=" << _dispatched << " merged=" << _merged << std::endl;
    //}
}

bool
LoadTileDataOperation::dispatch(bool async)
{
    // Make local copies that we want to pass to the lambda
    osg::ref_ptr<TerrainEngineNode> engine;
    if (!_engine.lock(engine))
        return false;

    osg::ref_ptr<const Map> map = engine->getMap();
    if (!map.valid())
        return false;

    _dispatched = true;

    CreateTileManifest manifest(_manifest);
    bool enableCancel = _enableCancel;

    TileKey key(_tilenode->getKey());

    auto load = [engine, map, key, manifest, enableCancel] (Cancelable& progress)
    {
        osg::ref_ptr<ProgressCallback> wrapper =
            enableCancel ? new ProgressCallback(&progress) : nullptr;

        osg::ref_ptr<TerrainTileModel> result = engine->createTileModel(
            map.get(),
            key,
            manifest,
            wrapper.get());

        return result;
    };

    // Priority function. This return the maximum priority if the tile
    // has disappeared so that it will be immediately rejected from the job queue.
    // You can change it to -FLT_MAX to let it fester on the end of the queue,
    // but that may slow down the job queue's sorting algorithm.
    osg::observer_ptr<TileNode> tile_obs(_tilenode);
    auto priority_func = [tile_obs]() -> float
    {
        if (tile_obs.valid() == false) return FLT_MAX; // quick trivial reject
        osg::ref_ptr<TileNode> tilenode;
        return tile_obs.lock(tilenode) ? tilenode->getLoadPriority() : FLT_MAX;
    };


    if (async)
    {
        jobs::context context;
        context.pool = jobs::get_pool(ARENA_LOAD_TILE);
        context.priority = priority_func;
        _result = jobs::dispatch(load, context);
    }
    else
    {
        Cancelable c;
        _result.resolve(load(c));
    }

    return true;
}


bool
LoadTileDataOperation::merge()
{
    _merged = true;

    // context went out of scope - bail
    osg::ref_ptr<TerrainEngineNode> engine;
    if (!_engine.lock(engine))
        return true;

    // map went out of scope - bail
    osg::ref_ptr<const Map> map = engine->getMap();
    if (!map.valid())
        return true;

    // tilenode went out of scope - bail
    osg::ref_ptr<TileNode> tilenode;
    if (!_tilenode.lock(tilenode))
        return true;

    // no data model at all - done
    // GW: should never happen.
    if (!_result.available())
    {
        OE_WARN << tilenode->getKey().str() << " bailing out of merge b/c data model is NULL" << std::endl;
        return false;
    }

    OE_SOFT_ASSERT_AND_RETURN(_result.available(), false);

    OE_PROFILING_ZONE;

    const osg::ref_ptr<TerrainTileModel>& model = _result.value(); //.get();

    // Check the map data revision and scan the manifest and see if any
    // revisions don't match the revisions in the original manifest.
    // If there are mismatches, that means the map has changed since we
    // submitted this request, and the results are now invalid.
    if (model->revision != map->getDataModelRevision() ||
        _manifest.inSyncWith(map.get()) == false)
    {
        // wipe the data model, update the revisions, and try again.
        _manifest.updateRevisions(map.get());
        _tilenode->refreshLayers(_manifest);
        return false;
    }

    // Merge the new data into the tile.
    tilenode->merge(model.get(), _manifest);

    return true;
}
