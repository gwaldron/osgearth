/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

    auto load = [engine, map, key, manifest, enableCancel] (Cancelable* progress)
    {
        osg::ref_ptr<ProgressCallback> wrapper =
            enableCancel ? new ProgressCallback(progress) : nullptr;

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
        Job job;
        job.setArena(ARENA_LOAD_TILE);
        job.setPriorityFunction(priority_func);
        _result = job.dispatch<LoadResult>(load);
    }
    else
    {
        Promise<LoadResult> promise("LoadTileData(OE)");
        _result = promise.getFuture();
        promise.resolve(load(nullptr));
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
    if (!_result.isAvailable())
    {
        OE_WARN << tilenode->getKey().str() << " bailing out of merge b/c data model is NULL" << std::endl;
        return false;
    }

    OE_SOFT_ASSERT_AND_RETURN(_result.isAvailable(), false);

    OE_PROFILING_ZONE;

    const osg::ref_ptr<TerrainTileModel>& model = _result.get();

    // Check the map data revision and scan the manifest and see if any
    // revisions don't match the revisions in the original manifest.
    // If there are mismatches, that means the map has changed since we
    // submitted this request, and the results are now invalid.
    if (model->getRevision() != map->getDataModelRevision() ||
        _manifest.inSyncWith(map.get()) == false)
    {
        // wipe the data model, update the revisions, and try again.
        _manifest.updateRevisions(map.get());
        OE_DEBUG << LC << "Request for tile " << tilenode->getKey().str() << " out of date and will be requeued" << std::endl;
        return false;
    }

    // Merge the new data into the tile.
    tilenode->merge(model.get(), _manifest);

    return true;
}
