/* osgEarth
* Copyright 2008-2014 Pelican Mapping
* MIT License
*/
#include "EngineContext"
#include "TileNodeRegistry"
#include <osgEarth/CullingUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

using namespace osgEarth::REX;
using namespace osgEarth;

#define LC "[EngineContext] "

EngineContext::EngineContext(
    const Map* map,
    TerrainEngineNode* terrainEngine,
    GeometryPool* geometryPool,
    Merger* merger,
    TileNodeRegistry::Ptr          tiles,
    const RenderBindings& renderBindings,
    const SelectionInfo& selectionInfo,
    const FrameClock* clock) :

    _map(map),
    _terrainEngine(terrainEngine),
    _geometryPool(geometryPool),
    _merger(merger),
    _tiles(tiles),
    _renderBindings(renderBindings),
    _options(terrainEngine->getOptions()),
    _selectionInfo(selectionInfo),
    _tick(0),
    _tilesLastCull(0),
    _clock(clock)
{
    _bboxCB = new ModifyBoundingBoxCallback(this);

    // create a bindless texture arena and set it to automatically
    // release textures that the terrain no longer references.
    _textures = new TextureArena();
    _textures->setName("REX Terrain Engine");
    _textures->setBindingPoint(29); // TODO
    _textures->setAutoRelease(true);

    // texture limiting :(
    int maxSize = std::min(
        (int)_options.getMaxTextureSize(),
        Registry::instance()->getMaxTextureSize());

    _textures->setMaxTextureSize(maxSize);
}

osg::ref_ptr<const Map>
EngineContext::getMap() const
{
    osg::ref_ptr<const Map> map;
    _map.lock(map);
    return map;
}
