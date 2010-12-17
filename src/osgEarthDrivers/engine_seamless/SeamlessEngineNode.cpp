/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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

#include "SeamlessEngineNode"
#include "Geographic"
#include "Projected"

namespace seamless
{
using namespace osgEarth;

// adapter that lets SeamlessEngineNode listen to Map events
// This should be a template or something.
struct SeamlessMapProxy : public MapCallback
{
    SeamlessMapProxy(SeamlessEngineNode* node) : _node(node) { }
    osg::observer_ptr<SeamlessEngineNode> _node;

    void onMapProfileEstablished( const Profile* profile ) {
        _node->onMapProfileEstablished(profile);
    }
    void onImageLayerAdded( ImageLayer* layer, unsigned int index ) {
        _node->onImageLayerAdded(layer, index);
    }
    void onImageLayerRemoved( ImageLayer* layer, unsigned int index ) {
        _node->onImageLayerRemoved(layer, index);
    }
    void onImageLayerMoved( ImageLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
        _node->onImageLayerMoved(layer,oldIndex,newIndex);
    }
    void onElevationLayerAdded( ElevationLayer* layer, unsigned int index ) {
        _node->onElevationLayerAdded(layer, index);
    }
    void onElevationLayerRemoved( ElevationLayer* layer, unsigned int index ) {
        _node->onElevationLayerRemoved(layer, index);
    }
    void onElevationLayerMoved( ElevationLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
        _node->onElevationLayerMoved(layer,oldIndex,newIndex);
    }
};

SeamlessEngineNode::SeamlessEngineNode()
    : _mapf(0)
{
}

SeamlessEngineNode::SeamlessEngineNode(const SeamlessEngineNode& rhs,
                       const osg::CopyOp& op)
    : TerrainEngineNode(rhs, op),
      _terrainOptions(rhs._terrainOptions), _mapf(0)
{
    _patchSet = static_cast<PatchSet*>(op(rhs._patchSet.get()));
    
}

SeamlessEngineNode::~SeamlessEngineNode()
{
    delete _mapf;
}

void SeamlessEngineNode::preInitialize(const Map* map, const TerrainOptions& options)
{
    TerrainEngineNode::preInitialize(map, options);
    _mapf = new MapFrame(map, Map::TERRAIN_LAYERS, "seamless");
    _terrainOptions.merge(options);
    if (map->getProfile())
        onMapProfileEstablished(map->getProfile());
    map->addMapCallback(new SeamlessMapProxy(this));
}

void SeamlessEngineNode::validateTerrainOptions(TerrainOptions& options)
{
}

void SeamlessEngineNode::onMapProfileEstablished(const Profile* mapProfile)
{
    const Map* map = getMap();
    int resolution = _terrainOptions.resolution().value();
    if (map->getMapOptions().coordSysType() == MapOptions::CSTYPE_GEOCENTRIC)
        _patchSet = new Geographic(map, _terrainOptions);
    else if (map->getMapOptions().coordSysType()
             == MapOptions::CSTYPE_PROJECTED)
        _patchSet = new Projected(map, _terrainOptions);
    else
    {
        OE_WARN << "map is not projected\n";
        return;
    }
    addChild(_patchSet
             ->createPatchSetGraph("bar.osgearth_engine_seamless_patch"));
}

void SeamlessEngineNode::onImageLayerAdded(ImageLayer*, unsigned int index)
{
}

void SeamlessEngineNode::onImageLayerRemoved(ImageLayer* layer,
                                             unsigned int index)
{
}

void SeamlessEngineNode::onImageLayerMoved(ImageLayer* layer,
                                           unsigned int oldIndex,
                                           unsigned int newIndex)
{
}

void SeamlessEngineNode::onElevationLayerAdded(ElevationLayer* layer,
                                               unsigned int index)
{
}

void SeamlessEngineNode::onElevationLayerRemoved(ElevationLayer* layer,
                                                 unsigned int index)
{
}

void SeamlessEngineNode::onElevationLayerMoved(ElevationLayer* layer,
                                               unsigned int oldIndex,
                                               unsigned int newIndex)
{
}

}
