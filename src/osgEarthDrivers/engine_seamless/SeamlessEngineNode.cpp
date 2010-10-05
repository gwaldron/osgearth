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

SeamlessEngineNode::SeamlessEngineNode()
  : _verticalScale(1.0)
{
}
SeamlessEngineNode::SeamlessEngineNode(const SeamlessEngineNode& rhs,
                       const osg::CopyOp& op)
    : TerrainEngineNode(rhs, op), _verticalScale(rhs._verticalScale),
      _terrainOptions(rhs._terrainOptions)
{
    _patchSet = static_cast<PatchSet*>(op(rhs._patchSet.get()));
    
}

SeamlessEngineNode::~SeamlessEngineNode()
{
}

void SeamlessEngineNode::initialize(Map* map, const TerrainOptions& options)
{
    TerrainEngineNode::initialize(map, options);
    _terrainOptions = options;
    _verticalScale = options.verticalScale().value();
    if (map->getProfile())
        onMapProfileEstablished(map->getProfile());
}

void SeamlessEngineNode::validateTerrainOptions(TerrainOptions& options)
{
}

void SeamlessEngineNode::onMapProfileEstablished(const Profile* mapProfile)
{
    Map* map = getMap();
    if (map->getMapOptions().coordSysType() == MapOptions::CSTYPE_GEOCENTRIC)
        _patchSet = new Geographic(map);
    else if (map->getMapOptions().coordSysType()
             == MapOptions::CSTYPE_PROJECTED)
        _patchSet = new Projected(map);
    else
    {
        OSG_WARN << "map is not projected\n";
        return;
    }
    _patchSet->setVerticalScale(_verticalScale);
    addChild(_patchSet->createPatchSetGraph("bar.tengpatch"));
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
