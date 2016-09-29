/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include "TerrainCuller"
#include "TileNode"
#include "SurfaceNode"

using namespace osgEarth::Drivers::RexTerrainEngine;


TerrainCuller::TerrainCuller()
{
    setVisitorType(CULL_VISITOR);
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
}

void
TerrainCuller::setup(const MapFrame& frame, const RenderBindings& bindings)
{
    _drawable = new TerrainDrawable();
    _drawable->setup(frame, bindings);
}

void
TerrainCuller::apply(osg::Node& node)
{
    // push the node's state.
    osg::StateSet* node_state = node.getStateSet();

    TileNode* tileNode = dynamic_cast<TileNode*>(&node);
    if (tileNode)
    {
        _currentTileNode = tileNode;
    }

    else
    {
        SurfaceNode* surface = dynamic_cast<SurfaceNode*>(&node);
        if (surface)
        {
            // push the surface matrix:
            osg::RefMatrix* matrix = createOrReuseMatrix(*getModelViewMatrix());
            surface->computeLocalToWorldMatrix(*matrix, this);
            pushModelViewMatrix(matrix, osg::Transform::ABSOLUTE_RF);
            
            const RenderBindings&    bindings = _context->getRenderBindings();
            const TileRenderingData& data     = _currentTileNode->renderingData();

            for (unsigned p = 0; p < data._passes.size(); ++p)
            {
                const RenderingPass& pass = data._passes[p];

                UID uid = pass._sourceUID;

                if (uid < 0 && data._passes.size() > 1)
                    continue;

                // add a new Draw command to the appropriate layer
                DrawLayerCommand& layer = _drawable->layer(uid);
                layer._tiles.push_back(DrawTileCommand());
                DrawTileCommand& tile = layer._tiles.back();

                tile._key = _currentTileNode->getTileKey();
                tile._pass = pass; // pointer instead?
                            
                tile._matrix = surface->getMatrix();
                tile._modelViewMatrix = *this->getModelViewMatrix();
                tile._keyValue = _currentTileNode->getTileKeyValue();
                tile._geom = surface->getDrawable()->_geom.get();
                tile._morphConstants = _currentTileNode->getMorphConstants();

                const osg::Image* elevRaster = _currentTileNode->getElevationRaster();
                if (elevRaster)
                {
                    float size = (float)elevRaster->s();
                    tile._elevTexelCoeff.set((size - 1.0f) / size, 0.5 / size);
                }
                else
                {
                    tile._elevTexelCoeff.set(1.0f, 0.0f);
                }
            }                

            _drawable->_bs.expandBy(surface->getBound());
            _drawable->_box.expandBy(_drawable->_bs);

            // pop the surface matrix:
            popModelViewMatrix();
        }
    }

    traverse(node);
}
