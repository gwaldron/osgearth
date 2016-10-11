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

#define LC "[TerrainCuller] "

using namespace osgEarth::Drivers::RexTerrainEngine;


TerrainCuller::TerrainCuller() :
_frame(0L),
_context(0L),
_camera(0L),
_currentTileNode(0L)
{
    setVisitorType(CULL_VISITOR);
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
}

void
TerrainCuller::setup(const MapFrame& frame, const RenderBindings& bindings, osg::StateSet* defaultStateSet)
{
    _terrain.setup(frame, bindings, defaultStateSet);
}

float
TerrainCuller::getDistanceToViewPoint(const osg::Vec3& pos, bool withLODScale) const
{
    if (withLODScale) return (pos-getViewPointLocal()).length()*getLODScale();
    else return (pos-getViewPointLocal()).length();
}

DrawTileCommand*
TerrainCuller::addDrawCommand(UID uid, const RenderingPass& pass, TileNode* tileNode)
{
    SurfaceNode* surface = tileNode->getSurfaceNode();

    const RenderBindings&  bindings = _context->getRenderBindings();

    //UID uid = pass._sourceUID;

    // skip layers that are not visible:
    if (pass._imageLayer.valid() && !pass._imageLayer->getVisible())
        return 0L;

    // add a new Draw command to the appropriate layer
    osg::ref_ptr<LayerDrawable> layer = _terrain.layer(uid);
    if (layer.valid())
    {
        layer->_tiles.push_back(DrawTileCommand());
        DrawTileCommand& tile = layer->_tiles.back();

        // install everything we need in the Draw Command:
        tile._pass = &pass;
        tile._matrix = surface->getMatrix();
        tile._modelViewMatrix = *this->getModelViewMatrix();
        tile._keyValue = tileNode->getTileKeyValue();
        tile._geom = surface->getDrawable()->_geom.get();
        tile._morphConstants = tileNode->getMorphConstants();
        tile._key = tileNode->getTileKey();

        // TODO: find a faster way?
        //osg::Matrix im;
        //im.invert(tile._matrix);
        osg::Vec3 c = surface->getBound().center() * surface->getInverseMatrix();
        tile._range = getDistanceToViewPoint(c, true);

        const osg::Image* elevRaster = tileNode->getElevationRaster();
        if (elevRaster)
        {
            float size = (float)elevRaster->s();
            tile._elevTexelCoeff.set((size - 1.0f) / size, 0.5 / size);
        }

        if (pass._patchLayer.valid() && pass._patchLayer->getDrawCallback())
        {
            tile._drawCallback = pass._patchLayer->getDrawCallback();
        }

        return &tile;
    }
    else
    {
        OE_WARN << LC << "Internal error - _terrain.layer(uid=" << uid << ") returned NULL\n";
        return 0L;
    }
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
        
        // todo: check for patch/virtual
        const RenderBindings&  bindings    = _context->getRenderBindings();
        TileRenderModel& renderModel = _currentTileNode->renderModel();

        bool pushedMatrix = false;

        for (unsigned p = 0; p < renderModel._passes.size(); ++p)
        {
            RenderingPass& pass = renderModel._passes[p];

            if (pass._layer.valid() &&
                pass._layer->getRenderType() == Layer::RENDERTYPE_PATCH)
            {
                // If this pass uses another pass's samplers, copy them over now.
                if (pass._surrogatePass >= 0)
                {
                    pass._surrogateSamplers = &renderModel._passes[pass._surrogatePass]._samplers;
                }

                if (!pushedMatrix)
                {
                    SurfaceNode* surface = tileNode->getSurfaceNode();

                    // push the surface matrix:
                    osg::Matrix mvm = *getModelViewMatrix();
                    surface->computeLocalToWorldMatrix(mvm, this);
                    pushModelViewMatrix(createOrReuseMatrix(mvm), surface->getReferenceFrame());
                    pushedMatrix = true;
                }

                DrawTileCommand* cmd = addDrawCommand(pass._sourceUID, pass, tileNode);
                if (cmd)
                {
                    cmd->_drawPatch = true;
                }
            }
        }

        if (pushedMatrix)
        {
            popModelViewMatrix();
        }
    }

    else
    {
        SurfaceNode* surface = dynamic_cast<SurfaceNode*>(&node);
        if (surface)
        {            
            const TileRenderModel& renderModel = _currentTileNode->renderModel();

            // push the surface matrix:
            osg::Matrix mvm = *getModelViewMatrix();
            surface->computeLocalToWorldMatrix(mvm, this);
            pushModelViewMatrix(createOrReuseMatrix(mvm), surface->getReferenceFrame());

            for (unsigned p = 0; p < renderModel._passes.size(); ++p)
            {
                const RenderingPass& pass = renderModel._passes[p];
                
                if (pass._sourceUID < 0 || pass._layer->getRenderType() == Layer::RENDERTYPE_TILE)
                {
                    addDrawCommand(pass._sourceUID, pass, _currentTileNode);
                }
            }

            // Add a draw command for each "global" layer (overlays, derivatives)
            if (!renderModel._passes.empty())
            {
                if (!_terrain.globalLayers().empty())
                {
                    // Use the default pass - later we may provide the option to bind to 
                    // a specific layer's rendering pass so we bind its COLOR sampler.
                    const RenderingPass* pass = renderModel.getPass(-1);
                
                    for (unsigned t = 0; t < _terrain.globalLayers().size(); ++t)
                    {
                        Layer* layer = _terrain.globalLayers().at(t);
                        addDrawCommand(layer->getUID(), *pass, _currentTileNode);
                    }
                }
            }

            popModelViewMatrix();

            _terrain._drawState->_bs.expandBy(surface->getBound());
            _terrain._drawState->_box.expandBy(_terrain._drawState->_bs);
        }
    }

    traverse(node);
}
