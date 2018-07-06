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


TerrainCuller::TerrainCuller(osgUtil::CullVisitor* cullVisitor, EngineContext* context) :
_camera(0L),
_currentTileNode(0L),
_orphanedPassesDetected(0u),
_cv(cullVisitor),
_context(context)
{
    setVisitorType(CULL_VISITOR);
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
    setCullingMode(cullVisitor->getCullingMode());

    setFrameStamp(new osg::FrameStamp(*_cv->getFrameStamp()));
    setDatabaseRequestHandler(_cv->getDatabaseRequestHandler());
    pushReferenceViewPoint(_cv->getReferenceViewPoint());
    pushViewport(_cv->getViewport());
    pushProjectionMatrix(_cv->getProjectionMatrix());
    pushModelViewMatrix(_cv->getModelViewMatrix(), _cv->getCurrentCamera()->getReferenceFrame());
    setLODScale(_cv->getLODScale());
    _camera = _cv->getCurrentCamera();
}

void
TerrainCuller::setup(const Map* map, LayerExtentVector& layerExtents, const RenderBindings& bindings)
{
    unsigned frameNum = getFrameStamp() ? getFrameStamp()->getFrameNumber() : 0u;
    _layerExtents = &layerExtents;
    _terrain.setup(map, bindings, frameNum, _cv);
}

float
TerrainCuller::getDistanceToViewPoint(const osg::Vec3& pos, bool withLODScale) const
{
    return _cv->getDistanceToViewPoint(pos, withLODScale);
    //if (withLODScale) return (pos-getViewPointLocal()).length()*getLODScale();
    //else return (pos-getViewPointLocal()).length();
}

DrawTileCommand*
TerrainCuller::addDrawCommand(UID uid, const TileRenderModel* model, const RenderingPass* pass, TileNode* tileNode)
{
    SurfaceNode* surface = tileNode->getSurfaceNode();

    const RenderBindings& bindings = _context->getRenderBindings();

    // skip layers that are not visible:
    if (pass && 
        pass->visibleLayer() && 
        pass->visibleLayer()->getVisible() == false)
    {
        //OE_DEBUG << LC << "Skipping " << pass->visibleLayer()->getName() << " because it's not visible." << std::endl;
        return 0L;
    }

    // add a new Draw command to the appropriate layer
    osg::ref_ptr<LayerDrawable> drawable = _terrain.layer(uid);
    if (drawable.valid())
    {
        // Layer marked for drawing?
        if (drawable->_draw)
        {
            // Cull based on the layer extent.
            if (drawable->_layer)
            {
                const LayerExtent& le = (*_layerExtents)[drawable->_layer->getUID()];
                if (le._computed && 
                    le._extent.isValid() &&
                    le._extent.intersects(tileNode->getKey().getExtent()) == false)
                {
                    // culled out!
                    //OE_DEBUG << LC << "Skippping " << drawable->_layer->getName() 
                    //    << " key " << tileNode->getKey().str()
                    //    << " because it was culled by extent." << std::endl;
                    return 0L;
                }
            }

            drawable->_tiles.push_back(DrawTileCommand());
            DrawTileCommand* tile = &drawable->_tiles.back();

            // install everything we need in the Draw Command:
            tile->_colorSamplers = pass ? &(pass->samplers()) : 0L;
            tile->_sharedSamplers = &model->_sharedSamplers;
            tile->_modelViewMatrix = this->getModelViewMatrix();
            tile->_keyValue = tileNode->getTileKeyValue();
            tile->_geom = surface->getDrawable()->_geom.get();
            tile->_morphConstants = tileNode->getMorphConstants();
            tile->_key = &tileNode->getKey();
            //tile->_order = (int)orderInTile;
            tile->_order = drawable->_order; // layer order in map tile.

            osg::Vec3 c = surface->getBound().center() * surface->getInverseMatrix();
            tile->_range = getDistanceToViewPoint(c, true);

            const osg::Image* elevRaster = tileNode->getElevationRaster();
            if (elevRaster)
            {
                float bias = _context->getUseTextureBorder() ? 1.5 : 0.5;

                // Compute an elevation texture sampling scale/bias so we sample elevation data on center
                // instead of on edge (as we do with color, etc.)
                //
                // This starts out as:
                //   scale = (size-1)/size : this shrinks the sample area by one texel since we're sampling on center
                //   bias = 0.5/size : this shifts the sample area over 1/2 texel to the center.
                //
                // But, since we also have a 1-texel border, we need to further reduce the scale by 2 texels to
                // remove the border, and shift an extra texel over as well. Giving us this:
                float size = (float)elevRaster->s();
                tile->_elevTexelCoeff.set((size - (2.0*bias)) / size, bias / size);
            }

            return tile;
        }
    }
    else if (pass)
    {
        // The pass exists but it's layer is not in the render data draw list.
        // This means that the layer is no longer in the map. Detect and record
        // this information so we can run a cleanup visitor later on.
        ++_orphanedPassesDetected;
    }
    
    return 0L;
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

        // reset the pointer to the first DrawTileCommand. We keep track of this so
        // we can set it's "order" member to zero at the end, so the rendering engine
        // knows to blend it with the terrain geometry color.
        _firstTileDrawCommandForTile = 0L;

        //_currentTileDrawCommands = 0u;
        
        if (!_terrain.patchLayers().empty())
        {
            // todo: check for patch/virtual
            const RenderBindings& bindings = _context->getRenderBindings();
            TileRenderModel& renderModel = _currentTileNode->renderModel();

            bool pushedMatrix = false;
            
            for (PatchLayerVector::const_iterator i = _terrain.patchLayers().begin(); i != _terrain.patchLayers().end(); ++i)
            {
                PatchLayer* layer = i->get();
                if (layer->getAcceptCallback() == 0L ||
                    layer->getAcceptCallback()->acceptKey(_currentTileNode->getKey()))
                {
                    // Push this tile's matrix if we haven't already done so:
                    if (!pushedMatrix)
                    {
                        SurfaceNode* surface = tileNode->getSurfaceNode();

                        // push the surface matrix:
                        osg::Matrix mvm = *getModelViewMatrix();
                        surface->computeLocalToWorldMatrix(mvm, this);
                        pushModelViewMatrix(createOrReuseMatrix(mvm), surface->getReferenceFrame());
                        pushedMatrix = true;
                    }

                    // Add the draw command:
                    DrawTileCommand* cmd = addDrawCommand(layer->getUID(), &renderModel, 0L, tileNode);
                    if (cmd)
                    {
                        cmd->_drawPatch = true;
                        cmd->_drawCallback = layer->getDrawCallback();
                    }
                }
            }

            if (pushedMatrix)
            {
                popModelViewMatrix();
            }
        }
    }

    else
    {
        SurfaceNode* surface = dynamic_cast<SurfaceNode*>(&node);
        if (surface)
        {
            TileRenderModel& renderModel = _currentTileNode->renderModel();

            // push the surface matrix:
            osg::Matrix mvm = *getModelViewMatrix();
            surface->computeLocalToWorldMatrix(mvm, this);
            pushModelViewMatrix(createOrReuseMatrix(mvm), surface->getReferenceFrame());

            int order = 0;

            // First go through any legit rendering pass data in the Tile and
            // and add a DrawCommand for each.
            for (unsigned p = 0; p < renderModel._passes.size(); ++p)
            {
                const RenderingPass& pass = renderModel._passes[p];
                DrawTileCommand* cmd = addDrawCommand(pass.sourceUID(), &renderModel, &pass, _currentTileNode);
                if (cmd)
                {
                    if (_firstTileDrawCommandForTile == 0L)
                    {
                        _firstTileDrawCommandForTile = cmd;
                    }
                    else if (cmd->_order < _firstTileDrawCommandForTile->_order)
                    {
                        //_firstTileDrawCommandForTile->_order = 1;
                        _firstTileDrawCommandForTile = cmd;
                    }
                }
            }

            // If the culler added no draw commands for this tile... we still need
            // to draw something or else there will be a hole! So draw a blank tile.
            // UID = -1 is the special UID code for a blank.
            if (_firstTileDrawCommandForTile == 0L)
            {
                //OE_INFO << LC << "Adding blank render for tile " << _currentTileNode->getKey().str() << std::endl;
                DrawTileCommand* cmd = addDrawCommand(-1, &renderModel, 0L, _currentTileNode);
                if (cmd)
                    cmd->_order = 0;
            }

            // Set the layer order of the first draw command for this tile to zero,
            // to support proper terrain blending.
            if (_firstTileDrawCommandForTile)
            {
                _firstTileDrawCommandForTile->_order = 0;
            }
                
            // pop the matrix from the cull stack
            popModelViewMatrix();

            // update our bounds
            _terrain._drawState->_bs.expandBy(surface->getBound());
            _terrain._drawState->_box.expandBy(_terrain._drawState->_bs);
        }
    }

    // Handle any CullCallbacks and traverse.
    osg::Callback* cullCallback = node.getCullCallback();
    if (cullCallback) cullCallback->run(&node, this);
    else traverse(node);
}
