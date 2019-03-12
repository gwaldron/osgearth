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
#include "TerrainCuller"
#include "TileNode"
#include "SurfaceNode"
#include "SelectionInfo"
#include <osgEarth/TraversalData>

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
    _isSpy = VisitorData::isSet(*cullVisitor, "osgEarth.Spy");
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
    // pass through, in case developer has overridden the method in the prototype CV
    return _cv->getDistanceToViewPoint(pos, withLODScale);
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
            tile->_modelViewMatrix = _cv->getModelViewMatrix();
            tile->_keyValue = tileNode->getTileKeyValue();
            tile->_geom = surface->getDrawable()->_geom.get();
            tile->_morphConstants = tileNode->getMorphConstants();
            tile->_key = &tileNode->getKey();

            osg::Vec3 c = surface->getBound().center() * surface->getInverseMatrix();
            tile->_range = getDistanceToViewPoint(c, true);

            tile->_layerOrder = drawable->_drawOrder;

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
    else
    {
        OE_WARN << "Added nothing for a UID -1 darw command" << std::endl;
    }
    
    return 0L;
}

void
TerrainCuller::apply(osg::Node& node)
{
    TileNode* tileNode = dynamic_cast<TileNode*>(&node);
    if (tileNode)
    {
        apply(*tileNode);
    }
    else
    {
        SurfaceNode* surfaceNode = dynamic_cast<SurfaceNode*>(&node);
        if (surfaceNode)
        {
            apply(*surfaceNode);
            return; // no need to traverse further
        }
    }

    // Handle any CullCallbacks and traverse.
    osg::Callback* cullCallback = node.getCullCallback();
    if (cullCallback) cullCallback->run(&node, this);
    else traverse(node);
}

bool
TerrainCuller::isCulledToBBox(osg::Transform* node, const osg::BoundingBox& box)
{
    osg::RefMatrix* matrix = createOrReuseMatrix(*_cv->getModelViewMatrix());
    node->computeLocalToWorldMatrix(*matrix, this);
    _cv->pushModelViewMatrix(matrix, node->getReferenceFrame());
    bool culled = _cv->isCulled(box);
    _cv->popModelViewMatrix();
    return culled;
}

void
TerrainCuller::apply(TileNode& node)
{
    _currentTileNode = &node;

    // reset the pointer to the first DrawTileCommand. We keep track of this so
    // we can set it's "layerOrder" member to zero at the end, so the rendering engine
    // knows to blend it with the terrain geometry color.
    _firstDrawCommandForTile = 0L;
        
    if (!_terrain.patchLayers().empty())
    {
        // todo: check for patch/virtual
        const RenderBindings& bindings = _context->getRenderBindings();
        TileRenderModel& renderModel = _currentTileNode->renderModel();

        bool pushedMatrix = false;

        // Render patch layers if applicable.
        // A patch layer is one rendered using GL_PATCHES.
        for (PatchLayerVector::const_iterator i = _terrain.patchLayers().begin(); i != _terrain.patchLayers().end(); ++i)
        {
            PatchLayer* layer = i->get();

            // is the layer accepting this key?
            if (layer->getAcceptCallback() && !layer->getAcceptCallback()->acceptKey(_currentTileNode->getKey()))
                continue;

            // is the tile in visible range?
            float range = _cv->getDistanceToViewPoint(node.getBound().center(), true) - node.getBound().radius();
            if (layer->getMaxVisibleRange() < range)
                continue;

            // Push this tile's matrix if we haven't already done so:
            if (!pushedMatrix)
            {
                SurfaceNode* surface = node.getSurfaceNode();
                    
                // push the surface matrix:
                osg::RefMatrix* matrix = createOrReuseMatrix(*_cv->getModelViewMatrix());
                surface->computeLocalToWorldMatrix(*matrix,this);
                _cv->pushModelViewMatrix(matrix, surface->getReferenceFrame());

                pushedMatrix = true;
            }

            // Add the draw command:
            DrawTileCommand* cmd = addDrawCommand(layer->getUID(), &renderModel, 0L, &node);
            if (cmd)
            {
                cmd->_drawPatch = true;
                cmd->_drawCallback = layer->getDrawCallback();
            }
        }

        if (pushedMatrix)
        {
           _cv->popModelViewMatrix();
        }
    }
}

void
TerrainCuller::apply(SurfaceNode& node)
{
    TileRenderModel& renderModel = _currentTileNode->renderModel();

    // push the surface matrix:
    osg::RefMatrix* matrix = createOrReuseMatrix(*getModelViewMatrix());
    node.computeLocalToWorldMatrix(*matrix,this);
    _cv->pushModelViewMatrix(matrix, node.getReferenceFrame());

    // now test against the local bounding box for tighter culling:
    if (!_cv->isCulled(node.getAlignedBoundingBox()))
    {
        if (!_isSpy)
        {
            node.setLastFramePassedCull(getFrameStamp()->getFrameNumber());
        }

        int order = 0;
        unsigned count = 0;

        // First go through any legit rendering pass data in the Tile and
        // and add a DrawCommand for each.
        for (unsigned p = 0; p < renderModel._passes.size(); ++p)
        {
            const RenderingPass& pass = renderModel._passes[p];
            DrawTileCommand* cmd = addDrawCommand(pass.sourceUID(), &renderModel, &pass, _currentTileNode);
            if (cmd)
            {
                if (_firstDrawCommandForTile == 0L)
                {
                    _firstDrawCommandForTile = cmd;
                }
                else if (cmd->_layerOrder < _firstDrawCommandForTile->_layerOrder)
                {
                    _firstDrawCommandForTile = cmd;
                }
            }
        }

        // If the culler added no draw commands for this tile... we still need
        // to draw something or else there will be a hole! So draw a blank tile.
        // UID = -1 is the special UID code for a blank.
        if (_firstDrawCommandForTile == 0L)
        {
            //OE_INFO << LC << "Adding blank render for tile " << _currentTileNode->getKey().str() << std::endl;
            DrawTileCommand* cmd = addDrawCommand(-1, &renderModel, 0L, _currentTileNode);
            if (cmd)
            {
                _firstDrawCommandForTile = cmd;
            }
        }

        // Set the layer order of the first draw command for this tile to zero,
        // to support proper terrain blending.
        if (_firstDrawCommandForTile)
        {
            _firstDrawCommandForTile->_layerOrder = 0;
        }

        // update our bounds
        _terrain._drawState->_bs.expandBy(node.getBound());
        _terrain._drawState->_box.expandBy(_terrain._drawState->_bs);
    }
                
    // pop the matrix from the cull stack
    _cv->popModelViewMatrix();

    if (node.getDebugNode())
    {
        node.accept(*_cv);
    }
}

