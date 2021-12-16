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

#include <osgEarth/VisibleLayer>
#include <osgEarth/CameraUtils>

#define LC "[TerrainCuller] "

using namespace osgEarth::REX;


TerrainCuller::TerrainCuller() :
    _lastTimeVisited(DBL_MAX)
{
    setVisitorType(CULL_VISITOR);
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
}

void
TerrainCuller::reset(
    osgUtil::CullVisitor* parent_cullVisitor,
    TerrainRenderData::PersistentData& pd,
    EngineContext* context,
    LayerExtentMap& layerExtents)
{
    _cv = parent_cullVisitor;
    _context = context;
    _camera = _cv->getCurrentCamera();
    _currentTileNode = nullptr;
    _firstDrawCommandForTile = nullptr;
    _orphanedPassesDetected = 0u;
    _layerExtents = &layerExtents;
    bool temp;
    _isSpy = _cv->getUserValue("osgEarth.Spy", temp);
    _patchLayers.clear();
    _lastTimeVisited = osg::Timer::instance()->tick();

    // skip surface nodes is this is a shadow camera and shadowing is disabled.
    _acceptSurfaceNodes =
        CameraUtils::isShadowCamera(_cv->getCurrentCamera()) == false ||
        context->options().castShadows() == true;

    setCullingMode(_cv->getCullingMode());
    setFrameStamp(new osg::FrameStamp(*_cv->getFrameStamp()));
    setDatabaseRequestHandler(_cv->getDatabaseRequestHandler());
    pushReferenceViewPoint(_cv->getReferenceViewPoint());
    pushViewport(_cv->getViewport());
    pushProjectionMatrix(_cv->getProjectionMatrix());
    pushModelViewMatrix(_cv->getModelViewMatrix(), _cv->getCurrentCamera()->getReferenceFrame());
    setLODScale(_cv->getLODScale());
    setUserDataContainer(_cv->getUserDataContainer());

    unsigned frameNum = getFrameStamp() ? getFrameStamp()->getFrameNumber() : 0u;

    _terrain.reset(
        context->getMap().get(),
        context->getRenderBindings(),
        frameNum,
        pd,
        context->options().useGL4().get(),
        _cv,
        _context);
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
    if ( !surface )
        return 0L;

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
    LayerDrawable* drawable = _terrain.layer(uid);
    if (drawable)
    {
        // Layer marked for drawing?
        if (drawable->_draw)
        {
            // Cull based on the layer extent.
            if (drawable->_layer)
            {
                const LayerExtent& le = (*_layerExtents)[drawable->_layer->getUID()];
                if (le._extent.isValid() &&
                    ! le._extent.intersects(tileNode->getKey().getExtent(), false))
                {
                    // culled out!
                    //OE_DEBUG << LC << "Skippping " << drawable->_layer->getName() 
                    //    << " key " << tileNode->getKey().str()
                    //    << " because it was culled by extent." << std::endl;
                    return 0L;
                }            
            }

            DrawTileCommand tile;

            // install everything we need in the Draw Command:
            tile._colorSamplers = pass ? &(pass->samplers()) : 0L;
            tile._sharedSamplers = &model->_sharedSamplers;
            tile._modelViewMatrix = _cv->getModelViewMatrix();
            tile._keyValue = tileNode->getTileKeyValue();
            tile._geom = surface->getDrawable()->_geom.get();
            tile._tile = surface->getDrawable();
            tile._morphConstants = tileNode->getMorphConstants();
            tile._key = &tileNode->getKey();
            tile._tileRevision = tileNode->getRevision();

            osg::Vec3 c = surface->getBound().center() * surface->getInverseMatrix();
            tile._range = getDistanceToViewPoint(c, true);

            tile._layerOrder = drawable->_drawOrder;

#if 0
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
                tile._elevTexelCoeff.set((size - (2.0*bias)) / size, bias / size);
            }
#endif

            drawable->_tiles.emplace_back(std::move(tile));
            return &drawable->_tiles.back();
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
    
    return nullptr;
}

void
TerrainCuller::apply(osg::Node& node)
{
    TileNode* tileNode = dynamic_cast<TileNode*>(&node);
    if (tileNode)
    {
        apply(*tileNode);
    }
    else if (_acceptSurfaceNodes)
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
        
    if (!_terrain.patchLayers().empty() && 
        node.getSurfaceNode() != nullptr && 
        !node.isEmpty())
    {
        const RenderBindings& bindings = _context->getRenderBindings();
        TileRenderModel& renderModel = _currentTileNode->renderModel();

        // Render patch layers if applicable.
        _patchLayers.clear();

        for(auto& patchLayer : _terrain.patchLayers())
        {
            // is the layer accepting this key?
            if (patchLayer->getAcceptCallback() != nullptr &&
                !patchLayer->getAcceptCallback()->acceptKey(_currentTileNode->getKey()))
            {
                continue;
            }

            // is the tile in visible range?
            float range = _cv->getDistanceToViewPoint(node.getBound().center(), true) - node.getBound().radius();
            if (patchLayer->getMaxVisibleRange() < range)
            {
                continue;
            }

            _patchLayers.push_back(patchLayer.get());
        }

        if (!_patchLayers.empty())
        {
            SurfaceNode* surface = node.getSurfaceNode();
                    
            // push the surface matrix:
            osg::RefMatrix* matrix = createOrReuseMatrix(*_cv->getModelViewMatrix());
            surface->computeLocalToWorldMatrix(*matrix,this);
            _cv->pushModelViewMatrix(matrix, surface->getReferenceFrame());

            if (!_cv->isCulled(surface->getAlignedBoundingBox()))
            {
                // Add the draw command:
                for(auto patchLayer : _patchLayers)
                {
                    DrawTileCommand* cmd = addDrawCommand(patchLayer->getUID(), &renderModel, nullptr, &node);
                    if (cmd)
                    {
                        cmd->_drawPatch = true;
                        cmd->_drawCallback = patchLayer->getRenderer();
                        //cmd->_drawCallback = patchLayer->getDrawCallback();
                    }
                }
            }

           _cv->popModelViewMatrix();
        }
    }
}

void
TerrainCuller::apply(SurfaceNode& node)
{
    TileRenderModel& renderModel = _currentTileNode->renderModel();

    float range = _cv->getDistanceToViewPoint(node.getBound().center(), true) - node.getBound().radius();

    // push the surface matrix:
    osg::RefMatrix* matrix = createOrReuseMatrix(*getModelViewMatrix());
    node.computeLocalToWorldMatrix(*matrix,this);
    _cv->pushModelViewMatrix(matrix, node.getReferenceFrame());

    // now test against the local bounding box for tighter culling:
    if (!_cv->isCulled(node.getAlignedBoundingBox()))
    {
        if (!_isSpy)
        {
            node.setLastFramePassedCull(_context->getClock()->getFrame());
        }

        int order = 0;
        unsigned count = 0;

        // First go through any legit rendering pass data in the Tile and
        // and add a DrawCommand for each.
        for (unsigned p = 0; p < renderModel._passes.size(); ++p)
        {
            const RenderingPass& pass = renderModel._passes[p];

            // is the tile in visible range?
            if (pass.visibleLayer() && pass.visibleLayer()->getMaxVisibleRange() < range)
                continue;

            //TODO: see if we can skip adding a draw command for 1-pixel images
            // or other "placeholder" textures
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

