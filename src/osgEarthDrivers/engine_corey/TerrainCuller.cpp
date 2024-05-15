
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
#include "SelectionInfo"
#include "EngineData"

#include <osgEarth/VisibleLayer>
#include <osgEarth/CameraUtils>
#include <osgUtil/CullVisitor>

#define LC "[TerrainCuller] "

using namespace osgEarth::Corey;


namespace
{
    // hack to get access to createOrReuseMatrix
    struct CullVisitorEx : public osgUtil::CullVisitor {
        inline osg::RefMatrix* createOrReuseMatrixEx(const osg::Matrix& value) {
            return createOrReuseMatrix(value);
        }
    };
}


void
CullData::reset(osgUtil::CullVisitor* cv, TerrainRenderData::PersistentData& pd, EngineData& data)
{
    auto& map = data.map;
    _mapRevision = map->getDataModelRevision();
    _data = &data;
    _orphanedPassesDetected = 0u;
    _layerExtents = &data.cachedLayerExtents;
    //bool temp;
    //_isSpy = _cv->getUserValue("osgEarth.Spy", temp);
    _patchLayers.clear();
    _lastTimeVisited = osg::Timer::instance()->tick();
    _staleTiles.clear();

    // skip surface nodes is this is a shadow camera and shadowing is disabled.
    _acceptSurfaceNodes =
        CameraUtils::isShadowCamera(cv->getCurrentCamera()) == false ||
        data.options.getCastShadows() == true;

    unsigned frameNum = cv->getFrameStamp() ? cv->getFrameStamp()->getFrameNumber() : 0u;

    _renderData.reset(frameNum, pd, cv, data);
}

DrawTileCommand*
CullData::addDrawCommand(UID uid, const TileRenderModel* model, const RenderingPass* pass, TileNode* tilenode, osgUtil::CullVisitor* cv)
{
    if (tilenode->getBound().valid() == false)
        return nullptr;

    const RenderBindings& bindings = _data->renderBindings;

    // add a new Draw command to the appropriate layer
    LayerDrawable* drawable = _renderData.layer(uid);
    if (drawable)
    {
        // Layer marked for drawing?
        if (drawable->_draw)
        {
            drawable->_tiles.emplace_back();
            DrawTileCommand& tile = drawable->_tiles.back();
            tile._intersectsLayerExtent = true;

            // If the tile is outside the layer extent, we MAY or MAY NOT need to
            // actually render it. Mark it as such.
            if (drawable->_layer)
            {
                const LayerExtent& le = (*_layerExtents)[drawable->_layer->getUID()];
                if (le._extent.isValid() && !le._extent.intersects(tilenode->getKey().getExtent(), false))
                {
                    tile._intersectsLayerExtent = false;
                }
            }

            // install everything we need in the Draw Command:
            tile._colorSamplers = pass ? &(pass->samplers()) : nullptr;
            tile._commonSamplers = &model->_commonSamplers;
            tile._localToWorld = tilenode->_xform->getMatrix();
            tile._modelViewMatrix = *cv->getModelViewMatrix();
            tile._keyValue = tilenode->_tileKeyValue;
            tile._geom = tilenode->_drawable;
            tile._morphConstants = tilenode->_morphConstants;
            tile._key = &tilenode->getKey();
            tile._tileRevision = tilenode->_fullDataModel->mapRevision;

            // note: _xform->getInverseMatrix() is cached internally
            osg::Vec3 center_local = tilenode->_xform->getBound().center() * tilenode->_xform->getInverseMatrix();
            tile._range = cv->getDistanceToViewPoint(center_local, true);

            tile._layerOrder = drawable->_drawOrder;

            // assign the draw sequence:
            tile._sequence = drawable->_tiles.size();

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
        OE_WARN << "Added nothing for a UID -1 draw command" << std::endl;
    }

    return nullptr;
}


bool
TileNodeCuller::run(osg::Object* object, osg::Object* visitor)
{
    auto* tilenode = static_cast<TileNode*>(object);
    auto* cv = dynamic_cast<osgUtil::CullVisitor*>(visitor);
    auto& cullData = _data->cullData;

    if (cv->isCulled(tilenode->getBound()))
        return false;

    // handle patch layers:
    if (cullData._renderData.patchLayers().size() > 0)
    {
        const RenderBindings& bindings = cullData._data->renderBindings;
        TileRenderModel& renderModel = tilenode->_renderModel;

        // Render patch layers if applicable.
        cullData._patchLayers.clear();

        osg::BoundingBox buffer(0, 0, 0, 0, 0, 0);

        for (auto& patchLayer : cullData._renderData.patchLayers())
        {
            // is the layer accepting this key?
            if (patchLayer->getAcceptCallback() != nullptr &&
                !patchLayer->getAcceptCallback()->acceptKey(tilenode->getKey()))
            {
                continue;
            }

            // is the tile in visible range?
            float range = cv->getDistanceToViewPoint(tilenode->getBound().center(), true) - tilenode->getBound().radius();
            if (patchLayer->getMaxVisibleRange() < range)
            {
                continue;
            }

            buffer.expandBy(patchLayer->getBuffer());

            cullData._patchLayers.push_back(patchLayer.get());
        }

        if (cullData._patchLayers.size() > 0)
        {
            // push the surface matrix:
            auto cs = static_cast<CullVisitorEx*>(cv);
            osg::ref_ptr<osg::RefMatrix> matrix = cs->createOrReuseMatrixEx(*cv->getModelViewMatrix());
            tilenode->_xform->computeLocalToWorldMatrix(*matrix.get(), cv);
            cv->pushModelViewMatrix(matrix.get(), tilenode->_xform->getReferenceFrame());

            // adjust the tile bounding box to account for the patch layer buffer.
            auto bbox = tilenode->_drawable->getBoundingBox();
            bbox._min += buffer._min, bbox._max += buffer._max;

            if (!cv->isCulled(bbox))
            {
                float range, morphStart, morphEnd;
                _data->selectionInfo.get(tilenode->getKey(), range, morphStart, morphEnd);

                // Add the draw commands:
                for (auto patchLayer : cullData._patchLayers)
                {
                    DrawTileCommand* cmd = cullData.addDrawCommand(patchLayer->getUID(), &renderModel, nullptr, tilenode, cv);
                    if (cmd)
                    {
                        cmd->_drawPatch = true;
                        cmd->_drawCallback = patchLayer->getRenderer();
                        cmd->_morphStartRange = morphStart;
                        cmd->_morphEndRange = morphEnd;
                    }
                }
            }

            cv->popModelViewMatrix();
        }
    }

    // check for a stale data model and request an update if necessary.
    if (tilenode->_dirty)
        //tilenode->_fullDataModel->revision != cullData._mapRevision)
    {
        // submit an update request.
        cullData._staleTiles.emplace_back(tilenode->_key);
        tilenode->_dirty = false;
    }

    // store this tilenode and traverse the children.
    cullData._tileNodeStack.push(tilenode);
    bool ok = traverse(object, visitor);
    cullData._tileNodeStack.pop();

    return ok;
}


bool
TileDrawableCuller::cull(osg::NodeVisitor* visitor, osg::Drawable* node, osg::RenderInfo* renderInfo) const
{
    auto& cullData = _data->cullData;
    
    // skip surface nodes if this is a shadow camera and self-shadowing is disabled;
    // this will only render patch layers in the shader camera (see TileNode)
    if (!cullData._acceptSurfaceNodes)
        return true;

    auto* cv = dynamic_cast<osgUtil::CullVisitor*>(visitor);
    auto& drawable = *static_cast<TileGeometry*>(node);

    auto& bbox = drawable.getBoundingBox();
    if (drawable.isCullingActive() && cv->isCulled(bbox))
        return true;

    float center_range = cv->getDistanceToViewPoint(drawable.getBound().center(), true);
    float node_radius = drawable.getBound().radius();
    float near_range = center_range - node_radius;
    float far_range = center_range + node_radius;

#if 0
    if (!_isSpy)
    {
        node.setLastFramePassedCull(_context->getClock()->getFrame());
    }
#endif

    // find the drawable's parent tilenode:
    TileNode& tilenode = *cullData._tileNodeStack.top();

    TileRenderModel& renderModel = tilenode._renderModel;

    int order = 0;
    unsigned count = 0;
    DrawTileCommand* first_cmd = nullptr;
    auto lod = tilenode.getKey().getLOD();

    // First go through any legit rendering pass data in the Tile and
    // and add a DrawCommand for each.
    for (unsigned p = 0; p < renderModel._passes.size(); ++p)
    {
        const RenderingPass& pass = renderModel._passes[p];

        if (pass.tileLayer())
        {
            if (pass.tileLayer()->getVisible() == false ||
                pass.tileLayer()->getMaxLevel() < lod ||
                pass.tileLayer()->getMinLevel() > lod ||
                pass.tileLayer()->getMaxVisibleRange() < near_range ||
                pass.tileLayer()->getMinVisibleRange() > far_range)
            {
                continue;
            }
        }
        else if (pass.visibleLayer())
        {
            if (pass.visibleLayer()->getVisible() == false ||
                pass.visibleLayer()->getMaxVisibleRange() < near_range ||
                pass.visibleLayer()->getMinVisibleRange() > far_range)
            {
                continue;
            }
        }

        DrawTileCommand* cmd = cullData.addDrawCommand(pass.sourceUID(), &renderModel, &pass, &tilenode, cv);

        if (cmd)
        {
            if (first_cmd == nullptr)
            {
                first_cmd = cmd;
            }
            else if (cmd->_layerOrder < first_cmd->_layerOrder)
            {
                first_cmd = cmd;
            }
        }
    }

    // If the culler added no draw commands for this tile... we still need
    // to draw something or else there will be a hole! So draw a blank tile.
    // UID = -1 is the special UID code for a blank.
    if (first_cmd == nullptr)
    {
        DrawTileCommand* cmd = cullData.addDrawCommand(-1, &renderModel, nullptr, &tilenode, cv);
        if (cmd)
        {
            first_cmd = cmd;
        }
    }

    // Set the layer order of the first draw command for this tile to zero,
    // to support proper terrain blending.
    if (first_cmd)
    {
        first_cmd->_layerOrder = 0;
    }

    // update our bounds (using the tilenode's world-space bounds)
    cullData._renderData._drawState->_bs.expandBy(tilenode.getBound());
    cullData._renderData._drawState->_box.expandBy(cullData._renderData._drawState->_bs);

    return true;
}
