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
#include "TileNode"
#include "EngineData"
#include "SelectionInfo"
#include "TerrainCuller"

using namespace osgEarth::Corey;
using namespace osgEarth;

#define LC "[TileNode] "

namespace
{
    // Scale and bias matrices, one for each TileKey quadrant.
    const osg::Matrixf scaleBias[4] =
    {
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.0f,0.5f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.5f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.0f,0.0f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.0f,0,1.0f)
    }; 

    TerrainTileModel* combine(TerrainTileModel* child, const TerrainTileModel* parent)
    {
        OE_SOFT_ASSERT_AND_RETURN(child, nullptr);

        if (!parent)
            return child;

        auto lod = child->key.getLOD();
        int quadrant = child->key.getQuadrant();

        TerrainTileModel::ColorLayer::Vector colorLayersToInherit;

        // Look for parent color layers that don't exist in the child.
        // For each one, copy its data model and apply the scale/bias matrix.
        unsigned original_num_color_layers = child->colorLayers.size();
        for (auto& parent_layer : parent->colorLayers)
        {
            if (parent_layer.tileLayer.valid() && parent_layer.tileLayer->getMaxLevel() >= lod)
            {
                bool found = false;
                for(unsigned i=0; i< original_num_color_layers; ++i)
                {
                    auto& child_layer = child->colorLayers[i];
                    if (child_layer.layer.valid() && child_layer.layer->getUID() == parent_layer.layer->getUID())
                    {
                        // found a match; carry on.
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    child->colorLayers.emplace_back(parent_layer);
                    auto& new_color_layer = child->colorLayers.back();
                    new_color_layer.matrix.preMult(scaleBias[quadrant]);
                }
            }
        }

        if (child->normalMap.texture == nullptr && parent->normalMap.texture != nullptr)
        {
            child->normalMap = parent->normalMap;
            child->normalMap.matrix.preMult(scaleBias[quadrant]);
        }

        if (child->elevation.texture == nullptr && parent->elevation.texture != nullptr)
        {
            child->elevation = parent->elevation;
            child->elevation.matrix.preMult(scaleBias[quadrant]);
        }

        // todo, landcover IF we need it. Hopefully never.

        return child;
    }
}


TileNode::TileNode(const TileKey& key) :
    _key(key)
{
    //nop
}

void
TileNode::set(TerrainTileModel* new_model, const TerrainTileModel* parent_model, EngineData& data)
{
    OE_SOFT_ASSERT_AND_RETURN(new_model, void()); // require the data model and context
    OE_SOFT_ASSERT_AND_RETURN(new_model->mesh.verts.valid(), void()); // require valid mesh

    bool newMesh =
        !_partialDataModel.valid() ||
        _partialDataModel->mesh.revision != new_model->mesh.revision;

    _partialDataModel = new_model;

    // Fills in any missing data in the new data model with data inherited
    // from the parent data model, if available.
    _fullDataModel = combine(new_model, parent_model);

    // todo: populate the full model based on parent data --
    // -- maybe do this in the caller and pass them both in --
    // -- then TileNode doesn't need to know about its parent

    auto& mesh = _fullDataModel->mesh;

    if (!_xform)
    {
        _xform = new osg::MatrixTransform();
        _xform->addChild(_drawable);
        addChild(_xform);
        this->setCullCallback(data.tileNodeCullCallback.get());

        // Encode the tile key in a uniform. Note! The X and Y components are presented
        // modulo 2^16 form so they don't overrun single-precision space.
        unsigned tw, th;
        _key.getProfile()->getNumTiles(_key.getLOD(), tw, th);
        double x = (double)_key.getTileX();
        double y = (double)(th - _key.getTileY() - 1);
        _tileKeyValue.set((float)(x - tw / 2), (float)(y - th / 2), (float)_key.getLOD(), -1.0f);

        // initialize all the per-tile uniforms the shaders will need:
        float range, morphStart, morphEnd;
        data.selectionInfo.get(_key, range, morphStart, morphEnd);

        float one_over_end_minus_start = 1.0f / (morphEnd - morphStart);
        _morphConstants.set(morphEnd * one_over_end_minus_start, one_over_end_minus_start);
    }

    if (newMesh)
    {
        _drawable = TileGeometry::create(mesh);
        _drawable->setCullCallback(data.tileDrawableCullCallback.get());
        _xform->removeChildren(0, 1);
        _xform->addChild(_drawable);
        _xform->setMatrix(mesh.localToWorld);
    }

    // update the samplers based on the new data model.
    updateSamplers(nullptr, data);
}

void
TileNode::updateSamplers(const TileRenderModel* parentRenderModel, EngineData& data)
{
    bool newElevationData = false;
    const RenderBindings& bindings = data.renderBindings;
    RenderingPasses& myPasses = _renderModel._passes;
    vector_set<UID> uidsLoaded;

#if 0
    // if terrain constraints are in play, regenerate the tile's geometry.
    // this could be kinda slow, but meh, if you are adding and removing
    // constraints, frame drops are not a big concern
    if (manifest.includesConstraints())
    {
        // todo: progress callback here? I believe progress gets
        // checked before merge() anyway.
        createGeometry(nullptr);
    }
#endif

    auto* model = _fullDataModel.get();

    // First deal with the rendering passes (for color data):
    const SamplerBinding& color = bindings[SamplerBinding::COLOR];
    if (color.isActive())
    {
        // loop over all the layers included in the new data model and
        // add them to our render model (or update them if they already exist)
        for (const auto& colorLayer : model->colorLayers)
        {
            auto& layer = colorLayer.layer;
            if (!layer.valid())
                continue;

            // Look up the parent pass in case we need it
            RenderingPass* pass = _renderModel.getPass(layer->getUID());

            const RenderingPass* parentPass = parentRenderModel ? parentRenderModel->getPass(layer->getUID()) : nullptr;

            // ImageLayer?
            ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer.get());
            if (imageLayer)
            {
                bool isNewPass = (pass == nullptr);
                if (isNewPass)
                {
                    // Pass didn't exist here, so add it now.
                    pass = &_renderModel.addPass();
                    pass->setLayer(layer);
                }

                pass->setSampler(SamplerBinding::COLOR, colorLayer.texture, colorLayer.matrix, colorLayer.revision);

                // If this is a new rendering pass, just copy the color into the color-parent.
                if (isNewPass && bindings[SamplerBinding::COLOR_PARENT].isActive())
                {
                    pass->sampler(SamplerBinding::COLOR_PARENT) = pass->sampler(SamplerBinding::COLOR);
                }

#if 0
                // check to see if this data requires an image update traversal.
                if (_imageUpdatesActive == false)
                {
                    _imageUpdatesActive = colorLayer.texture->needsUpdates();
                }
#endif

#if 0
                if (imageLayer->getAsyncLoading())
                {
                    if (parentPass)
                    {
                        pass->inheritFrom(*parentPass, scaleBias[_key.getQuadrant()]);

                        if (bindings[SamplerBinding::COLOR_PARENT].isActive())
                        {
                            Sampler& colorParent = pass->sampler(SamplerBinding::COLOR_PARENT);
                            colorParent._texture = parentPass->sampler(SamplerBinding::COLOR)._texture;
                            colorParent._matrix = parentPass->sampler(SamplerBinding::COLOR)._matrix;
                            colorParent._matrix.preMult(scaleBias[_key.getQuadrant()]);
                        }
                    }
                    else
                    {
                        // note: this can happen with an async layer load
                        OE_DEBUG << "no parent pass in my pass. key=" << model->key.str() << std::endl;
                    }

                    // check whether it's actually a futuretexture.
                    // if it's not, it is likely an empty texture and we'll ignore it
                    if (dynamic_cast<FutureTexture*>(colorLayer.texture->osgTexture().get()))
                    {
                        pass->sampler(SamplerBinding::COLOR)._futureTexture = colorLayer.texture;
                    }

                    // require an update pass to process the future texture
                    _imageUpdatesActive = true;
                }
#endif

                uidsLoaded.insert(pass->sourceUID());
            }

            else // non-image color layer (like splatting, e.g.)
            {
                if (!pass)
                {
                    pass = &_renderModel.addPass();
                    pass->setLayer(colorLayer.layer.get());
                }

                uidsLoaded.insert(pass->sourceUID());
            }
        }

#if 0
        // Next loop over all the passes that we OWN, we asked for, but we didn't get.
        // That means they no longer exist at this LOD, and we need to convert them
        // into inherited samplers (or delete them entirely)
        for (int p = 0; p < myPasses.size(); ++p)
        {
            RenderingPass& myPass = myPasses[p];

            if (myPass.ownsTexture() &&
                manifest.includes(myPass.layer()) &&
                !uidsLoaded.contains(myPass.sourceUID()))
            {
                OE_DEBUG << LC << "Releasing orphaned layer " << myPass.layer()->getName() << std::endl;

                bool deletePass = true;

                if (parentRenderModel)
                {
                    const RenderingPass* parentPass = parentRenderModel->getPass(myPass.sourceUID());
                    if (parentPass)
                    {
                        myPass.inheritFrom(*parentPass, scaleBias[_key.getQuadrant()]);
                        deletePass = false;
                    }
                }

                if (deletePass)
                {
                    myPasses.erase(myPasses.begin() + p);
                    --p;
                }
            }
        }
#endif
    }

    // Elevation data:
    const SamplerBinding& elevation = bindings[SamplerBinding::ELEVATION];
    if (elevation.isActive() && model->elevation.texture)
    {
        _renderModel.setCommonSampler(SamplerBinding::ELEVATION,
            model->elevation.texture, model->elevation.matrix, model->elevation.revision);

        //updateElevationRaster();
        newElevationData = true;
    }
    else
    {
        _renderModel.clearCommonSampler(SamplerBinding::ELEVATION);
    }

#if 0
        else if (
            manifest.includesElevation() &&
            _renderModel._CommonSamplers[SamplerBinding::ELEVATION].ownsTexture())
        {
            // We OWN elevation data, requested new data, and didn't get any.
            // That means it disappeared and we need to delete what we have.
            inheritCommonSampler(SamplerBinding::ELEVATION);

            updateElevationRaster();

            newElevationData = true;
        }
#endif
    

    // Normals:
    const SamplerBinding& normals = bindings[SamplerBinding::NORMAL];
    if (normals.isActive() && model->normalMap.texture)
    {
        _renderModel.setCommonSampler(SamplerBinding::NORMAL,
            model->normalMap.texture, model->normalMap.matrix, model->normalMap.revision);
        //updateNormalMap();
    }
    else
    {
        _renderModel.clearCommonSampler(SamplerBinding::NORMAL);
    }

#if 0
        // If we OWN normal data, requested new data, and didn't get any,
        // that means it disappeared and we need to delete what we have:
        else if (
            manifest.includesElevation() && // not a typo, check for elevation
            _renderModel._CommonSamplers[SamplerBinding::NORMAL].ownsTexture())
        {
            inheritCommonSampler(SamplerBinding::NORMAL);
            updateNormalMap();
        }
#endif
    

    // Land Cover:
    const SamplerBinding& landCover = bindings[SamplerBinding::LANDCOVER];
    if (landCover.isActive() && model->landCover.texture)
    {
        _renderModel.setCommonSampler(SamplerBinding::LANDCOVER,
            model->landCover.texture, model->landCover.matrix, model->landCover.revision);
    }
    else
    {
        _renderModel.clearCommonSampler(SamplerBinding::LANDCOVER);
    }

#if 0
        else if (
            manifest.includesLandCover() &&
            _renderModel._CommonSamplers[SamplerBinding::LANDCOVER].ownsTexture())
        {
            // We OWN landcover data, requested new data, and didn't get any.
            // That means it disappeared and we need to delete what we have.
            inheritCommonSampler(SamplerBinding::LANDCOVER);
        }
#endif
    


    // Other Shared Layers:
    uidsLoaded.clear();

    for (unsigned index : model->sharedLayerIndices)
    {
        auto& sharedLayer = model->colorLayers[index];
        // locate the shared binding corresponding to this layer:
        UID uid = sharedLayer.layer->getUID();
        unsigned bindingIndex = INT_MAX;
        for (unsigned i = SamplerBinding::SHARED; i < bindings.size() && bindingIndex == INT_MAX; ++i)
        {
            if (bindings[i].isActive() && bindings[i].sourceUID.isSetTo(uid))
            {
                bindingIndex = i;
            }
        }

        if (sharedLayer.texture && bindingIndex < INT_MAX)
        {
            _renderModel.setCommonSampler(bindingIndex, sharedLayer.texture, sharedLayer.revision);
            uidsLoaded.insert(uid);
        }
        else
        {
            _renderModel.clearCommonSampler(bindingIndex);
        }
    }



#if 0
    // Look for shared layers we need to remove because we own them,
    // requested them, and didn't get updates for them:
    for (unsigned i = SamplerBinding::SHARED; i < bindings.size(); ++i)
    {
        if (bindings[i].isActive() &&
            manifest.includes(bindings[i].sourceUID().get()) &&
            !uidsLoaded.contains(bindings[i].sourceUID().get()))
        {
            inheritCommonSampler(i);
        }
    }

    // Propagate changes we made down to this tile's children.
    if (_childrenReady)
    {
        for (int i = 0; i < 4; ++i)
        {
            TileNode* child = getSubTile(i);
            if (child)
            {
                child->refreshInheritedData(this, bindings);
            }
        }
    }

    if (newElevationData)
    {
        _context->getEngine()->getTerrain()->notifyTileUpdate(getKey(), this);
    }
#endif


    //if (newElevationData)
    //{
    //    auto updateElevation = [&](SharedGeometry& geom, const osg::Matrix& matrix)
    //        {
    //        };

    //    TypedNodeVisitor<SharedGeometry> visitor(updateElevation);
    //    this->accept(visitor);
    //}
}
