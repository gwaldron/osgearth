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
#include "TerrainRenderData"
#include "TileNode"
#include "SurfaceNode"

using namespace osgEarth::Drivers::RexTerrainEngine;

#undef  LC
#define LC "[DrawTileCommand] "


void
DrawTileCommand::draw(osg::RenderInfo& ri, DrawState& ds) const
{
    osg::State& state = *ri.getState();

    //OE_INFO << LC << "      TILE: " << _geom << std::endl;
        
    // Tile key encoding, if the uniform is required.
    if (ds._tileKeyUL >= 0 )
    {
        ds._ext->glUniform4fv(ds._tileKeyUL, 1, _keyValue.ptr());
    }

    // Elevation coefficients (can probably be terrain-wide)
    if (ds._elevTexelCoeffUL >= 0 && !ds._elevTexelCoeff.isSetTo(_elevTexelCoeff))
    {
        ds._ext->glUniform2fv(ds._elevTexelCoeffUL, 1, _elevTexelCoeff.ptr());
        ds._elevTexelCoeff = _elevTexelCoeff;
    }

    // Morphing constants for this LOD
    if (ds._morphConstantsUL >= 0 && !ds._morphConstants.isSetTo(_morphConstants))
    {
        ds._ext->glUniform2fv(ds._morphConstantsUL, 1, _morphConstants.ptr());
        ds._morphConstants = _morphConstants;
    }

    // MVM for this tile:
    state.applyModelViewMatrix(_modelViewMatrix);
    
    // MVM uniforms for GL3 core:
    if (state.getUseModelViewAndProjectionUniforms())
    {
        state.applyModelViewAndProjectionUniformsIfRequired();
    }

    // Apply samplers for this tile draw:
    const Samplers& samplers = _pass->_surrogateSamplers ? *_pass->_surrogateSamplers : _pass->_samplers;


    for(unsigned s=0; s<samplers.size(); ++s)
    {
        const Sampler& sampler = samplers[s];

        SamplerState& samplerState = ds._samplerState._samplers[s];

        if (sampler._texture.valid() && !samplerState._texture.isSetTo(sampler._texture))
        {
            state.setActiveTextureUnit((*ds._bindings)[s].unit());
            sampler._texture->apply(state);
            samplerState._texture = sampler._texture.get();
        }

        if (samplerState._matrixUL >= 0 && !samplerState._matrix.isSetTo(sampler._matrix))
        {
            ds._ext->glUniformMatrix4fv(samplerState._matrixUL, 1, GL_FALSE, sampler._matrix.ptr());
            samplerState._matrix = sampler._matrix;
        }

        // Need a special uniform for color parents.
        if (s == SamplerBinding::COLOR_PARENT)
        {
            if (ds._parentTextureExistsUL >= 0 && !ds._parentTextureExists.isSetTo(sampler._texture != 0L))
            {
                ds._ext->glUniform1f(ds._parentTextureExistsUL, sampler._texture.valid() ? 1.0f : 0.0f);
                ds._parentTextureExists = sampler._texture.valid();
            }
        }
    }

    if (_drawCallback)
    {
        _drawCallback->draw(ri, _range);
    }

    else
    // If there's a geometry, draw it now:
    if (_geom)
    {
        // Set up the vertex arrays:
        _geom->drawVertexArraysImplementation(ri);

        // Draw as GL PATCHES?
        GLenum mode = _drawPatch ? GL_PATCHES : GL_TRIANGLES;

        for (unsigned i = 0; i < _geom->getNumPrimitiveSets(); ++i)
        {
            osg::DrawElementsUShort* de = static_cast<osg::DrawElementsUShort*>(_geom->getPrimitiveSet(i));
            osg::GLBufferObject* ebo = de->getOrCreateGLBufferObject(state.getContextID());
            state.bindElementBufferObject(ebo);
            if (ebo)
            {
                glDrawElements(mode, de->size(), GL_UNSIGNED_SHORT, (const GLvoid *)(ebo->getOffset(de->getBufferIndex())));
            }
        }
    }
}


void
TerrainRenderData::sortDrawCommands()
{
    for (LayerDrawableList::iterator i = _layerList.begin(); i != _layerList.end(); ++i)
    {
        i->get()->_tiles.sort();
    }
}

void
TerrainRenderData::setup(const MapFrame& frame, const RenderBindings& bindings, osg::StateSet* defaultStateSet)
{
    _bindings = &bindings;

    // Create a new State object to track sampler and uniform settings
    _drawState = new DrawState();
    _drawState->_bindings = &bindings;

    // The "default" layer if there's nothing else to draw.
    // For now we always add it ... later we will look for another color layer
    // to draw instead of this one because otherwise we are doing an unnecessary pass.
    LayerDrawable* defaultLayer = addLayer(0L);
    defaultLayer->setStateSet(defaultStateSet);

    // Make a drawable for each rendering pass (i.e. each render-able map layer).
    for(LayerVector::const_iterator i = frame.layers().begin();
        i != frame.layers().end();
        ++i)
    {
        Layer* layer = i->get();
        if (layer->getRenderType() != Layer::RENDERTYPE_NONE)
        {
            bool render = false;

            // If this is an image layer, check the enabled/visible states.
            ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer);
            if (imageLayer)
            {
                if (imageLayer->getEnabled() && imageLayer->getVisible())
                    render = true;
            }
            else
            {
                // all other types of renderable layers are true for now.
                render = true;
            }

            if (render)
            {
                addLayer(layer);
            }
        }
    }
}

LayerDrawable*
TerrainRenderData::addLayer(const Layer* layer)
{
    UID uid = layer ? layer->getUID() : -1;
    LayerDrawable* ld = new LayerDrawable();
    _layerList.push_back( ld );
    _layerMap[uid] = ld;
    ld->_layer = layer;
    ld->_imageLayer = dynamic_cast<const ImageLayer*>(layer);
    ld->_order = _layerList.size()-1;
    ld->_drawState = _drawState.get();
    if (layer)
        ld->setStateSet(layer->getStateSet());

    return ld;
}

LayerDrawable::LayerDrawable() :
_order(0),
_layer(0L),
_clearOsgState(false)
{
    setDataVariance(DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
}

void
LayerDrawable::drawImplementation(osg::RenderInfo& ri) const
{    
    DrawState& ds = *_drawState;

    // Make sure the draw state is up to date:
    ds.refresh(ri);

    if (ds._layerOrderUL >= 0)
    {
        ds._ext->glUniform1i(ds._layerOrderUL, (GLint)_order);
    }

    if (_layer)
    {
        if (ds._layerUidUL >= 0)
            ds._ext->glUniform1i(ds._layerUidUL,      (GLint)_layer->getUID());
        if (ds._layerOpacityUL >= 0 && _imageLayer)
            ds._ext->glUniform1f(ds._layerOpacityUL,  (GLfloat)_imageLayer->getOpacity());
        if (ds._layerMinRangeUL >= 0 && _imageLayer)
            ds._ext->glUniform1f(ds._layerMinRangeUL, (GLfloat)_imageLayer->getMinVisibleRange());
        if (ds._layerMaxRangeUL >= 0 && _imageLayer)
            ds._ext->glUniform1f(ds._layerMaxRangeUL, (GLfloat)_imageLayer->getMaxVisibleRange());
    }
    else
    {
        if (ds._layerUidUL >= 0)
            ds._ext->glUniform1i(ds._layerUidUL,      (GLint)-1);
        if (ds._layerOpacityUL >= 0)
            ds._ext->glUniform1f(ds._layerOpacityUL,  (GLfloat)1.0f);
        if (ds._layerMinRangeUL >= 0)
            ds._ext->glUniform1f(ds._layerMinRangeUL, (GLfloat)0.0f);
        if (ds._layerMaxRangeUL >= 0)
            ds._ext->glUniform1f(ds._layerMaxRangeUL, (GLfloat)FLT_MAX);
    }

    for (DrawTileCommands::const_iterator tile = _tiles.begin(); tile != _tiles.end(); ++tile)
    {
        tile->draw(ri, ds);
    }

    // If set, dirty all OSG state to prevent any leakage - this is sometimes
    // necessary when doing custom OpenGL within a Drawable.
    if (_clearOsgState)
    {
        ri.getState()->dirtyAllAttributes();
        ri.getState()->dirtyAllModes();
        ri.getState()->dirtyAllVertexArrays();
        
        // unbind local buffers when finished.
        ds._ext->glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
        ds._ext->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);

        ri.getState()->apply();
    }
}
